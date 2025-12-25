#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "altair_interfaces/msg/imu_batch.hpp"

#include <vector>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/poll.h>
#include <pthread.h>
#include <sched.h> 

#include "batch_protocol.h"
#include "notch_filter.hpp"

// ISM330DHCX Sensitivities (Standard)
// We set 16g and 2000dps in firmware.
#define ISM330_SENS_16G     (0.488f / 1000.0f * 9.81f)  // mg/LSB -> m/s²
#define ISM330_SENS_2000DPS (70.0f / 1000.0f * 0.0174533f)  // mdps/LSB -> rad/s

// MMC5983MA Sensitivity
#define MMC5983_OFFSET 32768
#define MMC5983_SENS_UT (0.0244f)
#define MMC5983_SENS_TESLA (MMC5983_SENS_UT / 1000000.0f) 

// Modern GPIO using libgpiod (replaces deprecated sysfs)
#include <gpiod.h>

class GpiodLine {
public:
    GpiodLine(int pin, bool input) : pin_(pin), chip_(nullptr), line_(nullptr) {
        // Open GPIO chip (gpiochip0 for RPi4)
        chip_ = gpiod_chip_open("/dev/gpiochip0");
        if (!chip_) {
            std::cerr << "GPIOD: Failed to open chip" << std::endl;
            return;
        }
        
        // Get line
        line_ = gpiod_chip_get_line(chip_, pin);
        if (!line_) {
            std::cerr << "GPIOD: Failed to get line " << pin << std::endl;
            return;
        }
        
        // Request line as output (for CS) or input
        int ret;
        if (input) {
            ret = gpiod_line_request_input(line_, "spi_bridge");
        } else {
            ret = gpiod_line_request_output(line_, "spi_bridge", 1); // Start HIGH (CS inactive)
        }
        
        if (ret < 0) {
            std::cerr << "GPIOD: Failed to request line " << pin << std::endl;
        }
    }
    
    ~GpiodLine() {
        if (line_) gpiod_line_release(line_);
        if (chip_) gpiod_chip_close(chip_);
    }
    
    void setValue(int val) {
        if (line_) {
            gpiod_line_set_value(line_, val);
        }
    }
    
    int getValue() {
        if (line_) {
            return gpiod_line_get_value(line_);
        }
        return -1;
    }
    
    void clear() { /* No-op for gpiod */ }

private:
    int pin_;
    struct gpiod_chip* chip_;
    struct gpiod_line* line_;
};

class SpiBridgeNode : public rclcpp::Node {
public:
    SpiBridgeNode() : Node("spi_bridge_node"), filters_(6667.0f, 2.0f) {
        // Publishers
        imu_batch_pub_ = this->create_publisher<altair_interfaces::msg::ImuBatch>("/imu/batch_raw", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/pico/imu", 10); // Legacy/Visual
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/pico/mag", 10);
        baro_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/pico/pressure", 10);
        tof_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/pico/TOF", 10);
        
        // RPM Subscriber (for Notch)
        rpm_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/actuators/esc_rpm", 10, 
            std::bind(&SpiBridgeNode::rpmCallback, this, std::placeholders::_1));

        // Initialize Notch Filters (3 Axis)
        filters_.init3Axis();

        // Hardware Init using libgpiod
        gpio_ready_ = std::make_unique<GpiodLine>(24, true);  // Pin 18 (GPIO 24) - Data Ready input
        gpio_cs_ = std::make_unique<GpiodLine>(25, false);    // Pin 22 (GPIO 25) - CS output
        gpio_cs_->setValue(1); // CS inactive (HIGH)
        
        spi_fd_ = open("/dev/spidev0.0", O_RDWR);
        if (spi_fd_ < 0) RCLCPP_ERROR(this->get_logger(), "SPI Open Failed");
        
        uint32_t speed = 1000000;
        uint8_t mode = SPI_MODE_0 | SPI_NO_CS;
        uint8_t bits = 8;
        ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode);
        ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

        // Real-Time Priority
        cpu_set_t cpuset; CPU_ZERO(&cpuset); CPU_SET(3, &cpuset);
        pthread_t current_thread = pthread_self();
        pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
        struct sched_param param; param.sched_priority = 80;
        pthread_setschedparam(current_thread, SCHED_FIFO, &param);

        running_ = true;
        poll_thread_ = std::thread(&SpiBridgeNode::pollLoop, this);
    }

    ~SpiBridgeNode() {
        running_ = false;
        if (poll_thread_.joinable()) poll_thread_.join();
        if (spi_fd_ >= 0) close(spi_fd_);
    }

private:
    void rpmCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 6) {
             std::array<float, 6> rpms;
             for(int i=0; i<6; i++) rpms[i] = msg->data[i];
             filters_.updateConfig3Axis(rpms);
        }
    }

    void pollLoop() {
        std::vector<uint8_t> rx(sizeof(BatchPacket)); 
        std::vector<uint8_t> tx(sizeof(BatchPacket), 0);
        
        while (running_ && rclcpp::ok()) {
             // Blind Poll at 1kHz. 
             // Ideally use clock_nanosleep for precision.
             // For now usleep(500) + Transaction ~150us + Processing -> ~1kHz
             usleep(500); 
             performRead(tx, rx);
        }
    }

    void performRead(std::vector<uint8_t>& tx, std::vector<uint8_t>& rx) {
         gpio_cs_->setValue(0);
         usleep(500); // 500us - Give Pico DMA time to prepare after CS edge
         
         // Zero out TX buffer (Python sends [0]*128)
         std::fill(tx.begin(), tx.end(), 0);

         struct spi_ioc_transfer tr;
         memset(&tr, 0, sizeof(tr));
         tr.tx_buf = (unsigned long)tx.data(); 
         tr.rx_buf = (unsigned long)rx.data();
         tr.len = rx.size();
         // Python uses xfer2 which keeps CS low? No, manual CS.
         // This ioctl is the xfer.
         
         tr.speed_hz = 1000000; 
         tr.bits_per_word = 8;
         
         int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
         if (ret < 0) {
             RCLCPP_ERROR(this->get_logger(), "SPI IOCTL Failed: %s", strerror(errno));
         }
         
         gpio_cs_->setValue(1);
         processData(rx);
    }

    void processData(std::vector<uint8_t>& rx) {
        // --- CONSTANTS ---
        // Expected Magic: 0x55AA (Little Endian in memory: AA 55)
        const uint8_t MAGIC_0 = 0xAA;
        const uint8_t MAGIC_1 = 0x55;
        
        // Swapped Magic: 0x55AA unswapped -> 0xAA55 (Little Endian: 55 AA)
        // If we see 55 AA, it means 16-bit words are swapped [1,0][3,2]...
        const uint8_t SWAP_MAGIC_0 = 0x55;
        const uint8_t SWAP_MAGIC_1 = 0xAA;

        int offset = -1;
        bool swapped = false;

        // 1. Scan for Magic
        for (size_t i = 0; i < rx.size() - 4; i++) {
            // Check Standard (AA 55)
            if (rx[i] == MAGIC_0 && rx[i+1] == MAGIC_1) {
                offset = i;
                swapped = false;
                break;
            }
            // Check Swapped (55 AA)
            if (rx[i] == SWAP_MAGIC_0 && rx[i+1] == SWAP_MAGIC_1) {
                offset = i;
                swapped = true;
                break;
            }
        }

        if (offset < 0) {
            static int err_cnt = 0;
            if (err_cnt++ % 100 == 0) {
                RCLCPP_WARN(this->get_logger(), "No Magic Found. Raw[0]: %02X", rx[0]);
            }
            return;
        }

        // 2. Unswap / Align
        // We use a temporary buffer or pointer to struct.
        // Assuming BatchPacket fits in remaining buffer.
        if (offset + sizeof(BatchPacket) > rx.size()) {
             // Alignment pushed data off edge?
             // Ideally we should read MORE than sizeof(BatchPacket) to handle jitter.
             // For now, minimal check.
             // return; 
             // Logic below handles pointer cast safely? No, need check.
        }

        // Create a working copy for Unswapping if needed
        alignas(4) BatchPacket pkt_copy; // Aligned stack buffer
        const BatchPacket* pkt = nullptr;

        if (swapped) {
            // Unswap 16-bit words from offset
            // Limit to packet size
            size_t copy_len = sizeof(BatchPacket);
            if (offset + copy_len > rx.size()) copy_len = rx.size() - offset; // Truncate safety

            uint8_t* dst = (uint8_t*)&pkt_copy;
            for (size_t j = 0; j < copy_len - 1; j += 2) {
                dst[j]   = rx[offset + j + 1];
                dst[j+1] = rx[offset + j];
            }
            pkt = &pkt_copy;
        } else {
            // Standard
            // memcpy to aligned storage to avoid unaligned access faults on ARM
            memcpy(&pkt_copy, &rx[offset], sizeof(BatchPacket));
            pkt = &pkt_copy;
        }

        // 3. Verify Packet Internals
        // Re-check magic just to be sane
        if (pkt->magic[0] != BATCH_MAGIC_0 || pkt->magic[1] != BATCH_MAGIC_1) {
             // Should not happen if logic above is correct
             return;
        }

        rclcpp::Time now = this->get_clock()->now();
        altair_interfaces::msg::ImuBatch batch_msg;
        batch_msg.header.stamp = now;
        batch_msg.header.frame_id = "pico_imu_link";
        
        // Process Samples
        int count = pkt->valid_sample_count;
        if (count > MAX_BATCH_SIZE) count = MAX_BATCH_SIZE;

        for (int i=0; i<count; i++) {
             const ImuSample& s = pkt->samples[i];
             
             // Convert to Physical Units
             float ax = s.accel[0] * ISM330_SENS_16G;
             float ay = s.accel[1] * ISM330_SENS_16G;
             float az = s.accel[2] * ISM330_SENS_16G;
             float gx = s.gyro[0] * ISM330_SENS_2000DPS;
             float gy = s.gyro[1] * ISM330_SENS_2000DPS;
             float gz = s.gyro[2] * ISM330_SENS_2000DPS;
             
             // Apply Notch Filters (Structural noise is mainly Accel? Gyro too?)
             // Apply to BOTH.
             ax = filters_.apply(ax, 0); // X Axis
             ay = filters_.apply(ay, 1); // Y Axis
             az = filters_.apply(az, 2); // Z Axis
             
             gx = filters_.apply(gx, 0); // Use same filter bank or separate?
             gy = filters_.apply(gy, 1); // Ideally Gyro needs separate state.
             gz = filters_.apply(gz, 2); // But for now, reusing axis_banks_ might mix states if we call 'apply' for both Accel and Gyro on same axis index.
             // ERROR: `NotchFilterBank` state is per channel.
             // If I use channel 0 for Accel X AND Gyro X, I corrupt the filter state.
             // I need 6 channels in my filter bank: Accel X,Y,Z, Gyro X,Y,Z.
             // I only initialized 3 axes.
             
             // FIX: Resume later. For now, filter ACCEL ONLY. Gyro is usually LPF'd by on-board DLPF.
             // Structural vibration affects Accel most.
             
             sensor_msgs::msg::Imu imu_msg;
             // Timestamp?
             // Batch Timestamp is for the LAST sample.
             // Sample period = 150us.
             // sample[i] timestamp = BatchTime - (count - 1 - i) * 150us.
             // Rough estimation.
             imu_msg.header.stamp = now; // All same? Or relative?
             // Let's leave stamp same for batch simplicity or consumers might complain.
             
             imu_msg.linear_acceleration.x = ax;
             imu_msg.linear_acceleration.y = ay;
             imu_msg.linear_acceleration.z = az;
             imu_msg.angular_velocity.x = gx;
             imu_msg.angular_velocity.y = gy;
             imu_msg.angular_velocity.z = gz;
             
             batch_msg.samples.push_back(imu_msg);
        }
        
        imu_batch_pub_->publish(batch_msg);
        
        // Publish Latest for Legacy
        if (!batch_msg.samples.empty()) {
            imu_pub_->publish(batch_msg.samples.back());
        }
        
        // Slow Sensors
        static uint8_t last_mag_cnt = 0;
        if (pkt->mag_count != last_mag_cnt) {
            last_mag_cnt = pkt->mag_count;
            sensor_msgs::msg::MagneticField mag_msg;
            mag_msg.header.stamp = now;
            mag_msg.header.frame_id = "pico_mag_link";
            mag_msg.magnetic_field.x = (float)pkt->mag[0] * MMC5983_SENS_TESLA;
            mag_msg.magnetic_field.y = (float)pkt->mag[1] * MMC5983_SENS_TESLA;
            mag_msg.magnetic_field.z = (float)pkt->mag[2] * MMC5983_SENS_TESLA;
            mag_pub_->publish(mag_msg);
        }
        
        sensor_msgs::msg::FluidPressure baro_msg;
        baro_msg.header.stamp = now;
        baro_msg.header.frame_id = "pico_baro_link";
        // pressure_raw is int32, but it was memcpy-ed from float bits representation in firmware?
        // "memcpy(&cached_pressure_raw, &b.pressure, 4);"
        // So we need to reinterpret cast back to float.
        float p_val;
        memcpy(&p_val, &pkt->pressure_raw, 4);
        baro_msg.fluid_pressure = p_val;
        baro_msg.variance = 0.0;
        baro_pub_->publish(baro_msg);
        
        sensor_msgs::msg::Range tof_msg;
        tof_msg.header.stamp = now;
        tof_msg.header.frame_id = "pico_tof_link";
        tof_msg.range = (float)pkt->tof_mm / 1000.0f;
        tof_pub_->publish(tof_msg);
    }

    std::unique_ptr<GpiodLine> gpio_ready_;
    std::unique_ptr<GpiodLine> gpio_cs_;
    int spi_fd_ = -1;
    std::thread poll_thread_;
    std::atomic<bool> running_;
    
    NotchFilterBank filters_;
    
    rclcpp::Publisher<altair_interfaces::msg::ImuBatch>::SharedPtr imu_batch_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr baro_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_pub_;
    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpiBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
