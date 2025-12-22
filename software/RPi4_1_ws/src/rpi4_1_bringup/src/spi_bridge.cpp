#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <vector>
#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/poll.h>
#include <pthread.h>
#include <sched.h> 

// [0-3]:   Header AA BB CC DD
// [4-15]:  IMU raw: 12 bytes
// [16-21]: Mag raw: 6 bytes
// [22-25]: Baro: 4 bytes
// [26-27]: ToF: 2 bytes
// [28-31]: Reserved/Padding (byte alignment)
#define PACKET_BYTES 32

// ISM330DHCX Sensitivities
#define ISM330_SENS_16G     (0.488f / 1000.0f * 9.81f)  // mg/LSB -> m/s²
#define ISM330_SENS_2000DPS (70.0f / 1000.0f * 0.0174533f)  // mdps/LSB -> rad/s

// MMC5983MA Sensitivity
#define MMC5983_OFFSET 32768
#define MMC5983_SENS_UT (0.0244f)
#define MMC5983_SENS_TESLA (MMC5983_SENS_UT / 1000000.0f) // uT -> Tesla

// Header: 0xAABBCCDD
static const uint8_t HEADER[4] = {0xAA, 0xBB, 0xCC, 0xDD};

class SysfsGPIO {
public:
    SysfsGPIO(int pin, bool input) : pin_(pin) {
        // 1. Try to Unexport first
        int fd_un = open("/sys/class/gpio/unexport", O_WRONLY);
        if (fd_un >= 0) {
            std::string s = std::to_string(pin);
            write(fd_un, s.c_str(), s.length());
            close(fd_un);
        }
        usleep(100000); 

        // 2. Export
        int fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd >= 0) {
            std::string s = std::to_string(pin);
            write(fd, s.c_str(), s.length());
            close(fd);
        }
        
        // 3. Wait for filesystem
        std::string path_dir = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
        int retries = 0;
        while (access(path_dir.c_str(), F_OK) == -1 && retries < 10) {
            usleep(100000); 
            retries++;
        }

        // 4. Direction
        fd = open(path_dir.c_str(), O_WRONLY);
        if (fd >= 0) {
            if (input) write(fd, "in", 2);
            else       write(fd, "out", 3);
            close(fd);
        }
        
        // 5. Edge
        if (input) {
            std::string path_edge = "/sys/class/gpio/gpio" + std::to_string(pin) + "/edge";
            fd = open(path_edge.c_str(), O_WRONLY);
            if (fd >= 0) {
                write(fd, "rising", 6); 
                close(fd);
            }
        }

        // 6. Value FD (Keep open)
        std::string path_val = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
        fd_ = open(path_val.c_str(), O_RDWR);
    }

    ~SysfsGPIO() { 
        if (fd_ >= 0) close(fd_); 
    }

    int get_fd() const { return fd_; }
    
    void setValue(int val) {
        if (fd_ >= 0) {
            std::string s = std::to_string(val);
            write(fd_, s.c_str(), s.length());
        }
    }
    
    void clear_interrupt() {
        if (fd_ >= 0) {
            lseek(fd_, 0, SEEK_SET);
            char buf[2];
            read(fd_, buf, 2);
        }
    }

private:
    int pin_;
    int fd_ = -1;
};

class SpiBridgeNode : public rclcpp::Node {
public:
    SpiBridgeNode() : Node("spi_bridge_node") {
        // Messages Publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/pico/imu", 10);
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/pico/mag", 10);
        baro_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/pico/pressure", 10);
        tof_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/pico/TOF", 10);
        
        // GPIO & SPI setup
        gpio_ready_ = std::make_unique<SysfsGPIO>(6, true);
        gpio_cs_ = std::make_unique<SysfsGPIO>(5, false);
        gpio_cs_->setValue(1);
        
        spi_fd_ = open("/dev/spidev0.0", O_RDWR);
        if (spi_fd_ < 0) RCLCPP_ERROR(this->get_logger(), "SPI Open Failed");
        
        uint32_t speed = 8000000; // 8MHz
        uint8_t mode = 0;
        uint8_t bits = 8;
        ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode);
        ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

        // Core 3 Affinity
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(3, &cpuset);
        pthread_t current_thread = pthread_self();
        pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);

        // Start Poll Loop
        running_ = true;
        std::cerr << "Node Initialized. Starting Poll Loop..." << std::endl;
        poll_thread_ = std::thread(&SpiBridgeNode::pollLoop, this);
    }

    ~SpiBridgeNode() {
        running_ = false;
        if (poll_thread_.joinable()) poll_thread_.join();
        if (spi_fd_ >= 0) close(spi_fd_);
    }

private:
    void pollLoop() {
        std::cerr << "Poll Loop Started." << std::endl;
        struct pollfd pfd;
        pfd.fd = gpio_ready_->get_fd();
        pfd.events = POLLPRI; 

        std::vector<uint8_t> rx(PACKET_BYTES); 
        std::vector<uint8_t> tx(PACKET_BYTES, 0);

        gpio_ready_->clear_interrupt();
        
        std::cerr << "Entering Main Loop..." << std::endl;
        while (running_ && rclcpp::ok()) {
             // Blind Polling Mode (IRQ line broken?)
             // poll(&pfd, 1, 100); 
             usleep(500); // Poll at ~2kHz
             performRead(tx, rx);
        }
    }

    void performRead(std::vector<uint8_t>& tx, std::vector<uint8_t>& rx) {
         gpio_cs_->setValue(0);
         usleep(50); 
         
         struct spi_ioc_transfer tr;
         memset(&tr, 0, sizeof(tr));
         tr.tx_buf = (unsigned long)tx.data();
         tr.rx_buf = (unsigned long)rx.data();
         tr.len = rx.size();
         tr.speed_hz = 1000000; 
         tr.bits_per_word = 8;
         
         ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
         
         gpio_cs_->setValue(1);
         usleep(50); // Minimal delay for signal integrity
         
         processData(rx);
    }

    void processData(const std::vector<uint8_t>& rx) {
        if (rx[0] == HEADER[0] && rx[1] == HEADER[1] && rx[2] == HEADER[2] && rx[3] == HEADER[3]) {
            // Parse IMU (Little Endian): GX, GY, GZ, AX, AY, AZ
            int16_t gx = (int16_t)(rx[4] | (rx[5] << 8));
            int16_t gy = (int16_t)(rx[6] | (rx[7] << 8));
            int16_t gz = (int16_t)(rx[8] | (rx[9] << 8));
            int16_t ax = (int16_t)(rx[10] | (rx[11] << 8));
            int16_t ay = (int16_t)(rx[12] | (rx[13] << 8));
            int16_t az = (int16_t)(rx[14] | (rx[15] << 8));
            
            // Parse Mag (Big Endian): MX, MY, MZ
            uint16_t mx_raw = ((uint16_t)rx[16] << 8) | rx[17];
            uint16_t my_raw = ((uint16_t)rx[18] << 8) | rx[19];
            uint16_t mz_raw = ((uint16_t)rx[20] << 8) | rx[21];
            
            // Parse Baro
            uint32_t p_raw = ((uint32_t)rx[22] << 24) | ((uint32_t)rx[23] << 16) | 
                             ((uint32_t)rx[24] << 8) | rx[25];
            float pressure = *reinterpret_cast<float*>(&p_raw);
            
            // Parse ToF
            uint16_t distance_mm = ((uint16_t)rx[26] << 8) | rx[27];
            
            // Physical Units
            float gyro[3] = {
                (float)gx * ISM330_SENS_2000DPS,
                (float)gy * ISM330_SENS_2000DPS,
                (float)gz * ISM330_SENS_2000DPS
            };
            float accel[3] = {
                (float)ax * ISM330_SENS_16G,
                (float)ay * ISM330_SENS_16G,
                (float)az * ISM330_SENS_16G
            };
            // Mag in Tesla
            float mag[3] = {
                ((float)((int16_t)(mx_raw - MMC5983_OFFSET))) * MMC5983_SENS_TESLA,
                ((float)((int16_t)(my_raw - MMC5983_OFFSET))) * MMC5983_SENS_TESLA,
                ((float)((int16_t)(mz_raw - MMC5983_OFFSET))) * MMC5983_SENS_TESLA
            };
            
            rclcpp::Time now = this->get_clock()->now();

            // 1. Publish IMU
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = now;
            imu_msg.header.frame_id = "pico_imu_link";
            imu_msg.linear_acceleration.x = accel[0];
            imu_msg.linear_acceleration.y = accel[1];
            imu_msg.linear_acceleration.z = accel[2];
            imu_msg.angular_velocity.x = gyro[0];
            imu_msg.angular_velocity.y = gyro[1];
            imu_msg.angular_velocity.z = gyro[2];
            imu_msg.orientation_covariance[0] = -1.0; // Orientation not available
            imu_pub_->publish(imu_msg);

            // 2. Publish Mag
            sensor_msgs::msg::MagneticField mag_msg;
            mag_msg.header.stamp = now;
            mag_msg.header.frame_id = "pico_mag_link";
            mag_msg.magnetic_field.x = mag[0];
            mag_msg.magnetic_field.y = mag[1];
            mag_msg.magnetic_field.z = mag[2];
            mag_pub_->publish(mag_msg);

            // 3. Publish Pressure
            sensor_msgs::msg::FluidPressure baro_msg;
            baro_msg.header.stamp = now;
            baro_msg.header.frame_id = "pico_baro_link";
            baro_msg.fluid_pressure = pressure;
            baro_msg.variance = 0.0;
            baro_pub_->publish(baro_msg);

            // 4. Publish Range
            sensor_msgs::msg::Range tof_msg;
            tof_msg.header.stamp = now;
            tof_msg.header.frame_id = "pico_tof_link";
            tof_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
            tof_msg.field_of_view = 0.44; // ~25 deg
            tof_msg.min_range = 0.001;
            tof_msg.max_range = 4.0;
            tof_msg.range = (float)distance_mm / 1000.0f;
            tof_pub_->publish(tof_msg);
            
            // Console Logging (Decimated)
            static int log_cnt = 0;
             if (log_cnt++ % 50 == 0) { // Log every 50 packets (~0.5s)
                  RCLCPP_INFO(this->get_logger(), "IMU:[%.2f,%.2f,%.2f] MAG:[%.6f] BARO:%.0f TOF:%.3fm",
                              accel[2], gyro[2], mag[2], pressure, tof_msg.range);
             }
        } else {
             // Header Check Failed
             static int err_cnt = 0;
             if (err_cnt++ % 50 == 0) {
                 RCLCPP_ERROR(this->get_logger(), "Bad Header: %02X %02X %02X %02X (Expected AA BB CC DD)",
                              rx[0], rx[1], rx[2], rx[3]);
             }
        }
    }

    std::unique_ptr<SysfsGPIO> gpio_ready_;
    std::unique_ptr<SysfsGPIO> gpio_cs_;
    int spi_fd_ = -1;
    std::thread poll_thread_;
    std::atomic<bool> running_;
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr baro_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpiBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
