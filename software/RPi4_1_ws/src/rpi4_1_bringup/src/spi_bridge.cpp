#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
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

// 9DoF 22-byte Packet Format:
// [0-3]: Header AA BB CC DD
// [4-15]: IMU raw: GX_L, GX_H, GY_L, GY_H, GZ_L, GZ_H, AX_L, AX_H, AY_L, AY_H, AZ_L, AZ_H
// [16-21]: Mag raw: MX_H, MX_L, MY_H, MY_L, MZ_H, MZ_L (Big Endian from sensor)
#define PACKET_BYTES 22
#define IMU_DATA_BYTES 12
#define MAG_DATA_BYTES 6

// ISM330DHCX Sensitivities
#define ISM330_SENS_16G     (0.488f / 1000.0f * 9.81f)  // mg/LSB -> m/s²
#define ISM330_SENS_2000DPS (70.0f / 1000.0f * 0.0174533f)  // mdps/LSB -> rad/s

// MMC5983MA Sensitivity (±8G = 16G range, 16-bit = 65536 counts)
// 1 LSB = 16G / 65536 ≈ 0.000244 Gauss = 0.0244 uT
#define MMC5983_OFFSET 32768
#define MMC5983_SENS_UT (0.0244f)

// Header: 0xAABBCCDD
static const uint8_t HEADER[4] = {0xAA, 0xBB, 0xCC, 0xDD};

class SysfsGPIO {
public:
    SysfsGPIO(int pin, bool input) : pin_(pin) {
        // 1. Try to Unexport first to clear any stale state
        int fd_un = open("/sys/class/gpio/unexport", O_WRONLY);
        if (fd_un >= 0) {
            std::string s = std::to_string(pin);
            write(fd_un, s.c_str(), s.length());
            close(fd_un);
        }
        usleep(100000); // Wait 100ms for cleanup

        // 2. Export
        int fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd >= 0) {
            std::string s = std::to_string(pin);
            write(fd, s.c_str(), s.length());
            close(fd);
        }
        
        // 3. Wait for filesystem to create the gpio directory (up to 1s)
        std::string path_dir = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
        int retries = 0;
        while (access(path_dir.c_str(), F_OK) == -1 && retries < 10) {
            usleep(100000); // 100ms
            retries++;
        }

        // 4. Direction
        fd = open(path_dir.c_str(), O_WRONLY);
        if (fd >= 0) {
            if (input) write(fd, "in", 2);
            else       write(fd, "out", 3);
            close(fd);
        } else {
             std::cerr << "Failed to open GPIO Direction: " << path_dir << std::endl;
        }

        // 5. Edge (for Interrupts)
        if (input) {
            std::string path_edge = "/sys/class/gpio/gpio" + std::to_string(pin) + "/edge";
            fd = open(path_edge.c_str(), O_WRONLY);
            if (fd >= 0) {
                write(fd, "rising", 6); // Trigger on Rising Edge (Data Ready)
                close(fd);
            } else {
                 std::cerr << "Failed to open GPIO Edge: " << path_edge << std::endl;
            }
        }

        // 6. Value FD (Keep open)
        std::string path_val = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
        fd_ = open(path_val.c_str(), O_RDWR);
        if (fd_ < 0) {
             std::cerr << "Failed to open GPIO Value: " << path_val << std::endl;
        }
    }

    ~SysfsGPIO() { 
        if (fd_ >= 0) close(fd_); 
        // Optional: unexport on destruction? Better to leave it exported to avoid thrashing.
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
        imu_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pico/imu_batch", 10);
        
        // Pin 31 (GPIO 6) = Data Ready Input
        gpio_ready_ = std::make_unique<SysfsGPIO>(6, true);

        // Pin 29 (GPIO 5) = Manual CS Output
        // Initialize HIGH (Idle)
        gpio_cs_ = std::make_unique<SysfsGPIO>(5, false);
        gpio_cs_->setValue(1);
        
        // SPI0
        spi_fd_ = open("/dev/spidev0.0", O_RDWR);
        if (spi_fd_ < 0) RCLCPP_ERROR(this->get_logger(), "SPI Open Failed");
        
        uint32_t speed = 8000000; // 8MHz
        uint8_t mode = 0;
        uint8_t bits = 8;
        // NOTE: We use Manual CS (GPIO 5), ignoring whatever CE0 does.
        // Even if kernel toggles CE0 (connected to nowhere), our GPIO 5 controls the Pico.
        
        ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode);
        ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

        // Core Affinity (Core 3)
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(3, &cpuset);
        pthread_t current_thread = pthread_self();
        if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Core 3 affinity");
        } else {
            RCLCPP_INFO(this->get_logger(), "Locked to Core 3");
        }

        // Start Polling Thread
        running_ = true;
        poll_thread_ = std::thread(&SpiBridgeNode::pollLoop, this);
    }

    ~SpiBridgeNode() {
        running_ = false;
        if (poll_thread_.joinable()) poll_thread_.join();
        if (spi_fd_ >= 0) close(spi_fd_);
    }

private:
    void pollLoop() {
        fprintf(stderr, "Trace: pollLoop started\n");
        fflush(stderr);

        struct pollfd pfd;
        pfd.fd = gpio_ready_->get_fd();
        pfd.events = POLLPRI; // Interrupt

        std::vector<uint8_t> rx(PACKET_BYTES); 
        std::vector<uint8_t> tx(PACKET_BYTES, 0);

        // Dummy read to clear initial state
        gpio_ready_->clear_interrupt();

        // KICKSTART DEADLOCK CHECK
        if (gpio_ready_->get_fd() >= 0) {
            lseek(gpio_ready_->get_fd(), 0, SEEK_SET);
            char val_buf[2] = {0};
            read(gpio_ready_->get_fd(), val_buf, 2);
            if (val_buf[0] == '1') {
                 fprintf(stderr, "Trace: Startup Line HIGH, kickstarting\n");
                 performRead(tx, rx);
            }
        }
        
        fprintf(stderr, "Trace: pollLoop entering while\n");
        fflush(stderr);

        while (running_ && rclcpp::ok()) {
            fprintf(stderr, "Trace: Calling performRead\n");
            fflush(stderr);
            performRead(tx, rx);
            usleep(100000);  // 100ms between reads
        }
        fprintf(stderr, "Trace: pollLoop exited\n");
    }

    void performRead(std::vector<uint8_t>& tx, std::vector<uint8_t>& rx) {
         std::cerr << "Trace: performRead start" << std::endl;
         
         // Manual CS Assert (GPIO 5)
         gpio_cs_->setValue(0);
         
         // Setup Delay: 50us
         usleep(50); 
         
         struct spi_ioc_transfer tr;
         memset(&tr, 0, sizeof(tr));
         tr.tx_buf = (unsigned long)tx.data();
         tr.rx_buf = (unsigned long)rx.data();
         tr.len = rx.size();
         // 1MHz Target Speed
         tr.speed_hz = 1000000; 
         tr.bits_per_word = 8;
         
         std::cerr << "Trace: calling ioctl" << std::endl;
         int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
         std::cerr << "Trace: ioctl returned " << ret << std::endl;
         
         // Manual CS Deassert (GPIO 5)
         gpio_cs_->setValue(1);
         
         // Post-Transaction Delay: 6ms for Pico Re-arm
         usleep(6000);
         
         processData(rx);
    }

    void processData(const std::vector<uint8_t>& rx) {
        std::cerr << "RX[0-3]: " 
                  << std::hex << (int)rx[0] << " " << (int)rx[1] << " " 
                  << (int)rx[2] << " " << (int)rx[3] << std::dec << std::endl;

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
            
            // Convert to physical units
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
            float mag[3] = {
                ((float)((int16_t)(mx_raw - MMC5983_OFFSET))) * MMC5983_SENS_UT,
                ((float)((int16_t)(my_raw - MMC5983_OFFSET))) * MMC5983_SENS_UT,
                ((float)((int16_t)(mz_raw - MMC5983_OFFSET))) * MMC5983_SENS_UT
            };
            
            static int log_cnt = 0;
            if (log_cnt++ % 10 == 0) {
                 RCLCPP_INFO(this->get_logger(), "A:[%.2f,%.2f,%.2f] G:[%.2f,%.2f,%.2f] M:[%.1f,%.1f,%.1f]",
                             accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2],
                             mag[0], mag[1], mag[2]);
            }
                        
            // Publish [ax, ay, az, gx, gy, gz, mx, my, mz]
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]};
            imu_pub_->publish(msg);
        } else {
             std::cerr << "Bad Header" << std::endl;
        }
    }

    std::unique_ptr<SysfsGPIO> gpio_ready_;
    std::unique_ptr<SysfsGPIO> gpio_cs_;
    int spi_fd_ = -1;
    std::thread poll_thread_;
    std::atomic<bool> running_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr imu_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpiBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
