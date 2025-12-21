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

#define PAYLOAD_FLOATS 24
#define PAYLOAD_BYTES (PAYLOAD_FLOATS * sizeof(float)) // 96 bytes

// Header: 0xAABBCCDD
static const uint8_t HEADER[4] = {0xAA, 0xBB, 0xCC, 0xDD};

class SysfsGPIO {
public:
    SysfsGPIO(int pin, bool input) : pin_(pin) {
        // Export
        int fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd >= 0) {
            std::string s = std::to_string(pin);
            write(fd, s.c_str(), s.length());
            close(fd);
        }

        // Direction
        std::string path_dir = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
        fd = open(path_dir.c_str(), O_WRONLY);
        if (fd >= 0) {
            if (input) write(fd, "in", 2);
            else       write(fd, "out", 3);
            close(fd);
        } else {
             std::cerr << "Failed to open GPIO Direction: " << path_dir << std::endl;
        }

        // Edge (for Interrupts)
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

        // Value FD (Keep open)
        std::string path_val = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
        fd_ = open(path_val.c_str(), O_RDWR);
        if (fd_ < 0) {
             std::cerr << "Failed to open GPIO Value: " << path_val << std::endl;
        }
    }

    ~SysfsGPIO() { if (fd_ >= 0) close(fd_); }

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
        
        uint32_t speed = 8000000; // 8MHz (Lower slightly for stability first)
        uint8_t mode = 0;
        uint8_t bits = 8;
        // Disable Kernel CS (We use manual)
        // SPI_NO_CS might be needed in mode if driver supports it, 
        // but otherwise ignoring CE0 is fine if disconnected.
        
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
        struct pollfd pfd;
        pfd.fd = gpio_ready_->get_fd();
        pfd.events = POLLPRI; // Interrupt

        std::vector<uint8_t> rx(PAYLOAD_BYTES + 4); 
        std::vector<uint8_t> tx(PAYLOAD_BYTES + 4, 0);

        // Dummy read to clear initial state
        gpio_ready_->clear_interrupt();

        // KICKSTART DEADLOCK CHECK
        if (gpio_ready_->get_fd() >= 0) {
            lseek(gpio_ready_->get_fd(), 0, SEEK_SET);
            char val_buf[2] = {0};
            read(gpio_ready_->get_fd(), val_buf, 2);
            if (val_buf[0] == '1') {
                 RCLCPP_WARN(this->get_logger(), "Startup: Line is HIGH. Kickstarting SPI read.");
                 performRead(tx, rx);
            }
        }

        while (running_ && rclcpp::ok()) {
            // Block waiting for GPIO Rise
            int ret = poll(&pfd, 1, 100); // 100ms timeout for faster fallback
            
            // If edge triggered, clear and read
            if (ret > 0 && (pfd.revents & POLLPRI)) {
                gpio_ready_->clear_interrupt();
                performRead(tx, rx);
            } else {
                // Fallback: If timeout, check raw level
                // This catches missed edges due to race conditions
                lseek(gpio_ready_->get_fd(), 0, SEEK_SET);
                char val_buf[2] = {0};
                read(gpio_ready_->get_fd(), val_buf, 2);
                if (val_buf[0] == '1') {
                    // Line is HIGH but we missed the edge
                    performRead(tx, rx);
                } else {
                    // Line is LOW, truly waiting
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for Data Ready signal...");
                }
            }
        }
    }

    void performRead(std::vector<uint8_t>& tx, std::vector<uint8_t>& rx) {
         // Manual CS Assert
         gpio_cs_->setValue(0);
         
         // Setup Delay: Give Pico time to trigger IRQ and start DMA
         usleep(500); 
         
         struct spi_ioc_transfer tr;
         memset(&tr, 0, sizeof(tr));
         tr.tx_buf = (unsigned long)tx.data();
         tr.rx_buf = (unsigned long)rx.data();
         tr.len = rx.size();
         // Ultra-Low Speed: 10kHz to debug Logic vs Noise
         tr.speed_hz = 10000; 
         tr.bits_per_word = 8;
         
         ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
         
         // Manual CS Deassert
         gpio_cs_->setValue(1);
         
         processData(rx);
    }

    void processData(const std::vector<uint8_t>& rx) {
        // DEBUG: Log first 8 bytes every 100th read
        static int debug_cnt = 0;
        if (debug_cnt++ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "RX[0-7]: %02X %02X %02X %02X %02X %02X %02X %02X",
                rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
        }

        // Expected layout after 4-byte header (0xAABBCCDD):
        // floats[0]=counter, [1]=temp, [2-4]=accel, [5-7]=gyro, [8-10]=mag
        if (rx[0] == HEADER[0] && rx[1] == HEADER[1] && rx[2] == HEADER[2] && rx[3] == HEADER[3]) {
            // Interpret payload as floats
            // Offset 4 bytes (Header) -> Start of Floats
            const float* payload = reinterpret_cast<const float*>(rx.data() + 4);
            
            RCLCPP_INFO(this->get_logger(), "F: Cnt:%.0f T:%.1f A:[%.2f %.2f %.2f] G:[%.2f %.2f %.2f]",
                        payload[0], payload[1],
                        payload[2], payload[3], payload[4],
                        payload[5], payload[6], payload[7]);
                        
            std_msgs::msg::Float32MultiArray msg;
            // Copy floats from payload to msg.data
            msg.data.assign(payload, payload + PAYLOAD_FLOATS);
            imu_pub_->publish(msg);
        } else {
             static int err_cnt = 0;
             if (err_cnt++ % 100 == 0) {
                 RCLCPP_WARN(this->get_logger(), "Bad Header: %02X %02X", rx[0], rx[1]);
             }
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
