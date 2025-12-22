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
        // ... (existing pollLoop logic is fine, will be kept by partial replacement if carefully targeted, but replace tool needs context)
        // I will replace the whole class to be safe or target specific blocks. 
        // The previous tool usage viewed the whole file, so I have context.
        // Let's replace the constructor to the end of performRead since I changed SysfsGPIO and performRead.
        
        // Actually, let's just use the replace setup I crafted above which covers SysfsGPIO and SpiBridgeNode constructor.
        // I also need performRead to have Manual CS.
        
        // Let's do SysfsGPIO first.
    }
// Wait, I can't put comments inside the ReplacementContent that are not in valid location.
// I will replace the SysfsGPIO class and SpiBridgeNode definitions up to performRead.

    void performRead(std::vector<uint8_t>& tx, std::vector<uint8_t>& rx) {
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
         
         ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
         
         // Manual CS Deassert (GPIO 5)
         gpio_cs_->setValue(1);
         
         // Post-Transaction Delay: 6ms for Pico Re-arm
         usleep(6000);
         
         processData(rx);
    }

    void processData(const std::vector<uint8_t>& rx) {
        // Force log to stderr every time for debugging
        // Use std::cerr to avoid buffering
        std::cerr << "RX[0-3]: " 
                  << std::hex << (int)rx[0] << " " << (int)rx[1] << " " 
                  << (int)rx[2] << " " << (int)rx[3] << std::dec << std::endl;

        // Expected layout after 4-byte header (0xAABBCCDD):
        // floats[0]=counter, ...
        if (rx[0] == HEADER[0] && rx[1] == HEADER[1] && rx[2] == HEADER[2] && rx[3] == HEADER[3]) {
            // Interpret payload as floats
            const float* payload = reinterpret_cast<const float*>(rx.data() + 4);
            
            // Log full data less frequently
            static int log_cnt = 0;
            if (log_cnt++ % 10 == 0) {
                 std::cerr << "Valid Data! Cnt: " << payload[0] << std::endl;
                 RCLCPP_INFO(this->get_logger(), "F: Cnt:%.0f ...", payload[0]);
            }
                        
            std_msgs::msg::Float32MultiArray msg;
            msg.data.assign(payload, payload + PAYLOAD_FLOATS);
            imu_pub_->publish(msg);
        } else {
             // Log bad header every time to confirm data flow
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
