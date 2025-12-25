#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <vector>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>

// Actuator Protocol
// TX: Magic(0xBABE), Throttle[6](u16), Flags(u8), Cksum(u8) -> 16 bytes
// RX: Magic(0xCAFE), RPM[6](s16), Voltage[6](u16), Current[6](u16), Temp[6](u8), Status(u8), Cksum(u8) -> 35 bytes (packed?). 
// Let's check firmware struct packing using byte inspection in verified scripts.
// In `verify_actuator.py`:
// magic(2), flags(1), state(1), rpm(12), voltage(2*1), current(2*1), temp(2*1)... wait.
// Let's consult `verify_actuator.py` logic again to match EXACTLY.
// Verify Script:
// magic(2), flags(1), state(1), rpm(12), V(2), I(2), T(2) = 22 bytes? 
// No, the firmware struct is:
// typedef struct {
//    uint8_t magic[2]; // 0xCA, 0xFE
//    uint16_t rpm[6];
//    uint16_t voltage[6];
//    uint16_t current[6];
//    uint8_t temp[6];
//    uint8_t status;
//    uint8_t checksum;
// } TelemetryPacket;
//
// Size = 2 + 12 + 12 + 12 + 6 + 1 + 1 = 46 bytes.
// Wait, `verify_actuator.py` only decoded 3 values for V, I, T in print, but unpacked "HHH" at offset 16? 
// Offset 16 is right after RPM (2+12=14). +2 magic = 16? No.
// Magic=2.
// RPM=12.
// 2+12 = 14.
// Offset 14 should be start of V.
// Let's stick to the FIRMWARE definition in `main.c` (pico_actuators).
// That is the Source of Truth.

#pragma pack(push, 1)
struct ActuatorPacket {
    uint8_t magic[2]; // 0xBA, 0xBE
    uint16_t throttle[6]; // 0-2047
    uint8_t flags; // 1=Arm, 2=Beep
    uint8_t checksum; 
};

struct TelemetryPacket {
    uint8_t magic[2]; // 0xCA, 0xFE
    uint16_t rpm[6];
    uint16_t voltage[6];
    uint16_t current[6];
    uint8_t temp[6];
    uint8_t status;
    uint8_t checksum;
};
#pragma pack(pop)

class ActuatorSpiNode : public rclcpp::Node {
public:
    ActuatorSpiNode() : Node("actuator_spi_node") {
        // Publishers
        rpm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/actuators/esc_rpm", 10);
        batt_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/actuators/battery", 10);

        // Subscribers
        cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pico/motor_commands", rclcpp::SensorDataQoS(),
            std::bind(&ActuatorSpiNode::cmdCallback, this, std::placeholders::_1));

        // SPI Init (SPI1 -> /dev/spidev1.0)
        // Ensure CS Pin 16 (GPIO 16) is handled by kernel driver logic (spidev1.0 uses chip select 0 of bus 1).
        // On RPi4, SPI1 CS0 is GPIO 18? No.
        // WIRING: Actuator CS is GPIO 16 (Pin 36).
        // RPi4 SPI1 Pins:
        // MOSI: GPIO 20 (Pin 38)
        // MISO: GPIO 19 (Pin 35)
        // SCLK: GPIO 21 (Pin 40)
        // CS0:  GPIO 18 (Pin 12)
        // CS1:  GPIO 17 (Pin 11)
        // CS2:  GPIO 16 (Pin 36) -> Valid for SPI1.
        // So we need `/dev/spidev1.2` or manually control GPIO 16.
        // `verify_actuator.py` used MANUAL CS on GPIO 16.
        // We will stick to MANUAL CS for consistency and control.
        
        setupManualCS(16);
        spi_fd_ = open("/dev/spidev1.0", O_RDWR); // Bus 1, Device 0 (We ignore kernel CS)
        if (spi_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/spidev1.0");
            return;
        }
        
        uint32_t speed = 1000000;
        uint8_t mode = 0;
        uint8_t bits = 8;
        ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode);
        ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

        // Loop Timer (1kHz)
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000), 
            std::bind(&ActuatorSpiNode::spiLoop, this));

        // Init Packet
        tx_pkt_.magic[0] = 0xBA;
        tx_pkt_.magic[1] = 0xBE;
    }

    ~ActuatorSpiNode() {
        if (spi_fd_ >= 0) close(spi_fd_);
    }

private:
    void setupManualCS(int pin) {
        // Simple Sysfs export (Production should use libgpiod)
        int fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd >= 0) {
            std::string s = std::to_string(pin);
            write(fd, s.c_str(), s.length());
            close(fd);
        }
        usleep(10000);
        std::string dir = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
        fd = open(dir.c_str(), O_WRONLY);
        if (fd >= 0) { write(fd, "out", 3); close(fd); }
        
        std::string val = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
        cs_fd_ = open(val.c_str(), O_RDWR);
        setCS(1);
    }

    void setCS(int val) {
        if (cs_fd_ >= 0) {
            char c = val ? '1' : '0';
            write(cs_fd_, &c, 1);
        }
    }

    void cmdCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) return;
        
        for(int i=0; i<6; i++) {
            // Scale floats (0.0-1.0 or PWMus?) -> To DShot 0-2047
            // Assuming input is 0.0 to 1.0 (Throttle %)
            // Or input is PWM 1000-2000?
            // "actuator_bridge_node.cpp" passes raw data.
            // Let's assume input is 0.0 -> 1.0 for now, scale to 48-2047.
            // Or if DShot, 0 is Disarm. 48 is Min spin.
            
            float cmd = msg->data[i];
            uint16_t dshot_val = 0;
            if (cmd > 0.01f) {
                dshot_val = 48 + (uint16_t)(cmd * 2000.0f);
                if (dshot_val > 2047) dshot_val = 2047;
            }
            tx_pkt_.throttle[i] = dshot_val;
        }
        
        // Flags
        // If any throttle > 0, Arm?
        // Ideally we need an ARMING topic. For now always arm if sending commands?
        // Or receive "Flags" in MultiArray?
        // Default to ARMED (1) to enable motors if command > 0.
        tx_pkt_.flags = 1; 
    }

    void spiLoop() {
        // Update Checksum
        uint8_t sum = 0;
        uint8_t* b = (uint8_t*)&tx_pkt_;
        for(size_t i=0; i<sizeof(ActuatorPacket)-1; i++) sum ^= b[i];
        tx_pkt_.checksum = sum;

        // Transaction
        TelemetryPacket rx_pkt;
        memset(&rx_pkt, 0, sizeof(rx_pkt));
        
        struct spi_ioc_transfer tr;
        memset(&tr, 0, sizeof(tr));
        tr.tx_buf = (unsigned long)&tx_pkt_;
        tr.rx_buf = (unsigned long)&rx_pkt;
        tr.len = sizeof(ActuatorPacket); // Wait, we need to read Telemetry Packet Size?
        // Duplex means MISO/MOSI clock together.
        // We must clock out enough bytes for the LARGER packet (Telemetry).
        // Telemetry is 46 bytes. Actuator is 16 bytes.
        // So we must transfer 46 bytes. TX buffer must be padded.
        
        uint8_t tx_buf_padded[sizeof(TelemetryPacket)];
        memset(tx_buf_padded, 0, sizeof(tx_buf_padded));
        memcpy(tx_buf_padded, &tx_pkt_, sizeof(ActuatorPacket));
        
        tr.tx_buf = (unsigned long)tx_buf_padded;
        tr.len = sizeof(TelemetryPacket);

        setCS(0);
        ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
        setCS(1);

        // Process RX
        if (rx_pkt.magic[0] == 0xCA && rx_pkt.magic[1] == 0xFE) {
            // Publish RPM
            std_msgs::msg::Float32MultiArray rpm_msg;
            for(int i=0; i<6; i++) rpm_msg.data.push_back((float)rx_pkt.rpm[i]);
            rpm_pub_->publish(rpm_msg);
            
            // Publish Battery (Use Avg of Cell voltages?)
            // Telemetry has array of voltage/current/temp per ESC?
            // "voltage[6]" -> Input voltage seen by ESC.
            // Use Motor 1 as reference.
            sensor_msgs::msg::BatteryState bat_msg;
            bat_msg.header.stamp = this->now();
            bat_msg.voltage = (float)rx_pkt.voltage[0] / 100.0f; // 10mV unit?
            bat_msg.current = (float)rx_pkt.current[0] / 100.0f; // 10mA unit?
            batt_pub_->publish(bat_msg);
        }
    }

    int spi_fd_ = -1;
    int cs_fd_ = -1;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batt_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    ActuatorPacket tx_pkt_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorSpiNode>());
    rclcpp::shutdown();
    return 0;
}
