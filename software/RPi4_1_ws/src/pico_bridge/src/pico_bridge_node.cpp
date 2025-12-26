/*
 * PicoBridgeNode - SPI Bridge to Pico 2A Sensor Hub
 * 
 * ============================================================================
 * HARDWARE WIRING NOTE (PROTOTYPE V1 - WORKAROUND ACTIVE):
 * ============================================================================
 * The current prototype has CS wired to RPi4 Pin22 (GPIO25) instead of the
 * hardware SPI CE0 (GPIO8/Pin24). This requires SOFTWARE CS CONTROL.
 * 
 * We use `libgpiod` for efficient user-space GPIO control without sudo.
 * 
 * Current Wiring (Prototype V1):
 *   RPi4 Pin22 (GPIO25) <-> Pico Pin22 (GP17)  [CS - SOFTWARE CONTROLLED]
 *   RPi4 Pin23 (GPIO11) <-> Pico Pin24 (GP18)  [SCLK]
 *   RPi4 Pin19 (GPIO10) <-> Pico Pin21 (GP16)  [MOSI]
 *   RPi4 Pin21 (GPIO9)  <-> Pico Pin25 (GP19)  [MISO]
 * 
 * TODO (Prototype V2): Rewire CS to RPi4 Pin24 (CE0/GPIO8) for hardware CS.
 *                      Then remove SPI_NO_CS flag and GPIO25 software control.
 * ============================================================================
 */

#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <linux/spi/spidev.h>
#include <memory>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <gpiod.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "pico_bridge/msg/imu_batch.hpp"

#include "sensor_packet.h"

using namespace std::chrono_literals;

// Software CS pin (GPIO25 = RPi4 Pin22)
// WORKAROUND: Hardware uses GPIO25 instead of CE0 (GPIO8)
// TODO: Prototype V2 should use hardware CE0, then remove this workaround
#define CS_GPIO_PIN 25
#define CS_GPIO_CHIP "gpiochip0" // RPi4 usually maps GPIOs to gpiochip0

class PicoBridgeNode : public rclcpp::Node {
public:
  PicoBridgeNode() : Node("pico_bridge_node") {
    // Parameters
    this->declare_parameter("spi_device", "/dev/spidev0.0");
    this->declare_parameter("spi_speed", 10000000);
    spi_device_ = this->get_parameter("spi_device").as_string();
    spi_speed_ = this->get_parameter("spi_speed").as_int();

    // Publishers
    imu_batch_pub_ = this->create_publisher<pico_bridge::msg::ImuBatch>("/pico/imu_batch", 100);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/pico/mag", 10);
    press_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/pico/pressure", 10);
    temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/pico/temperature", 10);
    range_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/pico/range", 10);

    // Setup GPIO for software CS using libgpiod
    if (!setup_gpio_cs()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to setup GPIO CS (Pin22/GPIO25) with libgpiod");
      rclcpp::shutdown();
      return;
    }

    // Setup SPI
    if (!setup_spi()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to setup SPI");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "PicoBridge initialized with software CS on GPIO%d via libgpiod", CS_GPIO_PIN);

    // Timer (1000 Hz)
    timer_ = this->create_wall_timer(
        1ms, std::bind(&PicoBridgeNode::timer_callback, this));
  }

  ~PicoBridgeNode() {
    if (spi_fd_ >= 0) {
      close(spi_fd_);
    }
    cleanup_gpio_cs();
  }

private:
  std::string spi_device_;
  int spi_speed_;
  int spi_fd_ = -1;
  
  // libgpiod handles
  struct gpiod_chip *chip_ = nullptr;
  struct gpiod_line *line_ = nullptr;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<pico_bridge::msg::ImuBatch>::SharedPtr imu_batch_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr press_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  
  // Time sync
  bool first_packet_ = true;
  int64_t time_offset_ns_ = 0;

  // Previous sequence numbers to detect updates
  uint8_t last_mag_seq_ = 0;
  uint8_t last_baro_seq_ = 0;
  uint8_t last_tof_seq_ = 0;

  // ============================================================================
  // SOFTWARE CS CONTROL using libgpiod (Workaround for V1 Wiring)
  // ============================================================================
  bool setup_gpio_cs() {
    chip_ = gpiod_chip_open_by_name(CS_GPIO_CHIP);
    if (!chip_) {
      RCLCPP_ERROR(this->get_logger(), "libgpiod: Open chip failed");
      return false;
    }

    line_ = gpiod_chip_get_line(chip_, CS_GPIO_PIN);
    if (!line_) {
      RCLCPP_ERROR(this->get_logger(), "libgpiod: Get line failed");
      gpiod_chip_close(chip_);
      return false;
    }

    // Request line as output, initial value HIGH (inactive)
    int ret = gpiod_line_request_output(line_, "pico_bridge_cs", 1);
    if (ret < 0) {
      RCLCPP_ERROR(this->get_logger(), "libgpiod: Request output failed");
      gpiod_chip_close(chip_);
      return false;
    }
    
    return true;
  }

  void cleanup_gpio_cs() {
    if (line_) {
      gpiod_line_set_value(line_, 1); // Ensure CS inactive
      gpiod_line_release(line_);
    }
    if (chip_) {
      gpiod_chip_close(chip_);
    }
  }

  inline void cs_low() {
    if (line_) {
      gpiod_line_set_value(line_, 0);
    }
  }

  inline void cs_high() {
    if (line_) {
      gpiod_line_set_value(line_, 1);
    }
  }
  // ============================================================================

  bool setup_spi() {
    spi_fd_ = open(spi_device_.c_str(), O_RDWR);
    if (spi_fd_ < 0) {
      return false;
    }

    // Use SPI_MODE_3 (CPOL=1, CPHA=1) to match Pico PIO
    uint8_t mode = SPI_MODE_3 | SPI_NO_CS;  // Disable hardware CS, we use software CS
    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0) return false;
    if (ioctl(spi_fd_, SPI_IOC_RD_MODE, &mode) < 0) return false;

    uint8_t bits = 8;
    if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) return false;

    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_) < 0) return false;
    if (ioctl(spi_fd_, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed_) < 0) return false;

    return true;
  }

  uint32_t calculate_checksum(const SensorPacket* packet) {
    uint32_t checksum = 0;
    const uint8_t* ptr = (const uint8_t*)packet;
    for (size_t i = 0; i < sizeof(SensorPacket) - sizeof(uint32_t); ++i) {
        checksum ^= ptr[i];
    }
    return checksum;
  }

  void timer_callback() {
    SensorPacket rx_packet;
    SensorPacket tx_packet; // Start with zeros
    std::memset(&tx_packet, 0, sizeof(SensorPacket));

    struct spi_ioc_transfer tr;
    std::memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)&tx_packet;
    tr.rx_buf = (unsigned long)&rx_packet;
    tr.len = sizeof(SensorPacket);
    tr.speed_hz = (uint32_t)spi_speed_;
    tr.delay_usecs = 0;
    tr.bits_per_word = 8;

    // SOFTWARE CS CONTROL: Assert CS before transfer
    cs_low();
    
    int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    
    // SOFTWARE CS CONTROL: Deassert CS after transfer
    cs_high();

    if (ret < 1) {
      RCLCPP_WARN(this->get_logger(), "SPI transfer failed");
      return;
    }

    // Debug Sizes (Once)
    static bool printed_size = false;
    if (!printed_size) {
        RCLCPP_INFO(this->get_logger(), "SensorPacket Size: %zu", sizeof(SensorPacket));
        printed_size = true;
    }

    // Check Checksum (Masking LSB to handle 1-bit error)
    uint32_t calc_crc = calculate_checksum(&rx_packet);
    if ((calc_crc & 0xFFFFFFFE) != (rx_packet.checksum & 0xFFFFFFFE)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "CRC Fail: Calc=0x%08X != Rx=0x%08X", calc_crc, rx_packet.checksum);
        return; // Drop packet
    }

    // Debug Sequence Numbers (Throttled)
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
    //    "Seqs - Pkt: %u, Count: %u, Mag: %u", 
    //    rx_packet.packet_seq, rx_packet.valid_imu_count, rx_packet.mag_seq);

    // Time Sync
    if (first_packet_ && rx_packet.valid_imu_count > 0) {
        int64_t msg_timestamp_ns = rx_packet.imu_samples[0].timestamp_us * 1000;
        time_offset_ns_ = this->now().nanoseconds() - msg_timestamp_ns;
        first_packet_ = false;
    }

    // Process IMU Batch
    if (rx_packet.valid_imu_count > 0) {
        pico_bridge::msg::ImuBatch batch_msg;
        batch_msg.header.stamp = this->now(); // Use current time or reconstructed?
        batch_msg.header.frame_id = "imu_link_pico";
        
        for (int i = 0; i < rx_packet.valid_imu_count; i++) {
            auto imu_msg = sensor_msgs::msg::Imu();
            
            // Timestamp reconstruction
            int64_t sample_time_ns = rx_packet.imu_samples[i].timestamp_us * 1000 + time_offset_ns_;
            // imu_msg.header.stamp = rclcpp::Time(sample_time_ns); // Individual timestamp inside Imu message? 
            // sensor_msgs/Imu has header. We can set it.
            imu_msg.header.stamp = rclcpp::Time(sample_time_ns);
            imu_msg.header.frame_id = "imu_link_pico";
            
            imu_msg.linear_acceleration.x = rx_packet.imu_samples[i].accel[0];
            imu_msg.linear_acceleration.y = rx_packet.imu_samples[i].accel[1];
            imu_msg.linear_acceleration.z = rx_packet.imu_samples[i].accel[2];
            
            imu_msg.angular_velocity.x = rx_packet.imu_samples[i].gyro[0];
            imu_msg.angular_velocity.y = rx_packet.imu_samples[i].gyro[1];
            imu_msg.angular_velocity.z = rx_packet.imu_samples[i].gyro[2];
            
            // Push to Batch
            batch_msg.samples.push_back(imu_msg);
        }
        
        // Publish Batch (Once per packet)
        if (!batch_msg.samples.empty()) {
            imu_batch_pub_->publish(batch_msg);
        }
    }

    // Process Mag (Convert Gauss -> Tesla)
    if (rx_packet.mag_seq != last_mag_seq_) {
        auto mag_msg = sensor_msgs::msg::MagneticField();
        int64_t sample_time_ns = (rx_packet.valid_imu_count > 0) ? 
            (rx_packet.imu_samples[0].timestamp_us * 1000 + time_offset_ns_) : 
            this->now().nanoseconds();

        mag_msg.header.stamp = rclcpp::Time(sample_time_ns);
        mag_msg.header.frame_id = "mag_link_pico";
        mag_msg.magnetic_field.x = rx_packet.mag[0] * 1e-4f; // Gauss to Tesla
        mag_msg.magnetic_field.y = rx_packet.mag[1] * 1e-4f;
        mag_msg.magnetic_field.z = rx_packet.mag[2] * 1e-4f;
        
        mag_pub_->publish(mag_msg);
        last_mag_seq_ = rx_packet.mag_seq;
    }

    // Process Barometer
    if (rx_packet.baro_seq != last_baro_seq_) {
        auto press_msg = sensor_msgs::msg::FluidPressure();
        auto temp_msg = sensor_msgs::msg::Temperature();
        int64_t sample_time_ns = (rx_packet.valid_imu_count > 0) ? 
            (rx_packet.imu_samples[0].timestamp_us * 1000 + time_offset_ns_) : 
            this->now().nanoseconds();

        press_msg.header.stamp = rclcpp::Time(sample_time_ns);
        press_msg.header.frame_id = "baro_link_pico";
        press_msg.fluid_pressure = rx_packet.pressure; // Pascals

        temp_msg.header.stamp = rclcpp::Time(sample_time_ns);
        temp_msg.header.frame_id = "baro_link_pico";
        temp_msg.temperature = rx_packet.temperature; // Deg C
        
        temp_pub_->publish(temp_msg);
        
        last_baro_seq_ = rx_packet.baro_seq;
    }

    // Process ToF (Range)
    if (rx_packet.tof_seq != last_tof_seq_) {
        auto range_msg = sensor_msgs::msg::Range();
        int64_t sample_time_ns = (rx_packet.valid_imu_count > 0) ? 
            (rx_packet.imu_samples[0].timestamp_us * 1000 + time_offset_ns_) : 
            this->now().nanoseconds();

        range_msg.header.stamp = rclcpp::Time(sample_time_ns);
        range_msg.header.frame_id = "tof_link_pico";
        range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        range_msg.field_of_view = 0.44; // Approx 25 deg
        range_msg.min_range = 0.001; 
        range_msg.max_range = 4.0;
        range_msg.range = rx_packet.distance_mm / 1000.0f; // mm to m
        
        range_pub_->publish(range_msg);
        last_tof_seq_ = rx_packet.tof_seq;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PicoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
