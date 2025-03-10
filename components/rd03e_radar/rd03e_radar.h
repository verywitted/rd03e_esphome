#pragma once

#include <esphome/core/component.h>
#include <esphome/components/uart/uart.h>
#include <esphome/components/binary_sensor/binary_sensor.h>
#include <esphome/components/sensor/sensor.h>

namespace esphome {
namespace rd03e_radar {

class RD03ERadarSensor : public Component, public uart::UARTDevice {
 public:
  RD03ERadarSensor() = default;

  inline void set_uart_parent(uart::UARTComponent *parent) { uart::UARTDevice::set_uart_parent(parent); }

  void setup() override;
  void loop() override;
  void dump_config() override;
  inline float get_setup_priority() const override { return setup_priority::LATE; }

  inline void set_presence_sensor(binary_sensor::BinarySensor *presence_sensor) { presence_sensor_ = presence_sensor; }
  inline void set_movement_sensor(binary_sensor::BinarySensor *movement_sensor) { movement_sensor_ = movement_sensor; }
  inline void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }

  void set_detection_distance(float distance);
  void set_sensitivity(uint8_t sensitivity);

 protected:
  binary_sensor::BinarySensor *presence_sensor_{nullptr};
  binary_sensor::BinarySensor *movement_sensor_{nullptr};
  sensor::Sensor *distance_sensor_{nullptr};

  // Buffer for receiving UART data
  static const uint8_t BUFFER_SIZE = 20;
  uint8_t buffer_[BUFFER_SIZE];
  uint8_t buffer_index_{0};
  
  // Message frame details
  static const uint8_t FRAME_HEADER_1 = 0x55;
  static const uint8_t FRAME_HEADER_2 = 0xAA;
  static const uint8_t FRAME_LENGTH = 13;  // Total frame length
  
  // Status tracking
  bool frame_started_{false};
  bool waiting_for_header2_{false};
  
  // Configuration settings
  float detection_distance_{3.0};
  uint8_t sensitivity_{5};
  bool config_pending_{true};

  void process_byte_(uint8_t data);
  void process_frame_();
  void send_config_command_();
};

}  // namespace rd03e_radar
}  // namespace esphome