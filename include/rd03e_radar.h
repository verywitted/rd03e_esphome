#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

class RD03ERadar {
public:
  RD03ERadar();
  
  // Initialize the radar with UART pins
  void begin(int rx_pin, int tx_pin, int baud_rate = 256000);
  
  // Update sensor readings (call in loop)
  void update();
  
  // Configuration methods
  void set_detection_distance(float distance);
  void set_sensitivity(uint8_t sensitivity);
  
  // State getter methods
  bool is_presence_detected() const;
  bool is_movement_detected() const;
  float get_distance() const;
  
private:
  // UART communication
  HardwareSerial *uart;
  
  // Buffer for receiving UART data
  static const uint8_t BUFFER_SIZE = 20;
  uint8_t buffer[BUFFER_SIZE];
  uint8_t buffer_index = 0;
  
  // Message frame details
  static const uint8_t FRAME_HEADER_1 = 0x55;
  static const uint8_t FRAME_HEADER_2 = 0xAA;
  static const uint8_t FRAME_LENGTH = 13;  // Total frame length
  
  // Status tracking
  bool frame_started = false;
  bool waiting_for_header2 = false;
  
  // Sensor states
  bool presence_detected = false;
  bool movement_detected = false;
  float distance_m = 0.0;
  
  // Configuration settings
  float detection_distance = 3.0;
  uint8_t sensitivity = 5;
  bool config_pending = true;
  
  // Private methods for data handling
  void process_byte(uint8_t data);
  void process_frame();
  void send_config_command();
  uint8_t calculate_checksum(const uint8_t* data, size_t length);
};