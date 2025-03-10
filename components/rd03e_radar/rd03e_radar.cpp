#include "rd03e_radar.h"
#include <esphome/core/log.h>

namespace esphome {
namespace rd03e_radar {

static const char *TAG = "rd03e_radar";

// Prevent multiple definition errors by making implementation-only functions
// Include guard
#ifndef RD03E_RADAR_CPP_
#define RD03E_RADAR_CPP_

void RD03ERadarSensor::setup() {
  ESP_LOGI(TAG, "Setting up RD03-E radar sensor...");
  
  // Set initial states
  if (this->presence_sensor_ != nullptr)
    this->presence_sensor_->publish_state(false);
  
  if (this->movement_sensor_ != nullptr)
    this->movement_sensor_->publish_state(false);
  
  if (this->distance_sensor_ != nullptr)
    this->distance_sensor_->publish_state(0.0);
  
  // Reset buffer
  this->buffer_index_ = 0;
  memset(this->buffer_, 0, BUFFER_SIZE);
}

void RD03ERadarSensor::loop() {
  // Send configuration if pending
  if (this->config_pending_) {
    this->send_config_command_();
    this->config_pending_ = false;
  }
  
  // Process incoming data
  while (available() > 0) {
    uint8_t data = read();
    this->process_byte_(data);
  }
}

void RD03ERadarSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "RD03-E Radar Sensor:");
  if (this->presence_sensor_ != nullptr) {
    LOG_BINARY_SENSOR("  ", "Presence Sensor", this->presence_sensor_);
  }
  if (this->movement_sensor_ != nullptr) {
    LOG_BINARY_SENSOR("  ", "Movement Sensor", this->movement_sensor_);
  }
  if (this->distance_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Distance Sensor", this->distance_sensor_);
  }
  ESP_LOGCONFIG(TAG, "  Detection Distance: %.1fm", this->detection_distance_);
  ESP_LOGCONFIG(TAG, "  Sensitivity: %d", this->sensitivity_);
}

void RD03ERadarSensor::set_detection_distance(float distance) {
  ESP_LOGI(TAG, "Setting detection distance to %.1fm", distance);
  this->detection_distance_ = distance;
  this->config_pending_ = true;
}

void RD03ERadarSensor::set_sensitivity(uint8_t sensitivity) {
  ESP_LOGI(TAG, "Setting sensitivity to %d", sensitivity);
  this->sensitivity_ = sensitivity;
  this->config_pending_ = true;
}

void RD03ERadarSensor::send_config_command_() {
  // Format: [0x55][0xAA][CMD][LEN][DATA...][CHECKSUM]
  uint8_t cmd[13];
  
  // Header
  cmd[0] = FRAME_HEADER_1;
  cmd[1] = FRAME_HEADER_2;
  
  // Command 0xA1 = Set Parameters
  cmd[2] = 0xA1;
  
  // Data length (8 bytes)
  cmd[3] = 0x08;
  
  // Data payload
  cmd[4] = (uint8_t)(this->detection_distance_ * 10);  // Distance in decimeters
  cmd[5] = this->sensitivity_;                         // Sensitivity level
  cmd[6] = 0x01;                                       // Enable continuous mode
  cmd[7] = 0x00;                                       // Reserved
  cmd[8] = 0x00;                                       // Reserved
  cmd[9] = 0x00;                                       // Reserved
  cmd[10] = 0x00;                                      // Reserved
  cmd[11] = 0x00;                                      // Reserved
  
  // Calculate checksum (XOR of all previous bytes)
  cmd[12] = 0x00;
  for (int i = 0; i < 12; i++) {
    cmd[12] ^= cmd[i];
  }
  
  // Send the command
  write_array(cmd, 13);
  ESP_LOGI(TAG, "Sent configuration command");
}

void RD03ERadarSensor::process_byte_(uint8_t data) {
  // Look for frame header sequence
  if (!this->frame_started_) {
    if (!this->waiting_for_header2_) {
      if (data == FRAME_HEADER_1) {
        this->waiting_for_header2_ = true;
        this->buffer_[0] = data;
        this->buffer_index_ = 1;
      }
    } else {
      if (data == FRAME_HEADER_2) {
        this->frame_started_ = true;
        this->buffer_[this->buffer_index_++] = data;
      } else {
        // Reset if the second byte is not the expected header
        this->waiting_for_header2_ = false;
        this->buffer_index_ = 0;
      }
    }
  } else {
    // Add byte to buffer
    this->buffer_[this->buffer_index_++] = data;
    
    // If we have received a complete frame
    if (this->buffer_index_ == FRAME_LENGTH) {
      this->process_frame_();
      
      // Reset for next frame
      this->frame_started_ = false;
      this->waiting_for_header2_ = false;
      this->buffer_index_ = 0;
    }
    
    // Prevent buffer overflow
    if (this->buffer_index_ >= BUFFER_SIZE) {
      this->frame_started_ = false;
      this->waiting_for_header2_ = false;
      this->buffer_index_ = 0;
    }
  }
}

void RD03ERadarSensor::process_frame_() {
  // Verify the frame with checksum
  uint8_t calculated_checksum = 0;
  for (int i = 0; i < FRAME_LENGTH - 1; i++) {
    calculated_checksum ^= this->buffer_[i];
  }
  
  // If checksum matches
  if (calculated_checksum == this->buffer_[FRAME_LENGTH - 1]) {
    // Check command type
    if (this->buffer_[2] == 0x01) {  // Status report
      // Extract presence state (0x01 = presence, 0x00 = no presence)
      bool presence = this->buffer_[4] == 0x01;
      if (this->presence_sensor_ != nullptr)
        this->presence_sensor_->publish_state(presence);
      
      // Extract movement state
      bool movement = this->buffer_[5] == 0x01;
      if (this->movement_sensor_ != nullptr)
        this->movement_sensor_->publish_state(movement);
      
      // Extract distance (bytes 6-7 contain distance in cm)
      float distance = (this->buffer_[6] + (this->buffer_[7] << 8)) / 100.0;  // Convert to meters
      if (distance > 0 && distance <= 8.0 && this->distance_sensor_ != nullptr) {  // Valid range check
        this->distance_sensor_->publish_state(distance);
      }
      
      ESP_LOGD(TAG, "Presence: %d, Movement: %d, Distance: %.2fm", 
              presence, movement, distance);
    }
  } else {
    ESP_LOGW(TAG, "Invalid checksum in received frame");
  }
}

#endif  // RD03E_RADAR_CPP_

}  // namespace rd03e_radar
}  // namespace esphome