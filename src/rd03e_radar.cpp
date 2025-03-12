#include "rd03e_radar.h"
//#include "rd03e_config.h"

#include <iostream>     // std::cout
#include <algorithm>    // std::copy
#include <vector> 

RD03ERadar::RD03ERadar() : uart(nullptr) {

  // Initialize variables
  buffer_index = 0;
  frame_started = false;
  waiting_for_header2 = false;
  presence_detected = false;
  movement_detected = false;
  distance_m = 0.0;
  config_pending = true;
  RD03EConfig = _config;  // Initialize the configuration object;
}

void RD03ERadar::begin(int rx_pin, int tx_pin, int baud_rate) {
  // Initialize the UART for communicating with the RD03-E
  uart = new HardwareSerial(2); // Use UART2
  uart->begin(baud_rate, SERIAL_8N1, rx_pin, tx_pin);
  
  Serial.println("RD03-E Radar initialized");
  
  // Clear the buffer
  buffer_index = 0;
  memset(buffer, 0, BUFFER_SIZE);
  
  // Send initial configuration
  //send_config_command();
}

void RD03ERadar::update() {
  // Send configuration if pending
  if (config_pending) {
    send_config_command();
    config_pending = false;
  }
  
  // Process any available data from the radar
  while (uart && uart->available() > 0) {
    uint8_t data = uart->read();
    if (BYPASS_MODE)
    if (data == -1){
      Serial.println("Failed to read from uart interface."); // Skip invalid data
    }
    process_byte(data);
  }
}

void RD03ERadar::set_detection_distance( distance) {
  if (distance >= 0.5 && distance <= 6.0) {
    this->_config = {600, 30, 425, 30, 1000};
    this->_config.modified.distanceSettings = true;
  }
    Serial.print("Setting detection distance to: ");
    Serial.println(distance);
  }
}

void RD03ERadar::set_sensitivity(uint8_t sensitivity) {
  if (sensitivity >= 1 && sensitivity <= 10) {
    this->sensitivity = sensitivity;
    this->config_pending = true;
    Serial.print("Setting sensitivity to: ");
    Serial.println(sensitivity);
  }
}

bool RD03ERadar::is_presence_detected() const {
  return presence_detected;
}

bool RD03ERadar::is_movement_detected() const {
  return movement_detected;
}

float RD03ERadar::get_distance() const {
  return distance_m;
}

void RD03ERadar::process_byte(uint8_t data) {
      if (data == DATA_FRAME_HEADER_1) {
        if (buffer[0] == DATA_FRAME_HEADER_1) {
          buffer[1] = data;
          buffer_index = 2;          
        }
        else {
          buffer[0] = data;
          buffer_index = 1;
        }
      }
     else {

           // If we have received a complete frame
      if (buffer_index == 6 && data == 0x55 && buffer[5] == 0x55) {
          buffer[buffer_index] = data;
          buffer_index = 0;
          process_frame();
        }
      else {
    // Add byte to buffer
          if (buffer_index > 1) {
            buffer[buffer_index] = data;
            buffer_index++;      
          }

        }

    // Prevent buffer overflow
    if (buffer_index >= BUFFER_SIZE) {
      buffer_index = 0;
    }
  }
}


void RD03ERadar::process_frame() {
  // Verify the frame with checksum
  if (buffer[0] == 0xAA && buffer[1] == 0xAA) {  // Status report
    // Extract presence state (0x01 = presence, 0x00 = no presence)
    presence_detected = buffer[2] > 0x00;
    
    // Extract movement state
    movement_detected = buffer[2] == 0x01;
      
    // Extract distance (bytes 6-7 contain distance in cm)
    float distance = (buffer[3] + (buffer[4] << 8)) / 100.0;  // Convert to meters
    if (distance > 0.00 && distance <= 10.0) {  // Valid range check
      distance_m = distance;
    }    
    // Serial.printf("Presence: %d, Movement: %d, Distance: %.2fm\n", 
    //         presence_detected, movement_detected, distance_m);
  } 
  else if (buffer[0] == 0xFA && buffer[1] == 0xFB) {  // Config report
    Serial.printf("Config Acknowledgment received\n");
    } 
}

void RD03ERadar::send_config_command(RD03EConfig& config) {
  if (!uart) return;
  bool sendCommand(uint16_t cmdWord, const uint8_t* data, uint16_t dataLen) {
    uint8_t header[] = {0xFD, 0xFC, 0xFB, 0xFA};
    uint8_t footer[] = {0x04, 0x03, 0x02, 0x01};
    uint16_t length = dataLen;
    
    uart->write(header, 4);
    uart->write((uint8_t*)&length, 2);
    uart->write((uint8_t*)&cmdWord, 2);
    if (data != nullptr && dataLen > 0) {
      uart->write(data, dataLen);
    }
    uart->write(footer, 4);
    
    // Wait for and validate ACK...
    return waitForAck(cmdWord);
  }
  
  // Wait for and validate ACK response
  bool RD03ERadar::waitForAck(uint16_t cmdWord) {
    // Implementation to read ACK and verify it matches expected format
    // ...
    return true;
  }

  void RD03ERadar::setConfig(const RD03EConfig& config) {
    _config = config;
  }
  
  // Get current configuration
  RD03EConfig& RD03ERadar::getConfig() {
    return _config;
  }
  
  // Apply all modified configuration settings
  bool RD03ERadar::applyConfig() {
    // Enable configuration mode first
    uint8_t enableData[] = {0x01, 0x00};
    if (!sendCommand(0x00FF, enableData, sizeof(enableData))) {
      return false;
    }
    
    // Apply distance calibration if modified
    if (_config.modified.distanceCalibration) {
      uint8_t data[6];
      uint16_t paramRef = static_cast<uint16_t>(ParamRef::DISTANCE_CALIBRATION);
      memcpy(data, &paramRef, 2);
      memcpy(data + 2, &_config.distanceCalibration, 4);
      if (!sendCommand(0x0072, data, 6)) return false;
    }
    
    // Apply distance settings if modified
    if (_config.modified.distanceSettings) {
      uint8_t data[32];
      uint16_t paramRefs[] = {
        static_cast<uint16_t>(ParamRef::MAX_MOVEMENT_DISTANCE),
        static_cast<uint16_t>(ParamRef::MIN_MOVEMENT_DISTANCE),
        static_cast<uint16_t>(ParamRef::MAX_MICRO_DISTANCE),
        static_cast<uint16_t>(ParamRef::MIN_MICRO_DISTANCE),
        static_cast<uint16_t>(ParamRef::UNMANNED_DURATION)
      };
      
      uint32_t values[] = {
        _config.distanceSettings.maxMovement,
        _config.distanceSettings.minMovement,
        _config.distanceSettings.maxMicroMotion,
        _config.distanceSettings.minMicroMotion,
        _config.distanceSettings.unmannedDuration
      };
      
      int offset = 0;
      for (int i = 0; i < 5; i++) {
        memcpy(data + offset, &paramRefs[i], 2);
        offset += 2;
        memcpy(data + offset, &values[i], 4);
        offset += 4;
      }
    }
  }
      
