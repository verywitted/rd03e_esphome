#include "rd03e_radar.h"

RD03ERadar::RD03ERadar() : uart(nullptr) {
  // Initialize variables
  buffer_index = 0;
  frame_started = false;
  waiting_for_header2 = false;
  presence_detected = false;
  movement_detected = false;
  distance_m = 0.0;
  config_pending = true;
}

void RD03ERadar::begin(int rx_pin, int tx_pin, int baud_rate) {
  // Initialize the UART for communicating with the RD03-E
  uart = new HardwareSerial(1); // Use UART1
  uart->begin(baud_rate, SERIAL_8N1, rx_pin, tx_pin);
  
  Serial.println("RD03-E Radar initialized");
  
  // Clear the buffer
  buffer_index = 0;
  memset(buffer, 0, BUFFER_SIZE);
  
  // Send initial configuration
  send_config_command();
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
    process_byte(data);
  }
}

void RD03ERadar::set_detection_distance(float distance) {
  if (distance >= 0.5 && distance <= 6.0) {
    this->detection_distance = distance;
    this->config_pending = true;
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
  // Look for frame header sequence
  if (!frame_started) {
    if (!waiting_for_header2) {
      if (data == FRAME_HEADER_1) {
        waiting_for_header2 = true;
        buffer[0] = data;
        buffer_index = 1;
      }
    } else {
      if (data == FRAME_HEADER_2) {
        frame_started = true;
        buffer[buffer_index++] = data;
      } else {
        // Reset if the second byte is not the expected header
        waiting_for_header2 = false;
        buffer_index = 0;
      }
    }
  } else {
    // Add byte to buffer
    buffer[buffer_index++] = data;
    
    // If we have received a complete frame
    if (buffer_index == FRAME_LENGTH) {
      process_frame();
      
      // Reset for next frame
      frame_started = false;
      waiting_for_header2 = false;
      buffer_index = 0;
    }
    
    // Prevent buffer overflow
    if (buffer_index >= BUFFER_SIZE) {
      frame_started = false;
      waiting_for_header2 = false;
      buffer_index = 0;
    }
  }
}

void RD03ERadar::process_frame() {
  // Verify the frame with checksum
  uint8_t calculated_checksum = calculate_checksum(buffer, FRAME_LENGTH - 1);
  
  // If checksum matches
  if (calculated_checksum == buffer[FRAME_LENGTH - 1]) {
    // Check command type
    if (buffer[2] == 0x01) {  // Status report
      // Extract presence state (0x01 = presence, 0x00 = no presence)
      presence_detected = buffer[4] == 0x01;
      
      // Extract movement state
      movement_detected = buffer[5] == 0x01;
      
      // Extract distance (bytes 6-7 contain distance in cm)
      float distance = (buffer[6] + (buffer[7] << 8)) / 100.0;  // Convert to meters
      if (distance > 0 && distance <= 8.0) {  // Valid range check
        distance_m = distance;
      }
      
      Serial.printf("Presence: %d, Movement: %d, Distance: %.2fm\n", 
              presence_detected, movement_detected, distance_m);
    }
  } else {
    Serial.println("Invalid checksum in received frame");
  }
}

void RD03ERadar::send_config_command() {
  if (!uart) return;
  
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
  cmd[4] = (uint8_t)(detection_distance * 10);  // Distance in decimeters
  cmd[5] = sensitivity;                         // Sensitivity level
  cmd[6] = 0x01;                                // Enable continuous mode
  cmd[7] = 0x00;                                // Reserved
  cmd[8] = 0x00;                                // Reserved
  cmd[9] = 0x00;                                // Reserved
  cmd[10] = 0x00;                               // Reserved
  cmd[11] = 0x00;                               // Reserved
  
  // Calculate checksum (XOR of all previous bytes)
  cmd[12] = calculate_checksum(cmd, 12);
  
  // Send the command
  uart->write(cmd, 13);
  Serial.println("Sent configuration command to radar");
}

uint8_t RD03ERadar::calculate_checksum(const uint8_t* data, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}