#include "rd03e_radar.h"

RD03ERadar::RD03ERadar() : uart(nullptr) {

  // struct RadarConfig {
  //     float detection_distance ,
  //     float sensitivity,
  //     float noise_threshold
  // }
  
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
    //send_config_command();
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

void RD03ERadar::send_config_command() {
  if (!uart) return;
  
  uint8_t init_config[14]; 
  // Header
  init_config[0] = CONFIG_FRAME_HEADER_1;
  init_config[1] = CONFIG_FRAME_HEADER_2;
  init_config[2] = CONFIG_FRAME_HEADER_3;
  init_config[3] = CONFIG_FRAME_HEADER_4;
   
  // Data length (4 bytes)
  init_config[4] = 0x00;
  init_config[5] = 0x04;

  // Command word = Set Parameters
  init_config[6] = 0x00;
  init_config[7] = 0xFF;

  //command value
  init_config[8] = 0x00;
  init_config[9] = 0x01;
  
  // Data length (8 bytes)
  init_config[10] = CONFIG_FRAME_FOOTER_1;
  init_config[11] = CONFIG_FRAME_FOOTER_2;
  init_config[12] = CONFIG_FRAME_FOOTER_3;
  init_config[13] = CONFIG_FRAME_FOOTER_4;

  
uint8_t end_config[11];

 // Header
 end_config[0] = CONFIG_FRAME_HEADER_1;
 end_config[1] = CONFIG_FRAME_HEADER_2;
 end_config[2] = CONFIG_FRAME_HEADER_3;
 end_config[3] = CONFIG_FRAME_HEADER_4;

  // Data length (13 bytes)
  end_config[4] = 0x00;
  end_config[5] = 0x02;

  // Command word = Set Parameters
  end_config[6] = 0x00;
  end_config[7] = 0xFE;


  end_config[8] =  CONFIG_FRAME_FOOTER_1;                               // Parameter ID
  end_config[9] =  CONFIG_FRAME_FOOTER_2;                                // Parameter value (continuous mode)
  end_config[10] = CONFIG_FRAME_FOOTER_3;                               // Reserved
  end_config[11] = CONFIG_FRAME_FOOTER_4;                               // Reserved

  
  // Send the command
  uart->write(end_config, 13);
  Serial.println("Sent configuration command to radar");
    }
  
