#pragma once

// Include secrets (copy secrets.h.example to secrets.h and fill with your credentials)
#include "secrets.h"

// If secrets.h is not found, use these defaults (DO NOT edit these directly)
#ifndef WIFI_SSID
  #define WIFI_SSID "your_wifi_ssid"
  #define WIFI_PASSWORD "your_wifi_password"
  #define MQTT_SERVER "192.168.1.x"
  #define MQTT_PORT 1883
  #define MQTT_USER ""
  #define MQTT_PASSWORD ""
#endif

// MQTT Topics - Zigbee2MQTT integration
#define MQTT_Z2M_PREFIX "zigbee2mqtt"
#define MQTT_DEVICE_ID "rd03e_radar"
#define MQTT_UNIQUE_ID "D30EFD30EF"

// Device topic structure
#define MQTT_BASE_TOPIC MQTT_Z2M_PREFIX "/" MQTT_DEVICE_ID
#define MQTT_PRESENCE_TOPIC MQTT_BASE_TOPIC "/presence"
#define MQTT_MOVEMENT_TOPIC MQTT_BASE_TOPIC "/movement"
#define MQTT_DISTANCE_TOPIC MQTT_BASE_TOPIC "/distance"
#define MQTT_CONFIG_TOPIC MQTT_BASE_TOPIC "/set"
#define MQTT_GET_TOPIC MQTT_BASE_TOPIC "/get"
#define MQTT_STATUS_TOPIC MQTT_BASE_TOPIC "/availability"

// Zigbee2MQTT specific topics
#define MQTT_Z2M_BRIDGE_TOPIC MQTT_Z2M_PREFIX "/bridge"
#define MQTT_Z2M_DEVICES_TOPIC MQTT_Z2M_BRIDGE_TOPIC "/devices"

// RD03-E Radar settings
#define RADAR_RX_PIN 16    // GPIO16 for receiving data from radar
#define RADAR_TX_PIN 17    // GPIO17 for sending commands to radar
#define RADAR_BAUD_RATE 256000
#define DEFAULT_DETECTION_DISTANCE 3.0   // 3 meters
#define DEFAULT_SENSITIVITY 5            // Range 1-10

// Other settings
#define UPDATE_INTERVAL 3    // 30 seconds between forced updates
#define MQTT_RECONNECT_DELAY 5000  // 5 seconds between MQTT reconnection attempts
#define SERIAL_BAUD_RATE 115200