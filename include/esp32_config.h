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

// MQTT Discovery configuration for Home Assistant
#define MQTT_DISCOVERY_PREFIX "homeassistant"
#define MQTT_DEVICE_ID "rd03e_radar"
#define MQTT_UNIQUE_ID "RD03E"
// Use a fixed identifier - don't use ESP.getChipModel() as it might change
#define MQTT_DEVICE_IDENTIFIER MQTT_UNIQUE_ID

// Home Assistant global status topic for detecting device availability
#define MQTT_HA_STATUS_TOPIC "homeassistant/status"

// Device topic structure for state updates
#define MQTT_BASE_TOPIC "rd03e_radar"
#define MQTT_PRESENCE_TOPIC MQTT_BASE_TOPIC "/presence/state"
#define MQTT_MOVEMENT_TOPIC MQTT_BASE_TOPIC "/movement/state"
#define MQTT_DISTANCE_TOPIC MQTT_BASE_TOPIC "/distance/state"
#define MQTT_CONFIG_TOPIC MQTT_BASE_TOPIC "/set"
#define MQTT_GET_TOPIC MQTT_BASE_TOPIC "/get"
#define MQTT_STATUS_TOPIC MQTT_BASE_TOPIC "/status"

// Discovery Topics
#define MQTT_DISCOVERY_BINARY_SENSOR MQTT_DISCOVERY_PREFIX "/binary_sensor"
#define MQTT_DISCOVERY_SENSOR MQTT_DISCOVERY_PREFIX "/sensor"
#define MQTT_DISCOVERY_NUMBER MQTT_DISCOVERY_PREFIX "/number"

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