#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "rd03e_radar.h"
#include "esp32_config.h"

// WiFi credentials
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// MQTT Broker settings
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PASSWORD;

// MQTT Topics
const char* mqtt_base_topic = MQTT_BASE_TOPIC;
const char* mqtt_presence_topic = MQTT_PRESENCE_TOPIC;
const char* mqtt_movement_topic = MQTT_MOVEMENT_TOPIC;
const char* mqtt_distance_topic = MQTT_DISTANCE_TOPIC;
const char* mqtt_config_topic = MQTT_CONFIG_TOPIC;
const char* mqtt_status_topic = MQTT_STATUS_TOPIC;

// Default sensor configuration
float detection_distance = DEFAULT_DETECTION_DISTANCE;
uint8_t sensitivity = DEFAULT_SENSITIVITY;

// UART pins for RD03-E sensor
const int RX_PIN = RADAR_RX_PIN;
const int TX_PIN = RADAR_TX_PIN;

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

// Radar sensor instance
RD03ERadar radar;

// Status variables
bool presence_state = false;
bool movement_state = false;
float distance_value = 0.0;
unsigned long last_publish = 0;
bool mqtt_connected = false;

// Function prototypes
void setup_wifi();
void reconnect_mqtt();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void publish_state();
void publish_sensor_data(bool force = false);
void publish_device_discovery();
void handle_config_message(const JsonDocument& doc);
void publish_noise_params();
void publish_distance_settings();

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("RD03-E Radar MQTT Client Starting...");
  
  // Initialize radar sensor
  radar.begin(RX_PIN, TX_PIN);
  radar.set_detection_distance(detection_distance);
  radar.set_sensitivity(40.0f, 6.0f, 40.0f, 9.0f);

  // Connect to WiFi
  setup_wifi();
  
  // Configure MQTT
  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_client.setCallback(mqtt_callback);
}

void loop() {
  // Ensure WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost, reconnecting...");
    setup_wifi();
  }
  
  // Ensure MQTT is connected
  if (!mqtt_client.connected()) {
    reconnect_mqtt();
  }
  mqtt_client.loop();
  
  // Read data from the radar sensor
  radar.update();
  
  // Get the sensor states
  bool new_presence = radar.is_presence_detected();
  bool new_movement = radar.is_movement_detected();
  float new_distance = radar.get_distance();
  
  // Check if states have changed
  bool state_changed = (new_presence != presence_state) || 
                      (new_movement != movement_state) || 
                      (abs(new_distance - distance_value) > 0.1); // 10cm threshold for distance change
  
  // Update states
  presence_state = new_presence;
  movement_state = new_movement;
  distance_value = new_distance;
  
  // Publish data if changed or periodically (every 30 seconds)
  if (state_changed || (millis() - last_publish > 300)) {
    publish_sensor_data(true);
    last_publish = millis();
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect_mqtt() {
  // Loop until we're reconnected
  int retries = 0;
  while (!mqtt_client.connected() && retries < 5) {
    Serial.print("[MQTT] Attempting connection to ");
    Serial.print(mqtt_server);
    Serial.print(":");
    Serial.print(mqtt_port);
    Serial.print(" as ");
    Serial.println(mqtt_user);
    
    retries++;
    
    // Create a random client ID with prefix
    String clientId = "ESP32_RD03E_";
    clientId += String(random(0xffff), HEX);
    
    Serial.print("[MQTT] Using client ID: ");
    Serial.println(clientId);
    
    // Log LWT details
    Serial.print("[MQTT] LWT Topic: ");
    Serial.print(mqtt_status_topic);
    Serial.println(" Message: offline");
    
    // Attempt to connect with LWT (Last Will and Testament)
    if (mqtt_client.connect(
        clientId.c_str(),           // Client ID
        mqtt_user,                  // Username
        mqtt_password,              // Password
        mqtt_status_topic,          // Will Topic (for availability)
        0,                          // Will QoS
        true,                       // Will Retain
        "offline"                   // Will Message (offline status)
    )) {
      Serial.println("[MQTT] Connected successfully");
      
      // Subscribe to relevant Zigbee2MQTT topics
      Serial.print("[MQTT SUB] ");
      Serial.println(mqtt_config_topic);
      mqtt_client.subscribe(mqtt_config_topic);
      
      Serial.print("[MQTT SUB] ");
      Serial.println(MQTT_GET_TOPIC);
      mqtt_client.subscribe(MQTT_GET_TOPIC);
      
      
      // Publish availability for Zigbee2MQTT
      Serial.print("[MQTT OUT] Topic: ");
      Serial.print(mqtt_status_topic);
      Serial.println(" Payload: online");
      mqtt_client.publish(mqtt_status_topic, "online", true);
      
      // Wait for MQTT infrastructure to process the connection
      delay(500);
      
      // Publish device discovery information for Zigbee2MQTT
      publish_device_discovery();
      
      // Publish initial state
      publish_sensor_data(true);
      
      mqtt_connected = true;
      
      // Reset retry counter on success
      retries = 0;
    } else {
      int state = mqtt_client.state();
      Serial.print("[MQTT ERROR] Connection failed, rc=");
      Serial.print(state);
      Serial.print(" (");
      
      // Print human-readable error
      switch(state) {
        case -4: Serial.print("MQTT_CONNECTION_TIMEOUT"); break;
        case -3: Serial.print("MQTT_CONNECTION_LOST"); break;
        case -2: Serial.print("MQTT_CONNECT_FAILED"); break;
        case -1: Serial.print("MQTT_DISCONNECTED"); break;
        case 1: Serial.print("MQTT_CONNECT_BAD_PROTOCOL"); break;
        case 2: Serial.print("MQTT_CONNECT_BAD_CLIENT_ID"); break;
        case 3: Serial.print("MQTT_CONNECT_UNAVAILABLE"); break;
        case 4: Serial.print("MQTT_CONNECT_BAD_CREDENTIALS"); break;
        case 5: Serial.print("MQTT_CONNECT_UNAUTHORIZED"); break;
        default: Serial.print("UNKNOWN_ERROR"); break;
      }
      
      Serial.println(")");
      Serial.println("[MQTT] Trying again in 5 seconds");
      delay(5000);
    }
  }
  
  if (!mqtt_client.connected()) {
    Serial.println("[MQTT ERROR] Failed to connect after multiple attempts. Will try again later.");
  }
}

void publish_sensor_data(bool force) {
  if (!mqtt_client.connected() && !force) return;
  
  // Get current values
  char presence_str[6];
  char movement_str[6];
  char distance_str[10];
  char detection_distance_str[10];
  char sensitivity_str[4];
  
  // Convert boolean and float values to strings
  strcpy(presence_str, presence_state ? "true" : "false");
  strcpy(movement_str, movement_state ? "true" : "false");
  dtostrf(distance_value, 4, 2, distance_str);
  dtostrf(detection_distance, 4, 2, detection_distance_str);
  itoa(sensitivity, sensitivity_str, 10);
  
  // Publish individual states
  bool pub_presence = mqtt_client.publish(MQTT_PRESENCE_TOPIC, presence_str, true);
  bool pub_movement = mqtt_client.publish(MQTT_MOVEMENT_TOPIC, movement_str, true);
  bool pub_distance = mqtt_client.publish(MQTT_DISTANCE_TOPIC, distance_str, true);
  bool pub_detection_distance = mqtt_client.publish((MQTT_BASE_TOPIC + String("/detection_distance/state")).c_str(), detection_distance_str, true);
  bool pub_sensitivity = mqtt_client.publish((MQTT_BASE_TOPIC + String("/sensitivity/state")).c_str(), sensitivity_str, true);
  
  // Also publish noise parameters and distance settings
  publish_noise_params();
  publish_distance_settings();
  
  // Log the publish results
  Serial.printf("[MQTT PUBLISH] Presence: %s, Movement: %s, Distance: %s, DetectionDist: %s, Sensitivity: %s\n",
               pub_presence ? "OK" : "Fail",
               pub_movement ? "OK" : "Fail",
               pub_distance ? "OK" : "Fail",
               pub_detection_distance ? "OK" : "Fail",
               pub_sensitivity ? "OK" : "Fail");
               
  // Track state changes
  static bool last_presence = false;
  static bool last_movement = false;
  static float last_distance = 0.0;
  
  if (last_presence != presence_state || last_movement != movement_state || 
      fabs(last_distance - distance_value) >= 0.1) {
    Serial.println("[MQTT] State changed significantly, logging details:");
    Serial.printf("  Presence: %s -> %s\n", last_presence ? "true" : "false", presence_state ? "true" : "false");
    Serial.printf("  Movement: %s -> %s\n", last_movement ? "true" : "false", movement_state ? "true" : "false");
    Serial.printf("  Distance: %.2f -> %.2f\n", last_distance, distance_value);
    
    last_presence = presence_state;
    last_movement = movement_state;
    last_distance = distance_value;
  }
}

void publish_noise_params() {
  RD03EConfig* config = radar.getConfig();
  if (config != nullptr && mqtt_client.connected()) {
    StaticJsonDocument<512> state_doc;
    state_doc["proximalMotion"] = config->noiseParams.proximalMotion;
    state_doc["distalMotion"] = config->noiseParams.distalMotion;
    state_doc["proximalMicro"] = config->noiseParams.proximalMicro;
    state_doc["distalMicro"] = config->noiseParams.distalMicro;
    
    char buffer[512];
    size_t n = serializeJson(state_doc, buffer);
    bool result = mqtt_client.publish((MQTT_BASE_TOPIC + String("/noise_params/state")).c_str(), buffer, true);
    Serial.printf("[MQTT PUBLISH] Noise Parameters: %s\n", result ? "OK" : "Fail");
  }
}

void publish_distance_settings() {
  RD03EConfig* config = radar.getConfig();
  if (config != nullptr && mqtt_client.connected()) {
    StaticJsonDocument<512> state_doc;
    state_doc["maxMacroMovement"] = config->distanceSettings.maxMacroMovement;
    state_doc["minMacroMovement"] = config->distanceSettings.minMacroMovement;
    state_doc["maxMicroMotion"] = config->distanceSettings.maxMicroMotion;
    state_doc["minMicroMotion"] = config->distanceSettings.minMicroMotion;
    state_doc["unmannedDuration"] = config->distanceSettings.unmannedDuration;
    
    char buffer[512];
    size_t n = serializeJson(state_doc, buffer);
    bool result = mqtt_client.publish((MQTT_BASE_TOPIC + String("/distance_settings/state")).c_str(), buffer, true);
    Serial.printf("[MQTT PUBLISH] Distance Settings: %s\n", result ? "OK" : "Fail");
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string for debugging
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("[MQTT IN] Topic: ");
  Serial.print(topic);
  Serial.print(" Payload: ");
  Serial.println(message);
  
  // Check if this is a config topic message
  if (String(topic) == String(mqtt_config_topic)) {
    Serial.println("[MQTT PROCESSING] Config message received");
    
    // Parse JSON configuration
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    
    if (error) {
      Serial.print("[MQTT ERROR] JSON parsing failed: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Debug: Print parsed config
    Serial.print("[MQTT CONFIG] Parsed configuration: ");
    serializeJson(doc, Serial);
    Serial.println();
    
    // Handle configuration changes
    handle_config_message(doc);
  }
  // Check if this is for specific parameter topics
  else if (String(topic).indexOf(MQTT_BASE_TOPIC) == 0 && String(topic).endsWith("/set")) {
    // Handle parameter-specific changes
    if (String(topic) == String(MQTT_BASE_TOPIC) + "/detection_distance/set") {
      // Parse as a float value
      float distance = message.toFloat();
      if (distance >= 0.5 && distance <= 6.0) {
        detection_distance = distance;
        radar.set_detection_distance(detection_distance);
        // Publish the updated state
        char distance_str[10];
        dtostrf(detection_distance, 4, 2, distance_str);
        mqtt_client.publish((MQTT_BASE_TOPIC + String("/detection_distance/state")).c_str(), distance_str, true);
      }
    }
    // Handle noise parameters setting
    else if (String(topic) == String(MQTT_BASE_TOPIC) + "/noise_params/set") {
      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, payload, length);
      
      if (!error) {
        // Check if we have all four parameters
        if (doc.containsKey("proximalMotion") && doc.containsKey("distalMotion") && 
            doc.containsKey("proximalMicro") && doc.containsKey("distalMicro")) {
            
          float proximalMotion = doc["proximalMotion"];
          float distalMotion = doc["distalMotion"];
          float proximalMicro = doc["proximalMicro"];
          float distalMicro = doc["distalMicro"];
          
          radar.set_sensitivity(proximalMotion, distalMotion, proximalMicro, distalMicro);
          
          // Publish the updated state
          StaticJsonDocument<512> state_doc;
          state_doc["proximalMotion"] = proximalMotion;
          state_doc["distalMotion"] = distalMotion;
          state_doc["proximalMicro"] = proximalMicro;
          state_doc["distalMicro"] = distalMicro;
          
          char buffer[512];
          size_t n = serializeJson(state_doc, buffer);
          mqtt_client.publish((MQTT_BASE_TOPIC + String("/noise_params/state")).c_str(), buffer, true);
        }
      }
    }
  }
  // Check if this is a get request (for retrieving values)
  else if (String(topic) == String(MQTT_GET_TOPIC)) {
    Serial.println("[MQTT PROCESSING] Get request received, publishing current state");
    
    // Parse the get request to see what parameter(s) to publish
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    
    if (!error) {
      if (doc.containsKey("param")) {
        const char* param = doc["param"];
        
        if (strcmp(param, "all") == 0) {
          // Publish all parameters
          publish_sensor_data(true);
          
          // Get noise parameters
          RD03EConfig* config = radar.getConfig();
          if (config != nullptr) {
            StaticJsonDocument<512> state_doc;
            state_doc["proximalMotion"] = config->noiseParams.proximalMotion;
            state_doc["distalMotion"] = config->noiseParams.distalMotion;
            state_doc["proximalMicro"] = config->noiseParams.proximalMicro;
            state_doc["distalMicro"] = config->noiseParams.distalMicro;
            
            char buffer[512];
            size_t n = serializeJson(state_doc, buffer);
            mqtt_client.publish((MQTT_BASE_TOPIC + String("/noise_params/state")).c_str(), buffer, true);
          }
        }
        else if (strcmp(param, "detection_distance") == 0) {
          // Publish just the detection distance
          char distance_str[10];
          dtostrf(detection_distance, 4, 2, distance_str);
          mqtt_client.publish((MQTT_BASE_TOPIC + String("/detection_distance/state")).c_str(), distance_str, true);
        }
        else if (strcmp(param, "noise_params") == 0) {
          // Publish just the noise parameters
          RD03EConfig* config = radar.getConfig();
          if (config != nullptr) {
            StaticJsonDocument<512> state_doc;
            state_doc["proximalMotion"] = config->noiseParams.proximalMotion;
            state_doc["distalMotion"] = config->noiseParams.distalMotion;
            state_doc["proximalMicro"] = config->noiseParams.proximalMicro;
            state_doc["distalMicro"] = config->noiseParams.distalMicro;
            
            char buffer[512];
            size_t n = serializeJson(state_doc, buffer);
            mqtt_client.publish((MQTT_BASE_TOPIC + String("/noise_params/state")).c_str(), buffer, true);
          }
        }
      } else {
        // No specific param, publish all
        publish_sensor_data(true);
      }
    } else {
      // If parsing failed, just publish all data
      publish_sensor_data(true);
    }
  }
  else {
    Serial.println("[MQTT WARNING] Message received on unexpected topic");
  }
}

void publish_device_discovery() {
  // Create a consistent device info object for all sensors
  StaticJsonDocument<200> device_doc;
  JsonObject device_info = device_doc.to<JsonObject>();
  device_info["identifiers"] = String(MQTT_UNIQUE_ID) + "_" + String(ESP.getChipModel(), HEX);
  device_info["name"] = "RD03E Radar Sensor";
  device_info["model"] = "RD03E";
  device_info["manufacturer"] = "ai-thinker";
  device_info["sw_version"] = "1.0.0";
  
  char device_buffer[200];
  serializeJson(device_info, device_buffer);
  
  // ----- Presence Sensor (Binary) -----
  StaticJsonDocument<400> presence_doc;
  presence_doc["name"] = "Presence";
  presence_doc["device_class"] = "presence";
  presence_doc["state_topic"] = MQTT_PRESENCE_TOPIC;
  presence_doc["availability_topic"] = MQTT_STATUS_TOPIC;
  presence_doc["payload_on"] = "true";
  presence_doc["payload_off"] = "false";
  presence_doc["value_template"] = "{{ value }}";
  presence_doc["unique_id"] = String(MQTT_UNIQUE_ID) + "_presence_" + String(ESP.getChipModel(), HEX);
  
  // Add the device info
  JsonObject presence_device = presence_doc.createNestedObject("device");
  presence_device["identifiers"] = String(MQTT_UNIQUE_ID) + "_" + String(ESP.getChipModel(), HEX);
  presence_device["name"] = "RD03E Radar Sensor";
  presence_device["model"] = "RD03E";
  presence_device["manufacturer"] = "ai-thinker";
  presence_device["sw_version"] = "1.0.0";
  
  char presence_buffer[400];
  size_t presence_length = serializeJson(presence_doc, presence_buffer, sizeof(presence_buffer));
  
  bool presence_result = mqtt_client.publish(MQTT_DISCOVERY_PRESENCE, presence_buffer, true);
  Serial.printf("[MQTT DISCOVERY] Presence sensor: %s\n", presence_result ? "OK" : "Failed");
  
  // ----- Movement Sensor (Binary) -----
  StaticJsonDocument<400> movement_doc;
  movement_doc["name"] = "Movement";
  movement_doc["device_class"] = "motion";
  movement_doc["state_topic"] = MQTT_MOVEMENT_TOPIC;
  movement_doc["availability_topic"] = MQTT_STATUS_TOPIC;
  movement_doc["payload_on"] = "true";
  movement_doc["payload_off"] = "false";
  movement_doc["value_template"] = "{{ value }}";
  movement_doc["unique_id"] = String(MQTT_UNIQUE_ID) + "_movement_" + String(ESP.getChipModel(), HEX);
  
  // Add the device info
  JsonObject movement_device = movement_doc.createNestedObject("device");
  movement_device["identifiers"] = String(MQTT_UNIQUE_ID) + "_" + String(ESP.getChipModel(), HEX);
  movement_device["name"] = "RD03E Radar Sensor";
  movement_device["model"] = "RD03E";
  movement_device["manufacturer"] = "ai-thinker";
  movement_device["sw_version"] = "1.0.0";
  
  char movement_buffer[400];
  size_t movement_length = serializeJson(movement_doc, movement_buffer, sizeof(movement_buffer));
  
  bool movement_result = mqtt_client.publish(MQTT_DISCOVERY_MOVEMENT, movement_buffer, true);
  Serial.printf("[MQTT DISCOVERY] Movement sensor: %s\n", movement_result ? "OK" : "Failed");
  
  // ----- Distance Sensor (Numeric) -----
  StaticJsonDocument<400> distance_doc;
  distance_doc["name"] = "Distance";
  distance_doc["device_class"] = "distance";
  distance_doc["state_topic"] = MQTT_DISTANCE_TOPIC;
  distance_doc["availability_topic"] = MQTT_STATUS_TOPIC;
  distance_doc["unit_of_measurement"] = "m";
  distance_doc["value_template"] = "{{ value }}";
  distance_doc["unique_id"] = String(MQTT_UNIQUE_ID) + "_distance_" + String(ESP.getChipModel(), HEX);
  
  // Add the device info
  JsonObject distance_device = distance_doc.createNestedObject("device");
  distance_device["identifiers"] = String(MQTT_UNIQUE_ID) + "_" + String(ESP.getChipModel(), HEX);
  distance_device["name"] = "RD03E Radar Sensor";
  distance_device["model"] = "RD03E";
  distance_device["manufacturer"] = "ai-thinker";
  distance_device["sw_version"] = "1.0.0";
  
  char distance_buffer[400];
  size_t distance_length = serializeJson(distance_doc, distance_buffer, sizeof(distance_buffer));
  
  bool distance_result = mqtt_client.publish(MQTT_DISCOVERY_DISTANCE, distance_buffer, true);
  Serial.printf("[MQTT DISCOVERY] Distance sensor: %s\n", distance_result ? "OK" : "Failed");
  
  // ----- Detection Distance Setting (Numeric with configuration) -----
  StaticJsonDocument<400> detection_distance_doc;
  detection_distance_doc["name"] = "Detection Distance";
  detection_distance_doc["icon"] = "mdi:ruler";
  detection_distance_doc["state_topic"] = MQTT_BASE_TOPIC + String("/detection_distance/state");
  detection_distance_doc["command_topic"] = MQTT_BASE_TOPIC + String("/detection_distance/set");
  detection_distance_doc["availability_topic"] = MQTT_STATUS_TOPIC;
  detection_distance_doc["unit_of_measurement"] = "m";
  detection_distance_doc["min"] = 0.5;
  detection_distance_doc["max"] = 6.0;
  detection_distance_doc["step"] = 0.1;
  detection_distance_doc["unique_id"] = String(MQTT_UNIQUE_ID) + "_detection_distance_" + String(ESP.getChipModel(), HEX);
  
  // Add the device info
  JsonObject detection_distance_device = detection_distance_doc.createNestedObject("device");
  detection_distance_device["identifiers"] = String(MQTT_UNIQUE_ID) + "_" + String(ESP.getChipModel(), HEX);
  detection_distance_device["name"] = "RD03E Radar Sensor";
  detection_distance_device["model"] = "RD03E";
  detection_distance_device["manufacturer"] = "ai-thinker";
  detection_distance_device["sw_version"] = "1.0.0";
  
  char detection_distance_buffer[400];
  size_t detection_distance_length = serializeJson(detection_distance_doc, detection_distance_buffer, sizeof(detection_distance_buffer));
  
  bool detection_distance_result = mqtt_client.publish(MQTT_DISCOVERY_DETECTION_DISTANCE, detection_distance_buffer, true);
  Serial.printf("[MQTT DISCOVERY] Detection Distance setting: %s\n", detection_distance_result ? "OK" : "Failed");
  
  // ----- Sensitivity Setting (Numeric with configuration) -----
  StaticJsonDocument<400> sensitivity_doc;
  sensitivity_doc["name"] = "Sensitivity";
  sensitivity_doc["icon"] = "mdi:tune";
  sensitivity_doc["state_topic"] = MQTT_BASE_TOPIC + String("/sensitivity/state");
  sensitivity_doc["command_topic"] = MQTT_BASE_TOPIC + String("/sensitivity/set");
  sensitivity_doc["availability_topic"] = MQTT_STATUS_TOPIC;
  sensitivity_doc["min"] = 1;
  sensitivity_doc["max"] = 10;
  sensitivity_doc["step"] = 1;
  sensitivity_doc["unique_id"] = String(MQTT_UNIQUE_ID) + "_sensitivity_" + String(ESP.getChipModel(), HEX);
  
  // Add the device info
  JsonObject sensitivity_device = sensitivity_doc.createNestedObject("device");
  sensitivity_device["identifiers"] = String(MQTT_UNIQUE_ID) + "_" + String(ESP.getChipModel(), HEX);
  sensitivity_device["name"] = "RD03E Radar Sensor";
  sensitivity_device["model"] = "RD03E";
  sensitivity_device["manufacturer"] = "ai-thinker";
  sensitivity_device["sw_version"] = "1.0.0";
  
  char sensitivity_buffer[400];
  size_t sensitivity_length = serializeJson(sensitivity_doc, sensitivity_buffer, sizeof(sensitivity_buffer));
  
  bool sensitivity_result = mqtt_client.publish(MQTT_DISCOVERY_SENSITIVITY, sensitivity_buffer, true);
  Serial.printf("[MQTT DISCOVERY] Sensitivity setting: %s\n", sensitivity_result ? "OK" : "Failed");
  
  // ----- Noise Parameters (JSON) -----
  StaticJsonDocument<600> noise_params_doc;
  noise_params_doc["name"] = "Noise Parameters";
  noise_params_doc["icon"] = "mdi:tune-vertical";
  noise_params_doc["state_topic"] = MQTT_BASE_TOPIC + String("/noise_params/state");
  noise_params_doc["json_attributes_topic"] = MQTT_BASE_TOPIC + String("/noise_params/state");
  noise_params_doc["command_topic"] = MQTT_BASE_TOPIC + String("/noise_params/set");
  noise_params_doc["availability_topic"] = MQTT_STATUS_TOPIC;
  noise_params_doc["unique_id"] = String(MQTT_UNIQUE_ID) + "_noise_params_" + String(ESP.getChipModel(), HEX);
  
  // Add the device info
  JsonObject noise_params_device = noise_params_doc.createNestedObject("device");
  noise_params_device["identifiers"] = String(MQTT_UNIQUE_ID) + "_" + String(ESP.getChipModel(), HEX);
  noise_params_device["name"] = "RD03E Radar Sensor";
  noise_params_device["model"] = "RD03E";
  noise_params_device["manufacturer"] = "ai-thinker";
  noise_params_device["sw_version"] = "1.0.0";
  
  char noise_params_buffer[600];
  size_t noise_params_length = serializeJson(noise_params_doc, noise_params_buffer, sizeof(noise_params_buffer));
  
  bool noise_params_result = mqtt_client.publish((MQTT_DISCOVERY_PREFIX + String("/sensor/") + MQTT_DEVICE_ID + String("/noise_params/config")).c_str(), noise_params_buffer, true);
  Serial.printf("[MQTT DISCOVERY] Noise Parameters: %s\n", noise_params_result ? "OK" : "Failed");
  
  // ----- Distance Settings (JSON) -----
  StaticJsonDocument<600> distance_settings_doc;
  distance_settings_doc["name"] = "Distance Settings";
  distance_settings_doc["icon"] = "mdi:ruler-square";
  distance_settings_doc["state_topic"] = MQTT_BASE_TOPIC + String("/distance_settings/state");
  distance_settings_doc["json_attributes_topic"] = MQTT_BASE_TOPIC + String("/distance_settings/state");
  distance_settings_doc["command_topic"] = MQTT_BASE_TOPIC + String("/distance_settings/set");
  distance_settings_doc["availability_topic"] = MQTT_STATUS_TOPIC;
  distance_settings_doc["unique_id"] = String(MQTT_UNIQUE_ID) + "_distance_settings_" + String(ESP.getChipModel(), HEX);
  
  // Add the device info
  JsonObject distance_settings_device = distance_settings_doc.createNestedObject("device");
  distance_settings_device["identifiers"] = String(MQTT_UNIQUE_ID) + "_" + String(ESP.getChipModel(), HEX);
  distance_settings_device["name"] = "RD03E Radar Sensor";
  distance_settings_device["model"] = "RD03E";
  distance_settings_device["manufacturer"] = "ai-thinker";
  distance_settings_device["sw_version"] = "1.0.0";
  
  char distance_settings_buffer[600];
  size_t distance_settings_length = serializeJson(distance_settings_doc, distance_settings_buffer, sizeof(distance_settings_buffer));
  
  bool distance_settings_result = mqtt_client.publish((MQTT_DISCOVERY_PREFIX + String("/sensor/") + MQTT_DEVICE_ID + String("/distance_settings/config")).c_str(), distance_settings_buffer, true);
  Serial.printf("[MQTT DISCOVERY] Distance Settings: %s\n", distance_settings_result ? "OK" : "Failed");
}

void handle_config_message(const JsonDocument& doc) {
  bool config_changed = false;
  
  Serial.println("[CONFIG] Processing configuration message");
  
  // Check if configuration contains detection_distance
  if (doc.containsKey("detection_distance")) {
    float new_distance = doc["detection_distance"];
    Serial.print("[CONFIG] Detection distance in message: ");
    Serial.print(new_distance);
    Serial.print(", current: ");
    Serial.println(detection_distance);
    
    if (new_distance >= 0.5 && new_distance <= 6.0) {
      if (new_distance != detection_distance) {
        detection_distance = new_distance;
        radar.set_detection_distance(detection_distance);
        config_changed = true;
        Serial.print("[CONFIG UPDATED] Detection distance set to: ");
        Serial.println(detection_distance);
      } else {
        Serial.println("[CONFIG] Detection distance unchanged (same value)");
      }
    } else {
      Serial.print("[CONFIG ERROR] Detection distance out of range (0.5-6.0m): ");
      Serial.println(new_distance);
    }
  }
  
  // Check if configuration contains sensitivity
  if (doc.containsKey("sensitivity")) {
    int new_sensitivity = doc["sensitivity"];
    Serial.print("[CONFIG] Sensitivity in message: ");
    Serial.print(new_sensitivity);
    Serial.print(", current: ");
    Serial.println(sensitivity);
    
    if (new_sensitivity >= 1 && new_sensitivity <= 10) {
      if (new_sensitivity != sensitivity) {
        sensitivity = new_sensitivity;
        radar.set_sensitivity(40.0f, 6.0f, 40.0f, 9.0f);
        config_changed = true;
        Serial.print("[CONFIG UPDATED] Sensitivity set to: ");
        Serial.println(sensitivity);
      } else {
        Serial.println("[CONFIG] Sensitivity unchanged (same value)");
      }
    } else {
      Serial.print("[CONFIG ERROR] Sensitivity out of range (1-10): ");
      Serial.println(new_sensitivity);
    }
  }
  
  // Check if configuration contains noise parameters
  if (doc.containsKey("noise_params")) {
    // Check if we have all four parameters
    if (doc["noise_params"].containsKey("proximalMotion") && 
        doc["noise_params"].containsKey("distalMotion") && 
        doc["noise_params"].containsKey("proximalMicro") && 
        doc["noise_params"].containsKey("distalMicro")) {
        
      float proximalMotion = doc["noise_params"]["proximalMotion"];
      float distalMotion = doc["noise_params"]["distalMotion"];
      float proximalMicro = doc["noise_params"]["proximalMicro"];
      float distalMicro = doc["noise_params"]["distalMicro"];
      
      // Validate values (using reasonable ranges)
      if (proximalMotion >= 10.0 && proximalMotion <= 100.0 &&
          distalMotion >= 1.0 && distalMotion <= 20.0 &&
          proximalMicro >= 10.0 && proximalMicro <= 100.0 &&
          distalMicro >= 1.0 && distalMicro <= 20.0) {
          
        radar.set_sensitivity(proximalMotion, distalMotion, proximalMicro, distalMicro);
        config_changed = true;
        
        Serial.println("[CONFIG UPDATED] Noise parameters set to:");
        Serial.print("  Proximal Motion: "); Serial.println(proximalMotion);
        Serial.print("  Distal Motion: "); Serial.println(distalMotion);
        Serial.print("  Proximal Micro: "); Serial.println(proximalMicro);
        Serial.print("  Distal Micro: "); Serial.println(distalMicro);
      } else {
        Serial.println("[CONFIG ERROR] One or more noise parameters out of range");
      }
    } else {
      Serial.println("[CONFIG ERROR] Missing one or more required noise parameters");
    }
  }
  
  // Check if configuration contains distance settings
  if (doc.containsKey("distance_settings")) {
    
    RD03EConfig* config = radar.getConfig();
    if (config != nullptr) {
      bool distanceChanged = false;
      
      if (doc["distance_settings"].containsKey("maxMacroMovement")) {
        uint32_t value = doc["distance_settings"]["maxMacroMovement"];
        if (value >= 30 && value <= 717) {
          config->distanceSettings.maxMacroMovement = value;
          distanceChanged = true;
        }
      }
      
      if (doc["distance_settings"].containsKey("minMacroMovement")) {
        uint32_t value = doc["distance_settings"]["minMacroMovement"];
        if (value >= 0 && value <= 100) {
          config->distanceSettings.minMacroMovement = value;
          distanceChanged = true;
        }
      }
      
      if (doc["distance_settings"].containsKey("maxMicroMotion")) {
        uint32_t value = doc["distance_settings"]["maxMicroMotion"];
        if (value >= 30 && value <= 425) {
          config->distanceSettings.maxMicroMotion = value;
          distanceChanged = true;
        }
      }
      
      if (doc["distance_settings"].containsKey("minMicroMotion")) {
        uint32_t value = doc["distance_settings"]["minMicroMotion"];
        if (value >= 0 && value <= 100) {
          config->distanceSettings.minMicroMotion = value;
          distanceChanged = true;
        }
      }
      
      if (doc["distance_settings"].containsKey("unmannedDuration")) {
        uint32_t value = doc["distance_settings"]["unmannedDuration"];
        if (value >= 0 && value <= 65535) {
          config->distanceSettings.unmannedDuration = value;
          distanceChanged = true;
        }
      }
      
      if (distanceChanged) {
        config->modified.distanceSettings = true;
        config_changed = true;
        Serial.println("[CONFIG UPDATED] Distance settings updated");
      }
    }
  }
  
  // Publish updated configuration if changed
  if (config_changed) {
    Serial.println("[CONFIG] Configuration changed, publishing updated state");
    publish_sensor_data(true);
    
    // Additionally, publish the specific parameter states that were changed
    if (doc.containsKey("noise_params")) {
      RD03EConfig* config = radar.getConfig();
      if (config != nullptr) {
        StaticJsonDocument<512> state_doc;
        state_doc["proximalMotion"] = config->noiseParams.proximalMotion;
        state_doc["distalMotion"] = config->noiseParams.distalMotion;
        state_doc["proximalMicro"] = config->noiseParams.proximalMicro;
        state_doc["distalMicro"] = config->noiseParams.distalMicro;
        
        char buffer[512];
        size_t n = serializeJson(state_doc, buffer);
        mqtt_client.publish((MQTT_BASE_TOPIC + String("/noise_params/state")).c_str(), buffer, true);
      }
    }
    
    if (doc.containsKey("distance_settings")) {
      RD03EConfig* config = radar.getConfig();
      if (config != nullptr) {
        StaticJsonDocument<512> state_doc;
        state_doc["maxMacroMovement"] = config->distanceSettings.maxMacroMovement;
        state_doc["minMacroMovement"] = config->distanceSettings.minMacroMovement;
        state_doc["maxMicroMotion"] = config->distanceSettings.maxMicroMotion;
        state_doc["minMicroMotion"] = config->distanceSettings.minMicroMotion;
        state_doc["unmannedDuration"] = config->distanceSettings.unmannedDuration;
        
        char buffer[512];
        size_t n = serializeJson(state_doc, buffer);
        mqtt_client.publish((MQTT_BASE_TOPIC + String("/distance_settings/state")).c_str(), buffer, true);
      }
    }
  } else {
    Serial.println("[CONFIG] No configuration changes applied");
  }
}