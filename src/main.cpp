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
const char* mqtt_ha_status_topic = MQTT_HA_STATUS_TOPIC;

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
      
      // Subscribe to relevant topics
      Serial.print("[MQTT SUB] ");
      Serial.println(mqtt_config_topic);
      mqtt_client.subscribe(mqtt_config_topic);
      
      Serial.print("[MQTT SUB] ");
      Serial.println(MQTT_GET_TOPIC);
      mqtt_client.subscribe(MQTT_GET_TOPIC);
      
      // Subscribe to Home Assistant status topic to detect when HA comes online
      Serial.print("[MQTT SUB] ");
      Serial.println(mqtt_ha_status_topic);
      mqtt_client.subscribe(mqtt_ha_status_topic);
      
      
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
  // Check if this is Home Assistant status message
  else if (String(topic) == String(mqtt_ha_status_topic)) {
    Serial.print("[MQTT] Home Assistant status change: ");
    Serial.println(message);
    
    // If Home Assistant is announcing it's online, republish discovery
    if (message == "online") {
      Serial.println("[MQTT] Home Assistant came online, republishing discovery and state");
      // Wait a moment for HA to get ready
      delay(1000);
      publish_device_discovery();
      delay(500);
      publish_sensor_data(true);
    }
  }
  else {
    Serial.println("[MQTT WARNING] Message received on unexpected topic: " + String(topic));
  }
}

// Forward declarations of discovery helper functions
void publish_binary_sensor_discovery(const char* name, const char* device_class, const char* state_topic, const char* unique_id_suffix);
void publish_distance_sensor_discovery();
void publish_number_discovery(const char* name, const char* icon, const char* state_topic, const char* command_topic, 
                          float min_val, float max_val, float step_val, const char* unit, const char* unique_id_suffix);
void publish_json_sensor_discovery(const char* name, const char* icon, const char* state_topic, const char* command_topic, 
                              const char* unique_id_suffix);

void publish_device_discovery() {
  // Publish individual discoveries one at a time to avoid stack overflows
  
  // First publish the binary sensors - use device name as prefix for better organization
  publish_binary_sensor_discovery("Presence", "presence", MQTT_PRESENCE_TOPIC, "presence");
  delay(100); // Slightly longer delay between publishes to avoid buffer issues
  
  publish_binary_sensor_discovery("Movement", "motion", MQTT_MOVEMENT_TOPIC, "movement");
  delay(100);
  
  // Publish the distance sensor
  publish_distance_sensor_discovery();
  delay(100);
  
  // Create topic strings
  String detection_distance_state = String(MQTT_BASE_TOPIC) + "/detection_distance/state";
  String detection_distance_set = String(MQTT_BASE_TOPIC) + "/detection_distance/set"; 
  String sensitivity_state = String(MQTT_BASE_TOPIC) + "/sensitivity/state";
  String sensitivity_set = String(MQTT_BASE_TOPIC) + "/sensitivity/set";
  String noise_params_state = String(MQTT_BASE_TOPIC) + "/noise_params/state";
  String noise_params_set = String(MQTT_BASE_TOPIC) + "/noise_params/set";
  String distance_settings_state = String(MQTT_BASE_TOPIC) + "/distance_settings/state";
  String distance_settings_set = String(MQTT_BASE_TOPIC) + "/distance_settings/set";
  
  // Publish the number settings
  publish_number_discovery("Detection Distance", "mdi:ruler", 
                        detection_distance_state.c_str(),
                        detection_distance_set.c_str(),
                        0.5, 6.0, 0.1, "m", "detection_distance");
  delay(100);
  
  publish_number_discovery("Sensitivity", "mdi:tune",
                        sensitivity_state.c_str(),
                        sensitivity_set.c_str(),
                        1, 10, 1, "", "sensitivity");
  delay(100);
  
  // Publish the JSON sensors
  publish_json_sensor_discovery("Noise Parameters", "mdi:tune-vertical",
                           noise_params_state.c_str(),
                           noise_params_set.c_str(),
                           "noise_params");
  delay(100);
  
  publish_json_sensor_discovery("Distance Settings", "mdi:ruler-square",
                           distance_settings_state.c_str(),
                           distance_settings_set.c_str(),
                           "distance_settings");
}

// Helper function to add common device info to discovery messages
void add_device_info(JsonDocument& doc) {
  JsonObject device = doc.createNestedObject("device");
  
  // Create an array for identifiers (Home Assistant prefers array format)
  JsonArray identifiers = device.createNestedArray("identifiers");
  // Use a consistent identifier that doesn't change between boots
  identifiers.add(MQTT_DEVICE_IDENTIFIER);
  
  device["name"] = "RD03E Radar Sensor";
  device["model"] = "RD03E";
  device["manufacturer"] = "ai-thinker";
  device["sw_version"] = "1.0.0";
  
  // Add suggested_area if you have one
  // device["suggested_area"] = "Living Room";
}

// Helper to publish a binary sensor discovery
void publish_binary_sensor_discovery(const char* name, const char* device_class, const char* state_topic, const char* unique_id_suffix) {
  StaticJsonDocument<384> doc;
  doc["name"] = name;
  doc["device_class"] = device_class;
  doc["state_topic"] = state_topic;
  doc["availability_topic"] = MQTT_STATUS_TOPIC;
  doc["payload_on"] = "true";
  doc["payload_off"] = "false";
  doc["value_template"] = "{{ value }}";
  doc["unique_id"] = String(MQTT_DEVICE_IDENTIFIER) + "_" + unique_id_suffix;
  
  // Add the device info
  add_device_info(doc);
  
  char buffer[384];
  size_t length = serializeJson(doc, buffer, sizeof(buffer));
  
  // Debug: Print the complete discovery message
  Serial.print("[DISCOVERY JSON] ");
  serializeJson(doc, Serial);
  Serial.println();
  
  String topic = String(MQTT_DISCOVERY_BINARY_SENSOR) + "/" + MQTT_DEVICE_ID + "/" + unique_id_suffix + "/config";
  bool result = mqtt_client.publish(topic.c_str(), buffer, true);
  Serial.printf("[MQTT DISCOVERY] %s sensor: %s\n", name, result ? "OK" : "Failed");
}

// Helper to publish distance sensor discovery
void publish_distance_sensor_discovery() {
  StaticJsonDocument<384> doc;
  doc["name"] = "Distance";
  doc["device_class"] = "distance";
  doc["state_topic"] = MQTT_DISTANCE_TOPIC;
  doc["availability_topic"] = MQTT_STATUS_TOPIC;
  doc["unit_of_measurement"] = "m";
  doc["value_template"] = "{{ value }}";
  doc["unique_id"] = String(MQTT_DEVICE_IDENTIFIER) + "_distance";
  
  // Add the device info
  add_device_info(doc);
  
  char buffer[384];
  size_t length = serializeJson(doc, buffer, sizeof(buffer));
  
  // Debug: Print the complete discovery message
  Serial.print("[DISCOVERY JSON] ");
  serializeJson(doc, Serial);
  Serial.println();
  
  String topic = String(MQTT_DISCOVERY_SENSOR) + "/" + MQTT_DEVICE_ID + "/distance/config";
  bool result = mqtt_client.publish(topic.c_str(), buffer, true);
  Serial.printf("[MQTT DISCOVERY] Distance sensor: %s\n", result ? "OK" : "Failed");
}

// Helper to publish a numeric setting with min/max/step
void publish_number_discovery(const char* name, const char* icon, const char* state_topic, const char* command_topic, 
                          float min_val, float max_val, float step_val, const char* unit, const char* unique_id_suffix) {
  StaticJsonDocument<384> doc;
  doc["name"] = name;
  doc["icon"] = icon;
  doc["state_topic"] = state_topic;
  doc["command_topic"] = command_topic;
  doc["availability_topic"] = MQTT_STATUS_TOPIC;
  doc["entity_category"] = "config";
  
  if (strlen(unit) > 0) {
    doc["unit_of_measurement"] = unit;
  }
  
  doc["min"] = min_val;
  doc["max"] = max_val;
  doc["step"] = step_val;
  doc["unique_id"] = String(MQTT_DEVICE_IDENTIFIER) + "_" + unique_id_suffix;
  
  // Add the device info
  add_device_info(doc);
  
  char buffer[384];
  size_t length = serializeJson(doc, buffer, sizeof(buffer));
  
  // Debug: Print the complete discovery message
  Serial.print("[DISCOVERY JSON] ");
  serializeJson(doc, Serial);
  Serial.println();
  
  String topic = String(MQTT_DISCOVERY_PREFIX) + "/number/" + MQTT_DEVICE_ID + "/" + unique_id_suffix + "/config";
  bool result = mqtt_client.publish(topic.c_str(), buffer, true);
  Serial.printf("[MQTT DISCOVERY] %s setting: %s\n", name, result ? "OK" : "Failed");
}

// Helper to publish a JSON sensor (for complex parameters)
void publish_json_sensor_discovery(const char* name, const char* icon, const char* state_topic, const char* command_topic, 
                              const char* unique_id_suffix) {
  StaticJsonDocument<384> doc;
  doc["name"] = name;
  doc["icon"] = icon;
  doc["state_topic"] = state_topic;
  doc["json_attributes_topic"] = state_topic;
  doc["command_topic"] = command_topic;
  doc["availability_topic"] = MQTT_STATUS_TOPIC;
  doc["unique_id"] = String(MQTT_DEVICE_IDENTIFIER) + "_" + unique_id_suffix;
  doc["entity_category"] = "config";
  
  // Add the device info
  add_device_info(doc);
  
  char buffer[384];
  size_t length = serializeJson(doc, buffer, sizeof(buffer));
  
  // Debug: Print the complete discovery message
  Serial.print("[DISCOVERY JSON] ");
  serializeJson(doc, Serial);
  Serial.println();
  
  String topic = String(MQTT_DISCOVERY_SENSOR) + "/" + MQTT_DEVICE_ID + "/" + unique_id_suffix + "/config";
  bool result = mqtt_client.publish(topic.c_str(), buffer, true);
  Serial.printf("[MQTT DISCOVERY] %s: %s\n", name, result ? "OK" : "Failed");
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