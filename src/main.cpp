#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "rd03e_radar.h"
#include "config.h"

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

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("RD03-E Radar MQTT Client Starting...");
  
  // Initialize radar sensor
  radar.begin(RX_PIN, TX_PIN);
  radar.set_detection_distance(detection_distance);
  radar.set_sensitivity(sensitivity);
  
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
      
      Serial.print("[MQTT SUB] ");
      Serial.println(MQTT_Z2M_BRIDGE_TOPIC);
      mqtt_client.subscribe(MQTT_Z2M_BRIDGE_TOPIC);
      
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
  
  // Create JSON document for Zigbee2MQTT-compatible sensor data
  StaticJsonDocument<200> doc;
  
  // Standard Z2MQTT format
  doc["presence"] = presence_state;
  doc["movement"] = movement_state;
  doc["distance"] = distance_value;
  doc["detection_distance"] = detection_distance;
  doc["sensitivity"] = sensitivity;
  doc["linkquality"] = (int)(WiFi.RSSI() + 100); // Convert RSSI to linkquality (0-100)
  
  char buffer[200];
  serializeJson(doc, buffer);
  
  // Debug - Log MQTT publish
  Serial.print("[MQTT OUT] Topic: ");
  Serial.print(mqtt_base_topic);
  Serial.print(" Payload: ");
  Serial.println(buffer);
  
  // Publish to main topic in Zigbee2MQTT format
  bool pub_result = mqtt_client.publish(mqtt_base_topic, buffer, true);
  if (!pub_result) {
    Serial.print("[MQTT ERROR] Failed to publish to: ");
    Serial.print(mqtt_base_topic);
    Serial.print(", State: ");
    Serial.println(mqtt_client.state());
  }
  
  // On state change, add state_change flag for Zigbee2MQTT to handle properly
  static bool last_presence = false;
  static bool last_movement = false;
  
  if (last_presence != presence_state || last_movement != movement_state) {
    Serial.println("State changed, sending detailed state update");
    last_presence = presence_state;
    last_movement = movement_state;
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
    StaticJsonDocument<200> doc;
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
  // Check if this is a get request (Z2M uses /get topic for retrieving values)
  else if (String(topic) == String(MQTT_GET_TOPIC)) {
    Serial.println("[MQTT PROCESSING] Get request received, publishing current state");
    // Immediately publish current state in response to a get request
    publish_sensor_data(true);
  }
  // Check if this is a bridge request to our device
  else if (String(topic).startsWith(MQTT_Z2M_BRIDGE_TOPIC)) {
    Serial.println("[MQTT PROCESSING] Bridge message received");
    
    if (message.indexOf("\"devices\":") >= 0) {
      Serial.println("[MQTT BRIDGE] Devices list request detected, republishing device info");
      // This is likely a devices list request, re-publish our device info
      publish_device_discovery();
    } else {
      Serial.println("[MQTT BRIDGE] Unhandled bridge message type");
    }
  } else {
    Serial.println("[MQTT WARNING] Message received on unexpected topic");
  }
}

void publish_device_discovery() {
  // Publish device information to Zigbee2MQTT
  StaticJsonDocument<768> device_doc; // Increased size to accommodate all device fields
  
  // Device definition information
  device_doc["name"] = "rd03e_radar";
  device_doc["description"] = "24GHz Radar Presence Sensor";
  device_doc["model"] = "RD03E";
  device_doc["vendor"] = "ai-thinker";
  device_doc["supported"] = true;
  device_doc["id"] = "30EFD30EF";
  // Create exposes array to describe capabilities
  JsonArray exposes = device_doc.createNestedArray("exposes");
  
  // Presence binary sensor
  JsonObject presence = exposes.createNestedObject();
  presence["type"] = "binary";
  presence["name"] = "presence";
  presence["property"] = "presence";
  presence["access"] = 1; // Read-only: 1, Read/Write: 3
  presence["value_on"] = true;
  presence["value_off"] = false;
  presence["description"] = "Indicates presence detection";
  
  // Movement binary sensor
  JsonObject movement = exposes.createNestedObject();
  movement["type"] = "binary";
  movement["name"] = "movement";
  movement["property"] = "movement";
  movement["access"] = 1;
  movement["value_on"] = true;
  movement["value_off"] = false;
  movement["description"] = "Indicates movement detection";
  
  // Distance sensor
  JsonObject distance = exposes.createNestedObject();
  distance["type"] = "numeric";
  distance["name"] = "distance";
  distance["property"] = "distance";
  distance["access"] = 1;
  distance["unit"] = "m";
  distance["description"] = "Distance to detected object";
  
  // Detection distance setting
  JsonObject detection_distance_setting = exposes.createNestedObject();
  detection_distance_setting["type"] = "numeric";
  detection_distance_setting["name"] = "detection_distance";
  detection_distance_setting["property"] = "detection_distance";
  detection_distance_setting["access"] = 3; // Read/Write
  detection_distance_setting["unit"] = "m";
  detection_distance_setting["min"] = 0.5;
  detection_distance_setting["max"] = 6.0;
  detection_distance_setting["description"] = "Detection distance setting";
  
  // Sensitivity setting
  JsonObject sensitivity_setting = exposes.createNestedObject();
  sensitivity_setting["type"] = "numeric";
  sensitivity_setting["name"] = "sensitivity";
  sensitivity_setting["property"] = "sensitivity";
  sensitivity_setting["access"] = 3; // Read/Write
  sensitivity_setting["min"] = 1;
  sensitivity_setting["max"] = 10;
  sensitivity_setting["description"] = "Sensitivity setting";
  
  // Define device metadata
  JsonObject device_info = device_doc.createNestedObject("device");
  device_info["manufacturer"] = "ai-thinker";
  device_info["model"] = "RD03E";
  device_info["sw_version"] = "1.0.0";
  device_info["hw_version"] = "1.0";
  device_info["id"] = "D30EFD30EF";
  char buffer[768];
  
  // Serialize and check for errors
  size_t json_length = serializeJson(device_doc, buffer, sizeof(buffer));
  
  if (json_length > 0 && json_length < sizeof(buffer)) {
    // Publish device definition to Zigbee2MQTT devices topic
    String device_topic = String(MQTT_Z2M_DEVICES_TOPIC) + "/" + MQTT_DEVICE_ID;
    
    // Debug - Log MQTT device discovery publish
    Serial.print("[MQTT DISCOVERY] Topic: ");
    Serial.print(device_topic);
    Serial.print(" Payload Length: ");
    Serial.println(json_length);
    
    bool pub_result = mqtt_client.publish(device_topic.c_str(), buffer, true);
    
    if (pub_result) {
      Serial.println("[MQTT DISCOVERY] Successfully published device information");
    } else {
      Serial.print("[MQTT ERROR] Failed to publish device discovery information. Length: ");
      Serial.print(json_length);
      Serial.print(", State: ");
      Serial.println(mqtt_client.state());
    }
  } else {
    Serial.print("[MQTT ERROR] JSON serialization failed or buffer too small. Length: ");
    Serial.println(json_length);
    
    // Try with a smaller payload if the full one is too large
    StaticJsonDocument<256> minimal_doc;
    minimal_doc["name"] = "rd03e_radar";
    minimal_doc["model"] = "RD03E";
    minimal_doc["id"] = "D30EFD30EF";
    
    char small_buffer[256];
    size_t small_length = serializeJson(minimal_doc, small_buffer, sizeof(small_buffer));
    
    Serial.print("[MQTT RECOVERY] Attempting smaller payload with length: ");
    Serial.println(small_length);
    
    String device_topic = String(MQTT_Z2M_DEVICES_TOPIC) + "/" + MQTT_DEVICE_ID;
    bool pub_result = mqtt_client.publish(device_topic.c_str(), small_buffer, true);
    Serial.print("[MQTT RECOVERY] Fallback publish result: ");
    Serial.println(pub_result ? "OK" : "Failed");
  }
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
        radar.set_sensitivity(sensitivity);
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
  
  // Publish updated configuration if changed
  if (config_changed) {
    Serial.println("[CONFIG] Configuration changed, publishing updated state");
    publish_sensor_data(true);
  } else {
    Serial.println("[CONFIG] No configuration changes applied");
  }
}