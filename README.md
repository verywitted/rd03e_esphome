# RD03-E Radar Presence Sensor with MQTT

This project implements a presence, movement, and distance sensor using the RD03-E 24GHz radar module from AI-Thinker with an ESP32 microcontroller. The sensor data is published to MQTT topics for integration with home automation systems like Home Assistant.

## Features

- Detects human presence and movement
- Measures distance to detected objects
- Configurable detection distance (0.5m - 6.0m)
- Configurable sensitivity (1-10)
- Real-time updates via MQTT
- Easy integration with Home Assistant through MQTT discovery
- Remote configuration through MQTT topics

## Hardware Requirements

- ESP32 development board
- RD03-E 24GHz radar module
- UART connection cables
- Power supply (5V)

## Wiring

Connect the RD03-E radar module to the ESP32 as follows:

- RD03-E VCC → ESP32 5V
- RD03-E GND → ESP32 GND
- RD03-E OUT → ESP32 GPIO16 (RX)
- RD03-E IN → ESP32 GPIO17 (TX)

## Software Requirements

- PlatformIO (or Arduino IDE with ESP32 support)
- Libraries:
  - PubSubClient (MQTT client)
  - ArduinoJson

## Configuration

Copy the example secrets file to create your own configuration:

```bash
cp include/secrets.h.example include/secrets.h
```

Then edit `include/secrets.h` to set your WiFi and MQTT broker details:

```cpp
// WiFi credentials
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"

// MQTT Broker settings
#define MQTT_SERVER "192.168.1.x"  // Replace with your MQTT broker IP address
#define MQTT_PORT 1883
#define MQTT_USER ""               // Leave empty if no authentication required
#define MQTT_PASSWORD ""           // Replace if you use MQTT authentication
```

The `secrets.h` file is in the `.gitignore` list to prevent accidentally committing your credentials to a public repository.

## Zigbee2MQTT Integration

This project integrates with Zigbee2MQTT by appearing as a virtual Zigbee device:

### MQTT Topics

The device uses the following Zigbee2MQTT-compatible topics:

- `zigbee2mqtt/rd03e_radar` - Main device topic with all sensor data
- `zigbee2mqtt/rd03e_radar/set` - Topic for setting configuration
- `zigbee2mqtt/rd03e_radar/get` - Topic for requesting current state
- `zigbee2mqtt/rd03e_radar/availability` - Device availability status
- `zigbee2mqtt/bridge/devices/rd03e_radar` - Device definition for Zigbee2MQTT

The device registers itself with Zigbee2MQTT on startup by publishing its device definition to the bridge topic. This makes it appear in the Zigbee2MQTT dashboard alongside your other Zigbee devices.

## Remote Configuration

You can change the sensor configuration through the Zigbee2MQTT dashboard or by publishing a JSON payload to the config topic:

```json
{
  "detection_distance": 4.0,
  "sensitivity": 7
}
```

Using the Zigbee2MQTT dashboard, these settings will appear as adjustable sliders.

## Setup and Building

1. Run the initialization script to set up the project:

```bash
./init.sh
```

2. Edit `include/secrets.h` with your WiFi and MQTT credentials

3. Build and flash using PlatformIO:

```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

## Home Assistant Integration

### Method 1: Via Zigbee2MQTT (Recommended)

If you already have Zigbee2MQTT integrated with Home Assistant, the radar sensor will automatically appear in Home Assistant after it connects to Zigbee2MQTT. No additional configuration required!

### Method 2: Direct MQTT Configuration

Alternatively, you can configure direct MQTT integration in your `configuration.yaml`:

```yaml
mqtt:
  binary_sensor:
    - name: "Radar Presence"
      state_topic: "zigbee2mqtt/rd03e_radar"
      value_template: "{{ value_json.presence }}"
      device_class: occupancy
      
    - name: "Radar Movement"
      state_topic: "zigbee2mqtt/rd03e_radar"
      value_template: "{{ value_json.movement }}"
      device_class: motion
      
  sensor:
    - name: "Radar Distance"
      state_topic: "zigbee2mqtt/rd03e_radar"
      value_template: "{{ value_json.distance }}"
      unit_of_measurement: "m"
      device_class: distance

# Configuration controls
number:
  - platform: mqtt
    name: "Radar Detection Distance"
    state_topic: "zigbee2mqtt/rd03e_radar"
    value_template: "{{ value_json.detection_distance }}"
    command_topic: "zigbee2mqtt/rd03e_radar/set/detection_distance"
    min: 0.5
    max: 6.0
    step: 0.5
    unit_of_measurement: "m"
    
  - platform: mqtt
    name: "Radar Sensitivity"
    state_topic: "zigbee2mqtt/rd03e_radar"
    value_template: "{{ value_json.sensitivity }}"
    command_topic: "zigbee2mqtt/rd03e_radar/set/sensitivity"
    min: 1
    max: 10
    step: 1
```

## License

MIT License

## Credits

- RD03-E radar protocol implementation based on the official datasheet
- Original implementation inspired by ESPHome component structure