import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, binary_sensor, sensor
from esphome.const import (
    CONF_ID, 
    CONF_UART_ID,
    DEVICE_CLASS_OCCUPANCY,
    DEVICE_CLASS_MOTION,
    DEVICE_CLASS_DISTANCE,
    CONF_DISTANCE,
    UNIT_METER,
    ICON_MOTION_SENSOR
)

DEPENDENCIES = ['uart']
AUTO_LOAD = ['binary_sensor', 'sensor']
CODEOWNERS = ['@esphome/core']

# Component name for YAML
COMPONENT_TYPE = "rd03e_radar"

CONF_PRESENCE = "presence"
CONF_MOVEMENT = "movement"
CONF_DETECTION_DISTANCE = "detection_distance"
CONF_SENSITIVITY = "sensitivity"

# Ensure namespace matches the component name
rd03e_radar_ns = cg.esphome_ns.namespace('rd03e_radar')
RD03ERadarSensor = rd03e_radar_ns.class_('RD03ERadarSensor', cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(RD03ERadarSensor),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_OCCUPANCY,
        icon=ICON_MOTION_SENSOR
    ),
    cv.Optional(CONF_MOVEMENT): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_MOTION,
        icon=ICON_MOTION_SENSOR
    ),
    cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
        unit_of_measurement=UNIT_METER,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_DISTANCE
    ),
    cv.Optional(CONF_DETECTION_DISTANCE, default=3.0): cv.float_range(min=0.5, max=6.0),
    cv.Optional(CONF_SENSITIVITY, default=5): cv.int_range(min=1, max=10),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # Get the UART component
    uart_component = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_component))
    
    if CONF_PRESENCE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_PRESENCE])
        cg.add(var.set_presence_sensor(sens))
    
    if CONF_MOVEMENT in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_MOVEMENT])
        cg.add(var.set_movement_sensor(sens))
    
    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))
    
    cg.add(var.set_detection_distance(config[CONF_DETECTION_DISTANCE]))
    cg.add(var.set_sensitivity(config[CONF_SENSITIVITY]))