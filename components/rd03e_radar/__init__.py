"""Support for RD03-E millimeter wave radar sensors."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ['@esphome/core']

# Import and re-export the component
CONF_RD03E_RADAR_ID = "rd03e_radar_id"

# Define the namespace and component class
rd03e_radar_ns = cg.esphome_ns.namespace("rd03e_radar")
RD03ERadarSensor = rd03e_radar_ns.class_("RD03ERadarSensor", cg.Component, uart.UARTDevice)

# Re-export from sensor.py
from .sensor import CONFIG_SCHEMA, CONF_DETECTION_DISTANCE, CONF_SENSITIVITY, to_code