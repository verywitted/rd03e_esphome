"""Support for RD03-E millimeter wave radar sensors."""

DEPENDENCIES = ['uart', 'binary_sensor', 'sensor', 'json']
AUTO_LOAD = ['binary_sensor', 'sensor', 'json']

# Re-export from sensor.py
from .sensor import CONF_DETECTION_DISTANCE, CONF_SENSITIVITY, rd03e_radar_ns, RD03ERadarSensor, CONFIG_SCHEMA, to_code