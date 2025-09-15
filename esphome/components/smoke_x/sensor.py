import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
)
from . import SmokeXComponent, smoke_x_ns

DEPENDENCIES = ["smoke_x"]

CONF_SMOKE_X_ID = "smoke_x_id"
CONF_PROBE_INDEX = "probe_index"
CONF_TEMPERATURE = "temperature"
CONF_ALARM_MAX = "alarm_max"
CONF_ALARM_MIN = "alarm_min"

SmokeXSensor = smoke_x_ns.class_("SmokeXSensor", sensor.Sensor, cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_SMOKE_X_ID): cv.use_id(SmokeXComponent),
    cv.Required(CONF_PROBE_INDEX): cv.int_range(min=0, max=3),
    cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
        accuracy_decimals=1,
    ),
    cv.Optional(CONF_ALARM_MAX): sensor.sensor_schema(
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
        accuracy_decimals=0,
    ),
    cv.Optional(CONF_ALARM_MIN): sensor.sensor_schema(
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
        accuracy_decimals=0,
    ),
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_SMOKE_X_ID])
    probe_index = config[CONF_PROBE_INDEX]
    
    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(hub.register_temperature_sensor(sens, probe_index))
    
    if CONF_ALARM_MAX in config:
        sens = await sensor.new_sensor(config[CONF_ALARM_MAX])
        cg.add(hub.register_alarm_max_sensor(sens, probe_index))
    
    if CONF_ALARM_MIN in config:
        sens = await sensor.new_sensor(config[CONF_ALARM_MIN])
        cg.add(hub.register_alarm_min_sensor(sens, probe_index))
