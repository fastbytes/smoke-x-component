import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID
from . import SmokeXComponent, smoke_x_ns

DEPENDENCIES = ["smoke_x"]

CONF_SMOKE_X_ID = "smoke_x_id"
CONF_DEVICE_ID = "device_id"
CONF_UNITS = "units"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_SMOKE_X_ID): cv.use_id(SmokeXComponent),
    cv.Optional(CONF_DEVICE_ID): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_UNITS): text_sensor.text_sensor_schema(),
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_SMOKE_X_ID])
    
    if CONF_DEVICE_ID in config:
        sens = await text_sensor.new_text_sensor(config[CONF_DEVICE_ID])
        cg.add(hub.register_device_id_sensor(sens))
    
    if CONF_UNITS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_UNITS])
        cg.add(hub.register_units_sensor(sens))
