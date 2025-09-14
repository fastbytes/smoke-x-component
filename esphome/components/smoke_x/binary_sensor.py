import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID, DEVICE_CLASS_CONNECTIVITY
from . import SmokeXComponent, smoke_x_ns

DEPENDENCIES = ["smoke_x"]

CONF_SMOKE_X_ID = "smoke_x_id"
CONF_PROBE_INDEX = "probe_index"
CONF_PROBE_ATTACHED = "probe_attached"
CONF_PROBE_ALARM = "probe_alarm"
CONF_BILLOWS_ATTACHED = "billows_attached"
CONF_SYNCED = "synced"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_SMOKE_X_ID): cv.use_id(SmokeXComponent),
    cv.Optional(CONF_PROBE_INDEX): cv.int_range(min=0, max=3),
    cv.Optional(CONF_PROBE_ATTACHED): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_CONNECTIVITY,
    ),
    cv.Optional(CONF_PROBE_ALARM): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_BILLOWS_ATTACHED): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_CONNECTIVITY,
    ),
    cv.Optional(CONF_SYNCED): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_CONNECTIVITY,
    ),
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_SMOKE_X_ID])
    
    if CONF_PROBE_ATTACHED in config:
        probe_index = config[CONF_PROBE_INDEX]
        sens = await binary_sensor.new_binary_sensor(config[CONF_PROBE_ATTACHED])
        cg.add(hub.register_probe_attached_sensor(sens, probe_index))
    
    if CONF_PROBE_ALARM in config:
        probe_index = config[CONF_PROBE_INDEX]
        sens = await binary_sensor.new_binary_sensor(config[CONF_PROBE_ALARM])
        cg.add(hub.register_probe_alarm_sensor(sens, probe_index))
    
    if CONF_BILLOWS_ATTACHED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_BILLOWS_ATTACHED])
        cg.add(hub.register_billows_attached_sensor(sens))
    
    if CONF_SYNCED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_SYNCED])
        cg.add(hub.register_synced_sensor(sens))
