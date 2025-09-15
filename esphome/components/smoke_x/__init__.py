import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import spi
from esphome.core import CORE
from esphome import pins

DEPENDENCIES = ["spi"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

smoke_x_ns = cg.esphome_ns.namespace("smoke_x")
SmokeXComponent = smoke_x_ns.class_("SmokeXComponent", cg.Component, spi.SPIDevice)

CONF_LORA_RST_PIN = "lora_rst_pin"
CONF_LORA_BUSY_PIN = "lora_busy_pin"
CONF_LORA_DIO1_PIN = "lora_dio1_pin"
CONF_SYNC_FREQUENCY = "sync_frequency"
CONF_NUM_PROBES = "num_probes"
CONF_ENABLED = "enabled"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SmokeXComponent),
    cv.Optional(CONF_LORA_RST_PIN, default=12): pins.gpio_output_pin_schema,
    cv.Optional(CONF_LORA_BUSY_PIN, default=13): pins.gpio_input_pin_schema,
    cv.Optional(CONF_LORA_DIO1_PIN, default=14): pins.gpio_input_pin_schema,
    cv.Optional(CONF_SYNC_FREQUENCY, default=920000000): cv.int_range(min=902000000, max=928000000),
    cv.Optional(CONF_NUM_PROBES, default=4): cv.one_of(2, 4, int=True),
    cv.Optional(CONF_ENABLED, default=False): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA).extend(spi.spi_device_schema(cs_pin_required=True))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
    
    rst_pin = await cg.gpio_pin_expression(config[CONF_LORA_RST_PIN])
    cg.add(var.set_lora_rst_pin(rst_pin))
    
    busy_pin = await cg.gpio_pin_expression(config[CONF_LORA_BUSY_PIN])
    cg.add(var.set_lora_busy_pin(busy_pin))
    
    dio1_pin = await cg.gpio_pin_expression(config[CONF_LORA_DIO1_PIN])
    cg.add(var.set_lora_dio1_pin(dio1_pin))
    
    cg.add(var.set_sync_frequency(config[CONF_SYNC_FREQUENCY]))
    cg.add(var.set_num_probes(config[CONF_NUM_PROBES]))
    cg.add(var.set_enabled(config[CONF_ENABLED]))
