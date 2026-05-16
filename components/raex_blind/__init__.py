import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi
from esphome.const import CONF_FREQUENCY, CONF_ID

DEPENDENCIES = ["spi"]
AUTO_LOAD = ["api"]  # uses api::CustomAPIDevice

CONF_GDO0_PIN = "gdo0_pin"
CONF_RX_PIN = "rx_pin"

# Pure RF + position-engine hub. Each blind (identity, aliases, travel times,
# micro-step, persistence) is defined per-cover; see cover/__init__.py.
raex_blind_ns = cg.esphome_ns.namespace("raex_blind")
RaexBlindTransmitComponent = raex_blind_ns.class_(
    "RaexBlindTransmitComponent", cg.Component, spi.SPIDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RaexBlindTransmitComponent),
            cv.Required(CONF_GDO0_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_RX_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_FREQUENCY, default="433.92MHz"): cv.frequency,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True, default_data_rate="4MHz"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)  # owns the cs_pin entirely
    cg.add(var.set_frequency(config[CONF_FREQUENCY]))
    gdo0 = await cg.gpio_pin_expression(config[CONF_GDO0_PIN])
    cg.add(var.set_gdo0_pin(gdo0))
    if CONF_RX_PIN in config:
        rx = await cg.gpio_pin_expression(config[CONF_RX_PIN])
        cg.add(var.set_rx_pin(rx))
