import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome import pins  # <-- add this
from esphome.const import CONF_ID

DEPENDENCIES = ["i2c", "lvgl"]

spd2010_ns = cg.esphome_ns.namespace("spd2010_touch")
SPD2010Touch = spd2010_ns.class_("SPD2010Touch", cg.Component, i2c.I2CDevice)

CONF_INT_PIN = "int_pin"
CONF_RST_PIN = "rst_pin"
CONF_SWAP_XY = "swap_xy"
CONF_MIRROR_X = "mirror_x"
CONF_MIRROR_Y = "mirror_y"
CONF_WIDTH = "width"
CONF_HEIGHT = "height"
CONF_POLL_INTERVAL = "poll_interval"

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(SPD2010Touch),

        # Use the pins helper schemas
        cv.Optional(CONF_INT_PIN): pins.gpio_input_pin_schema,   # active-low interrupt
        cv.Optional(CONF_RST_PIN): pins.gpio_output_pin_schema,  # reset output

        cv.Optional(CONF_WIDTH, default=412): cv.uint16_t,
        cv.Optional(CONF_HEIGHT, default=412): cv.uint16_t,

        cv.Optional(CONF_SWAP_XY, default=False): cv.boolean,
        cv.Optional(CONF_MIRROR_X, default=False): cv.boolean,
        cv.Optional(CONF_MIRROR_Y, default=False): cv.boolean,

        cv.Optional(CONF_POLL_INTERVAL, default="10ms"): cv.time_period,
    })
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x53))  # SPD2010 fixed I2C address
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_screen_size(config[CONF_WIDTH], config[CONF_HEIGHT]))
    cg.add(var.set_orientation(
        config[CONF_SWAP_XY],
        config[CONF_MIRROR_X],
        config[CONF_MIRROR_Y]
    ))

    cg.add(var.set_poll_interval(cg.uint32(config[CONF_POLL_INTERVAL].total_milliseconds)))
    # Resolve pin expressions via codegen (supports expander-backed pins)
    if CONF_INT_PIN in config:
        int_pin = await cg.gpio_pin_expression(config[CONF_INT_PIN])
        cg.add(var.set_int_pin(int_pin))

    if CONF_RST_PIN in config:
        rst_pin = await cg.gpio_pin_expression(config[CONF_RST_PIN])
        cg.add(var.set_rst_pin(rst_pin))