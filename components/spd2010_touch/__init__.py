
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import i2c

# Namespace & class binding
spd_ns = cg.esphome_ns.namespace("spd2010_glue")
Spd2010LvglGlue = spd_ns.class_("Spd2010LvglGlue", cg.Component)

# Config keys
CONF_WIDTH     = "width"
CONF_HEIGHT    = "height"
CONF_INT_GPIO  = "int_gpio"
CONF_SWAP_XY   = "swap_xy"
CONF_MIRROR_X  = "mirror_x"
CONF_MIRROR_Y  = "mirror_y"
CONF_I2C_ID    = "i2c_id"

DEPENDENCIES = ["lvgl", "i2c"]   # we feed LVGL directly, and now use the shared i2c bus
AUTO_LOAD    = ["lvgl"]         # ensure lvgl is present

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(Spd2010LvglGlue),
        # Reference to an existing `i2c:` bus defined in YAML (e.g. id: bus_imu_touch_rtc)
        cv.GenerateID(CONF_I2C_ID): cv.use_id(i2c.I2CBus),
        cv.Optional(CONF_WIDTH, default=412): cv.uint16_t,
        cv.Optional(CONF_HEIGHT, default=412): cv.uint16_t,
        cv.Optional(CONF_INT_GPIO, default=4): cv.int_,          # use plain integer (e.g., 4)
        cv.Optional(CONF_SWAP_XY, default=False): cv.boolean,
        cv.Optional(CONF_MIRROR_X, default=False): cv.boolean,
        cv.Optional(CONF_MIRROR_Y, default=False): cv.boolean,
    })
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    i2c_bus = await cg.get_variable(config[CONF_I2C_ID])
    cg.add(var.set_i2c_bus(i2c_bus))

    cg.add(var.set_screen_size(config[CONF_WIDTH], config[CONF_HEIGHT]))
    cg.add(var.set_int_gpio(config[CONF_INT_GPIO]))
    cg.add(var.set_swap_xy(config[CONF_SWAP_XY]))
    cg.add(var.set_mirror_x(config[CONF_MIRROR_X]))
    cg.add(var.set_mirror_y(config[CONF_MIRROR_Y]))
