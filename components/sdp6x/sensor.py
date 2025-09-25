import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_PRESSURE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_PASCAL,
    UNIT_CELSIUS,
)

DEPENDENCIES = ["i2c"]

# Configuration constants
CONF_DIFFERENTIAL_PRESSURE = "differential_pressure"
CONF_TEMPERATURE = "temperature"
CONF_RAW = "raw"
CONF_SCALE_FACTOR = "scale_factor"

# Namespace and component class
sdp6x_ns = cg.esphome_ns.namespace("sdp6x")
SDP6XComponent = sdp6x_ns.class_("SDP6XComponent", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SDP6XComponent),
            cv.Optional(CONF_DIFFERENTIAL_PRESSURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_PASCAL,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_PRESSURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ).extend(
                {
                    cv.Optional(CONF_RAW, default=False): cv.boolean,
                }
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ).extend(
                {
                    cv.Optional(CONF_RAW, default=False): cv.boolean,
                }
            ),
            cv.Optional(CONF_SCALE_FACTOR): cv.positive_float,
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x40))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_DIFFERENTIAL_PRESSURE in config:
        conf = config[CONF_DIFFERENTIAL_PRESSURE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_pressure_sensor(sens))
        if CONF_RAW in conf:
            cg.add(var.set_pressure_raw(conf[CONF_RAW]))

    if CONF_TEMPERATURE in config:
        conf = config[CONF_TEMPERATURE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_temperature_sensor(sens))
        if CONF_RAW in conf:
            cg.add(var.set_temperature_raw(conf[CONF_RAW]))
    
    if CONF_SCALE_FACTOR in config:
        cg.add(var.set_scale_factor(config[CONF_SCALE_FACTOR]))
