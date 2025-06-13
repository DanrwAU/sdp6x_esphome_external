import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_PRESSURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_PASCAL,
)

DEPENDENCIES = ["i2c"]

sdp6x_ns = cg.esphome_ns.namespace("sdp6x")
SDP6XComponent = sdp6x_ns.class_("SDP6XComponent", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SDP6XComponent),
            cv.Optional("differential_pressure"): sensor.sensor_schema(
                unit_of_measurement=UNIT_PASCAL,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_PRESSURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional("temperature"): sensor.sensor_schema(
                unit_of_measurement="°C",
                accuracy_decimals=1,
                device_class="temperature",
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x25))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if "differential_pressure" in config:
        sens = await sensor.new_sensor(config["differential_pressure"])
        cg.add(var.set_pressure_sensor(sens))

    if "temperature" in config:
        sens = await sensor.new_sensor(config["temperature"])
        cg.add(var.set_temperature_sensor(sens))