#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace sdp6x {

class SDP6XComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_pressure_sensor(sensor::Sensor *pressure_sensor) { pressure_sensor_ = pressure_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }

  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override;

 protected:
  sensor::Sensor *pressure_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  bool start_continuous_measurement_();
  bool read_measurement_(float &pressure, float &temperature);
  bool check_crc_(uint8_t data1, uint8_t data2, uint8_t crc);
};

}  // namespace sdp6x
}  // namespace esphome