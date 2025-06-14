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
  void set_pressure_raw(bool raw) { config_.pressure_raw = raw; }
  void set_temperature_raw(bool raw) { config_.temperature_raw = raw; }

  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  struct Config {
    bool pressure_raw = false;     // Use raw pressure values
    bool temperature_raw = false;  // Use raw temperature values
    bool started = false;          // Component started flag
  };

  sensor::Sensor *pressure_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  Config config_;

  bool start_continuous_measurement_();
  bool read_measurement_(float &pressure, float &temperature);
  bool check_crc_(uint8_t data1, uint8_t data2, uint8_t crc);
};

}  // namespace sdp6x
}  // namespace esphome