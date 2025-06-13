#include "sdp6x.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace sdp6x {

static const char *const TAG = "sdp6x";

static const uint8_t SDP6X_CMD_START_CONT_MEAS[] = {0x36, 0x03};
static const uint8_t SDP6X_CMD_STOP_CONT_MEAS[] = {0x3F, 0xF9};
static const uint8_t SDP6X_CMD_READ_MEAS[] = {0xE0, 0x00};

void SDP6XComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SDP6x...");
  
  if (!this->start_continuous_measurement_()) {
    ESP_LOGE(TAG, "Failed to start continuous measurement");
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "SDP6x setup complete");
}

void SDP6XComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "SDP6x:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Differential Pressure", this->pressure_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

float SDP6XComponent::get_setup_priority() const { return setup_priority::DATA; }

void SDP6XComponent::update() {
  float pressure, temperature;
  
  if (!this->read_measurement_(pressure, temperature)) {
    ESP_LOGW(TAG, "Failed to read measurement data");
    return;
  }
  
  if (this->pressure_sensor_ != nullptr) {
    this->pressure_sensor_->publish_state(pressure);
  }
  
  if (this->temperature_sensor_ != nullptr) {
    this->temperature_sensor_->publish_state(temperature);
  }
}

bool SDP6XComponent::start_continuous_measurement_() {
  if (!this->write_bytes_raw(SDP6X_CMD_START_CONT_MEAS, sizeof(SDP6X_CMD_START_CONT_MEAS))) {
    ESP_LOGE(TAG, "Failed to send start continuous measurement command");
    return false;
  }
  
  delay(20);
  return true;
}

bool SDP6XComponent::read_measurement_(float &pressure, float &temperature) {
  uint8_t data[9];
  
  if (!this->read_bytes_raw(data, 9)) {
    ESP_LOGE(TAG, "Failed to read measurement data");
    return false;
  }
  
  int16_t pressure_raw = (data[0] << 8) | data[1];
  uint8_t pressure_crc = data[2];
  
  int16_t temp_raw = (data[3] << 8) | data[4];
  uint8_t temp_crc = data[5];
  
  int16_t scale_factor_raw = (data[6] << 8) | data[7];
  uint8_t scale_crc = data[8];
  
  if (!this->check_crc_(data[0], data[1], pressure_crc) ||
      !this->check_crc_(data[3], data[4], temp_crc) ||
      !this->check_crc_(data[6], data[7], scale_crc)) {
    ESP_LOGE(TAG, "CRC check failed");
    return false;
  }
  
  pressure = (float)pressure_raw / (float)scale_factor_raw;
  
  temperature = (float)temp_raw / 200.0f;
  
  return true;
}

bool SDP6XComponent::check_crc_(uint8_t data1, uint8_t data2, uint8_t crc) {
  uint8_t calculated_crc = 0xFF;
  uint8_t data[2] = {data1, data2};
  
  for (int i = 0; i < 2; i++) {
    calculated_crc ^= data[i];
    for (int bit = 8; bit > 0; --bit) {
      if (calculated_crc & 0x80) {
        calculated_crc = (calculated_crc << 1) ^ 0x31;
      } else {
        calculated_crc = (calculated_crc << 1);
      }
    }
  }
  
  return calculated_crc == crc;
}

}  // namespace sdp6x
}  // namespace esphome