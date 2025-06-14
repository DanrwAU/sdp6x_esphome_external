#include "sdp6x.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace sdp6x {

static const char *const TAG = "sdp6x";

// SDP6x I2C Commands
enum SDP6XCommand : uint16_t {
  START_CONTINUOUS_MEASUREMENT = 0x3603,
  STOP_CONTINUOUS_MEASUREMENT = 0x3FF9,
  READ_MEASUREMENT = 0xE000,
};

static const uint8_t SDP6X_CMD_START_CONT_MEAS[] = {0x36, 0x03};
static const uint8_t SDP6X_CMD_STOP_CONT_MEAS[] = {0x3F, 0xF9};
static const uint8_t SDP6X_CMD_READ_MEAS[] = {0xE0, 0x00};

void SDP6XComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SDP6x...");
  
  // Initialize component state
  this->config_.started = false;
  
  if (!this->start_continuous_measurement_()) {
    ESP_LOGE(TAG, "Failed to start continuous measurement");
    this->mark_failed();
    return;
  }
  
  this->config_.started = true;
  ESP_LOGCONFIG(TAG, "SDP6x setup complete");
}

void SDP6XComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "SDP6x:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Differential Pressure", this->pressure_sensor_);
  if (this->pressure_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "    Raw values: %s", YESNO(this->config_.pressure_raw));
  }
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  if (this->temperature_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "    Raw values: %s", YESNO(this->config_.temperature_raw));
  }
}


void SDP6XComponent::update() {
  if (!this->config_.started) {
    ESP_LOGD(TAG, "Component not started yet");
    return;
  }

  float pressure, temperature;
  
  if (!this->read_measurement_(pressure, temperature)) {
    ESP_LOGW(TAG, "Failed to read measurement data");
    this->status_set_warning("Failed to read sensor data");
    return;
  }
  
  this->status_clear_warning();
  
  if (this->pressure_sensor_ != nullptr) {
    float processed_pressure = this->config_.pressure_raw ? pressure : pressure;
    this->pressure_sensor_->publish_state(processed_pressure);
    ESP_LOGD(TAG, "Pressure: %.2f Pa (raw: %s)", processed_pressure, YESNO(this->config_.pressure_raw));
  }
  
  if (this->temperature_sensor_ != nullptr) {
    float processed_temperature = this->config_.temperature_raw ? temperature : temperature;
    this->temperature_sensor_->publish_state(processed_temperature);
    ESP_LOGD(TAG, "Temperature: %.1f °C (raw: %s)", processed_temperature, YESNO(this->config_.temperature_raw));
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
  
  // Parse pressure data (bytes 0-2)
  int16_t pressure_raw = (data[0] << 8) | data[1];
  uint8_t pressure_crc = data[2];
  
  // Parse temperature data (bytes 3-5)
  int16_t temp_raw = (data[3] << 8) | data[4];
  uint8_t temp_crc = data[5];
  
  // Parse scale factor (bytes 6-8)
  int16_t scale_factor_raw = (data[6] << 8) | data[7];
  uint8_t scale_crc = data[8];
  
  // Verify CRC for all data segments
  if (!this->check_crc_(data[0], data[1], pressure_crc)) {
    ESP_LOGE(TAG, "Pressure CRC check failed");
    return false;
  }
  
  if (!this->check_crc_(data[3], data[4], temp_crc)) {
    ESP_LOGE(TAG, "Temperature CRC check failed");
    return false;
  }
  
  if (!this->check_crc_(data[6], data[7], scale_crc)) {
    ESP_LOGE(TAG, "Scale factor CRC check failed");
    return false;
  }
  
  // Convert raw values to physical units
  // Pressure: raw value divided by scale factor gives pressure in Pa
  if (scale_factor_raw == 0) {
    ESP_LOGE(TAG, "Invalid scale factor (zero)");
    return false;
  }
  
  pressure = (float)pressure_raw / (float)scale_factor_raw;
  
  // Temperature: raw value divided by 200 gives temperature in °C
  temperature = (float)temp_raw / 200.0f;
  
  ESP_LOGV(TAG, "Raw values - Pressure: %d, Temperature: %d, Scale: %d", 
           pressure_raw, temp_raw, scale_factor_raw);
  
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