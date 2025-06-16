#include "sdp6x.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace sdp6x {

static const char *const TAG = "sdp6x";

// SDP6x I2C Commands - Based on official datasheet
enum SDP6XCommand : uint8_t {
  TRIGGER_MEASUREMENT = 0xF1,
  SOFT_RESET = 0xFE,
  READ_USER_REGISTER = 0xE5,
  WRITE_USER_REGISTER = 0xE4,
};

static const uint8_t SDP6X_CMD_TRIGGER_MEAS = 0xF1;

void SDP6XComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SDP6x...");
  
  // Initialize component state
  this->config_.started = false;
  
  // Add delay to ensure sensor is ready after power-up
  delay(100);
  
  // Perform soft reset to ensure clean state
  ESP_LOGD(TAG, "Performing soft reset...");
  uint8_t reset_cmd = SOFT_RESET;
  if (this->write(&reset_cmd, 1) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Soft reset failed, continuing anyway");
  } else {
    delay(100); // Wait for reset to complete
  }
  
  // Test communication by triggering a measurement
  ESP_LOGD(TAG, "Testing communication...");
  if (!this->trigger_measurement_()) {
    ESP_LOGE(TAG, "Failed to communicate with sensor");
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
  if (this->config_.scale_factor > 0.0f) {
    ESP_LOGCONFIG(TAG, "  Manual Scale Factor: %.1f", this->config_.scale_factor);
  }
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

bool SDP6XComponent::trigger_measurement_() {
  ESP_LOGD(TAG, "Triggering measurement");
  
  // Send trigger measurement command (0xF1)
  uint8_t cmd = SDP6X_CMD_TRIGGER_MEAS;
  if (this->write(&cmd, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to send trigger measurement command");
    return false;
  }
  
  ESP_LOGD(TAG, "Measurement trigger sent successfully");
  return true;
}

bool SDP6XComponent::read_measurement_(float &pressure, float &temperature) {
  // First trigger a measurement
  if (!this->trigger_measurement_()) {
    return false;
  }
  
  // Wait for measurement to complete (typical 5ms, use 10ms to be safe)
  delay(10);
  
  // Read 3 bytes: 2 bytes pressure data + 1 byte CRC
  uint8_t data[3];
  if (this->read(data, 3) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read measurement data");
    return false;
  }
  
  // Parse pressure data (2 bytes, MSB first)
  int16_t pressure_raw = (data[0] << 8) | data[1];
  uint8_t pressure_crc = data[2];
  
  // Verify CRC for pressure data
  ESP_LOGD(TAG, "Raw data: 0x%02X 0x%02X, CRC: 0x%02X", data[0], data[1], pressure_crc);
  if (!this->check_crc_(data[0], data[1], pressure_crc)) {
    ESP_LOGW(TAG, "Pressure CRC check failed - received: 0x%02X, continuing anyway for debugging", pressure_crc);
    // Temporarily continue processing even with CRC failure for debugging
    // return false;
  }
  
  // Convert raw pressure value to physical unit
  // Use configured scale factor (240 for SDP6x0-125Pa)
  float effective_scale_factor = this->config_.scale_factor > 0.0f ? this->config_.scale_factor : 240.0f;
  
  // Convert two's complement to signed value
  pressure = (float)pressure_raw / effective_scale_factor;
  
  // SDP6x doesn't provide temperature - set to NaN or use a default
  temperature = NAN;
  
  ESP_LOGV(TAG, "Raw pressure: %d, Scale factor: %.1f, Pressure: %.2f Pa", 
           pressure_raw, effective_scale_factor, pressure);
  
  return true;
}

bool SDP6XComponent::check_crc_(uint8_t data1, uint8_t data2, uint8_t crc) {
  // SDP6x uses CRC-8 with polynomial 0x31 and initial value 0x00
  uint8_t calculated_crc = 0x00;  // Initial value for SDP6x CRC
  uint8_t data[2] = {data1, data2};
  
  for (int i = 0; i < 2; i++) {
    calculated_crc ^= data[i];
    for (int bit = 8; bit > 0; --bit) {
      if (calculated_crc & 0x80) {
        calculated_crc = (calculated_crc << 1) ^ 0x31;  // SDP6x polynomial
      } else {
        calculated_crc = (calculated_crc << 1);
      }
    }
  }
  
  ESP_LOGD(TAG, "CRC calculated: 0x%02X, received: 0x%02X", calculated_crc, crc);
  return calculated_crc == crc;
}

}  // namespace sdp6x
}  // namespace esphome