#include "sdp6x.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cmath>

namespace esphome {
namespace sdp6x {

static const char *const TAG = "sdp6x";

static const uint8_t SDP6X_CMD_SOFT_RESET = 0xFE;
static const uint16_t SDP6X_CMD_START_CONTINUOUS = 0x3603;
static const uint16_t SDP6X_CMD_READ_MEASUREMENT = 0xE000;
static const float SDP6X_TEMPERATURE_DIVISOR = 200.0f;

void SDP6XComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SDP6x...");
  
  // Initialize component state
  this->config_.started = false;
  
  // Add delay to ensure sensor is ready after power-up
  delay(50);
  
  // Perform soft reset to ensure clean state
  ESP_LOGD(TAG, "Performing soft reset...");
  uint8_t reset_cmd = SDP6X_CMD_SOFT_RESET;
  if (this->write(&reset_cmd, 1) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Soft reset failed, continuing anyway");
  } else {
    delay(20);  // Wait for reset to complete
  }
  
  // Try to start continuous mode first; fall back to single-shot if it fails
  if (this->start_continuous_measurement_()) {
    ESP_LOGD(TAG, "Continuous measurement mode started");
    this->mode_ = SensorMode::CONTINUOUS;
    this->temperature_supported_ = true;

    // Attempt an initial read to prime scale factor cache
    delay(50);
    int16_t pressure_raw;
    int16_t temperature_raw;
    uint16_t scale_raw;
    if (this->read_measurement_block_(pressure_raw, temperature_raw, scale_raw)) {
      this->sensor_scale_factor_ = static_cast<float>(scale_raw);
      ESP_LOGD(TAG, "Initial scale factor: %.1f", this->sensor_scale_factor_);
    } else {
      ESP_LOGW(TAG, "Initial measurement not available yet; will retry during updates");
    }
  } else {
    ESP_LOGW(TAG, "Continuous mode not acknowledged; switching to single-shot mode");
    this->mode_ = SensorMode::SINGLE_SHOT;
    this->temperature_supported_ = false;

    bool legacy_ready = false;
    int16_t pressure_test;
    for (uint8_t attempt = 0; attempt < 3 && !legacy_ready; ++attempt) {
      if (this->read_legacy_measurement_(pressure_test)) {
        legacy_ready = true;
        break;
      }
      ESP_LOGW(TAG, "Legacy measurement attempt %u failed, retrying", static_cast<unsigned>(attempt + 1));
      delay(25);
    }

    if (!legacy_ready) {
      ESP_LOGE(TAG, "Single-shot warm-up failed; keeping component active for retries");
      this->status_set_warning("Waiting for sensor");
    } else {
      this->status_clear_warning();
      ESP_LOGD(TAG, "Legacy mode initial pressure sample: %d counts", pressure_test);
    }

    if (this->config_.scale_factor > 0.0f) {
      this->sensor_scale_factor_ = this->config_.scale_factor;
    } else {
      this->sensor_scale_factor_ = 0.0f;
      ESP_LOGW(TAG, "Set 'scale_factor' in YAML to convert raw pressure counts in single-shot mode");
    }
    ESP_LOGCONFIG(TAG, "Legacy single-shot mode active");
  }
  
  this->config_.started = true;
  ESP_LOGCONFIG(TAG, "SDP6x setup complete");
}

void SDP6XComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "SDP6x:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  const char *mode_str = "unknown";
  switch (this->mode_) {
    case SensorMode::CONTINUOUS:
      mode_str = "continuous";
      break;
    case SensorMode::SINGLE_SHOT:
      mode_str = "single-shot";
      break;
    default:
      break;
  }
  ESP_LOGCONFIG(TAG, "  Mode: %s", mode_str);
  if (this->config_.scale_factor > 0.0f) {
    ESP_LOGCONFIG(TAG, "  Manual Scale Factor: %.1f", this->config_.scale_factor);
  } else if (this->sensor_scale_factor_ > 0.0f) {
    ESP_LOGCONFIG(TAG, "  Sensor Scale Factor: %.1f", this->sensor_scale_factor_);
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

  switch (this->mode_) {
    case SensorMode::CONTINUOUS: {
      int16_t pressure_raw;
      int16_t temperature_raw;
      uint16_t scale_raw;

      if (!this->read_measurement_block_(pressure_raw, temperature_raw, scale_raw)) {
        ESP_LOGW(TAG, "Failed to read measurement data");
        this->status_set_warning("Failed to read sensor data");
        return;
      }

      if (scale_raw > 0) {
        this->sensor_scale_factor_ = static_cast<float>(scale_raw);
      }

      float effective_scale = this->config_.scale_factor > 0.0f
                                  ? this->config_.scale_factor
                                  : (scale_raw > 0 ? static_cast<float>(scale_raw) : this->sensor_scale_factor_);
      // Prefer manual scale factor, otherwise use the most recent value reported by the sensor.
      if (effective_scale <= 0.0f) {
        ESP_LOGW(TAG, "Effective scale factor invalid: %.1f", effective_scale);
        this->status_set_warning("Invalid scale factor");
        return;
      }

      float pressure_processed = static_cast<float>(pressure_raw) / effective_scale;
      float temperature_processed = static_cast<float>(temperature_raw) / SDP6X_TEMPERATURE_DIVISOR;

      this->status_clear_warning();

      if (this->pressure_sensor_ != nullptr) {
        float publish_value = this->config_.pressure_raw ? static_cast<float>(pressure_raw) : pressure_processed;
        this->pressure_sensor_->publish_state(publish_value);
        ESP_LOGD(TAG, "Pressure raw=%d, processed=%.3f Pa (scale %.1f, raw_mode=%s)",
                 pressure_raw, pressure_processed, effective_scale, YESNO(this->config_.pressure_raw));
      }

      if (this->temperature_sensor_ != nullptr && this->temperature_supported_) {
        float publish_value = this->config_.temperature_raw ? static_cast<float>(temperature_raw) : temperature_processed;
        this->temperature_sensor_->publish_state(publish_value);
        ESP_LOGD(TAG, "Temperature raw=%d, processed=%.2f °C (raw_mode=%s)",
                 temperature_raw, temperature_processed, YESNO(this->config_.temperature_raw));
      } else if (this->temperature_sensor_ != nullptr) {
        this->temperature_sensor_->publish_state(NAN);
      }
      break;
    }

    case SensorMode::SINGLE_SHOT: {
      int16_t pressure_raw;
      if (!this->read_legacy_measurement_(pressure_raw)) {
        ESP_LOGW(TAG, "Failed to read single-shot measurement");
        this->status_set_warning("Failed to read sensor data");
        return;
      }

      float effective_scale = this->config_.scale_factor > 0.0f ? this->config_.scale_factor : this->sensor_scale_factor_;
      if (effective_scale <= 0.0f) {
        // No scale factor available from sensor; require manual override
        ESP_LOGW(TAG, "Provide scale_factor in config for single-shot mode");
        this->status_set_warning("Missing scale factor");
        return;
      }

      float pressure_processed = static_cast<float>(pressure_raw) / effective_scale;

      this->status_clear_warning();

      if (this->pressure_sensor_ != nullptr) {
        float publish_value = this->config_.pressure_raw ? static_cast<float>(pressure_raw) : pressure_processed;
        this->pressure_sensor_->publish_state(publish_value);
        ESP_LOGD(TAG, "Pressure raw=%d, processed=%.3f Pa (scale %.1f, raw_mode=%s)",
                 pressure_raw, pressure_processed, effective_scale, YESNO(this->config_.pressure_raw));
      }

      if (this->temperature_sensor_ != nullptr) {
        // Temperature is not available in single-shot mode
        this->temperature_sensor_->publish_state(NAN);
      }
      break;
    }

    case SensorMode::UNKNOWN:
    default:
      ESP_LOGW(TAG, "Sensor mode not initialised");
      this->status_set_warning("Sensor not initialised");
      break;
  }
}

bool SDP6XComponent::start_continuous_measurement_() {
  uint8_t cmd[2] = {static_cast<uint8_t>(SDP6X_CMD_START_CONTINUOUS >> 8),
                    static_cast<uint8_t>(SDP6X_CMD_START_CONTINUOUS & 0xFF)};
  if (this->write(cmd, sizeof(cmd)) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to start continuous measurement (cmd=0x%02X%02X)", cmd[0], cmd[1]);
    return false;
  }
  return true;
}

bool SDP6XComponent::read_measurement_block_(int16_t &pressure_raw, int16_t &temperature_raw, uint16_t &scale_raw) {
  uint8_t read_cmd[2] = {static_cast<uint8_t>(SDP6X_CMD_READ_MEASUREMENT >> 8),
                         static_cast<uint8_t>(SDP6X_CMD_READ_MEASUREMENT & 0xFF)};
  if (this->write(read_cmd, sizeof(read_cmd)) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to issue read measurement command");
    return false;
  }

  delay(5);  // Allow sensor to prepare the data

  // Sensor returns pressure, temperature, and scale factor segments (each 2 bytes + CRC)
  uint8_t data[9];
  if (this->read(data, sizeof(data)) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to read measurement bytes");
    return false;
  }

  ESP_LOGV(TAG, "Raw buffer: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
           data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);

  if (!this->check_crc_(&data[0], 2, data[2])) {
    ESP_LOGW(TAG, "Pressure CRC check failed (got 0x%02X)", data[2]);
    return false;
  }
  if (!this->check_crc_(&data[3], 2, data[5])) {
    ESP_LOGW(TAG, "Temperature CRC check failed (got 0x%02X)", data[5]);
    return false;
  }
  if (!this->check_crc_(&data[6], 2, data[8])) {
    ESP_LOGW(TAG, "Scale factor CRC check failed (got 0x%02X)", data[8]);
    return false;
  }

  uint16_t pressure_u = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  uint16_t temperature_u = (static_cast<uint16_t>(data[3]) << 8) | data[4];
  scale_raw = (static_cast<uint16_t>(data[6]) << 8) | data[7];

  pressure_raw = static_cast<int16_t>(pressure_u);
  temperature_raw = static_cast<int16_t>(temperature_u);

  return true;
}

bool SDP6XComponent::read_legacy_measurement_(int16_t &pressure_raw) {
  // Send trigger measurement command (0xF1)
  uint8_t cmd = 0xF1;
  if (this->write(&cmd, 1) != i2c::ERROR_OK) {
    ESP_LOGV(TAG, "Trigger command not acknowledged");
    return false;
  }

  // Sensirion app note recommends up to 100 ms for legacy trigger measurements.
  delay(100);

  uint8_t data[3];
  if (this->read(data, sizeof(data)) != i2c::ERROR_OK) {
    ESP_LOGV(TAG, "Failed to read legacy measurement bytes");
    return false;
  }

  if (!this->check_crc_(data, 2, data[2])) {
    ESP_LOGW(TAG, "Legacy pressure CRC failed (got 0x%02X)", data[2]);
    return false;
  }

  pressure_raw = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);

  return true;
}

bool SDP6XComponent::check_crc_(const uint8_t *data, size_t length, uint8_t crc) {
  uint8_t calculated_crc = 0x00;  // Initial value for SDP6x CRC

  for (size_t i = 0; i < length; i++) {
    calculated_crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if (calculated_crc & 0x80) {
        calculated_crc = static_cast<uint8_t>((calculated_crc << 1) ^ 0x31);
      } else {
        calculated_crc <<= 1;
      }
    }
  }

  return calculated_crc == crc;
}

}  // namespace sdp6x
}  // namespace esphome
