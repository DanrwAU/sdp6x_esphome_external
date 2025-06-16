# ESPHome SDP6x Sensor Component

This is a custom ESPHome component for the Sensirion SDP6x series differential pressure sensors.

## Features

- Reads differential pressure in Pascals
- Reads temperature in Celsius  
- I2C communication with CRC validation
- Raw value output option
- Configurable update intervals
- Comprehensive error handling and status reporting

## Usage

Add this component to your ESPHome configuration:

```yaml
external_components:
  - source: github://DanrwAU/sdp6x_esphome_external
    components: [sdp6x]

i2c:
  sda: GPIO21
  scl: GPIO22

sensor:
  - platform: sdp6x
    address: 0x40  # I2C address (default: 0x40)
    scale_factor: 240.0  # Optional: manual scale factor override
    differential_pressure:
      name: "Differential Pressure"
      raw: false  # Set to true for raw sensor values
    temperature:
      name: "Temperature"
      raw: false  # Set to true for raw sensor values
    update_interval: 60s
```

## Configuration Options

### Differential Pressure Sensor
- **name** (Required): Name for the pressure sensor
- **raw** (Optional, default: false): Output raw sensor values instead of processed values
- All standard ESPHome sensor options (filters, etc.)

### Temperature Sensor  
- **name** (Required): Name for the temperature sensor
- **raw** (Optional, default: false): Output raw sensor values instead of processed values
- All standard ESPHome sensor options (filters, etc.)

### Component Options
- **update_interval** (Optional, default: 60s): How often to read the sensor
- **address** (Optional, default: 0x40): I2C address of the sensor
- **scale_factor** (Optional): Manual scale factor override for pressure calculations. If not specified, uses the scale factor from the sensor itself

## Hardware Connections

- **VCC**: 3.3V
- **GND**: Ground
- **SDA**: I2C Data (default GPIO21 on ESP32)
- **SCL**: I2C Clock (default GPIO22 on ESP32)

**Important**: Use pull-up resistors (2.5k-10k ohms) on SDA and SCL lines for reliable I2C communication.

## Supported Sensors

- SDP600 series differential pressure sensors
- Default I2C address: 0x40
- Voltage: 3.3V operation

## Example Configuration

```yaml
# Complete example configuration
external_components:
  - source: github://DanrwAU/sdp6x_esphome_external
    components: [sdp6x]

i2c:
  sda: GPIO21
  scl: GPIO22
  scan: true

sensor:
  - platform: sdp6x
    address: 0x40
    scale_factor: 240.0  # Optional: manual scale factor override
    update_interval: 30s
    differential_pressure:
      name: "Room Pressure Difference"
      unit_of_measurement: "Pa"
      accuracy_decimals: 2
      filters:
        - sliding_window_moving_average:
            window_size: 5
            send_every: 3
    temperature:
      name: "Sensor Temperature"
      unit_of_measurement: "°C"
      accuracy_decimals: 1
      filters:
        - offset: -2.0  # Calibration offset if needed
```

## Troubleshooting

- **Sensor not found**: Check I2C wiring and use `scan: true` in I2C config to verify address
- **CRC errors**: Check power supply stability and I2C pull-up resistors
- **Intermittent readings**: Ensure stable 3.3V power supply and proper grounding