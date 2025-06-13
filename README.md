# ESPHome SDP6x Sensor Component

This is a custom ESPHome component for the Sensirion SDP6x series differential pressure sensors.

## Features

- Reads differential pressure in Pascals
- Reads temperature in Celsius  
- I2C communication
- CRC validation for data integrity
- Configurable update intervals

## Usage

Add this component to your ESPHome configuration:

```yaml
external_components:
  - source: github://your-username/esphome_external_joe
    components: [sdp6x]

i2c:
  sda: GPIO21
  scl: GPIO22

sensor:
  - platform: sdp6x
    differential_pressure:
      name: "Differential Pressure"
    temperature:
      name: "Temperature"
    update_interval: 60s
```

## Hardware Connections

- VCC: 3.3V
- GND: Ground
- SDA: I2C Data (default GPIO21 on ESP32)
- SCL: I2C Clock (default GPIO22 on ESP32)

Use pull-up resistors (2.5k-10k ohms) on SDA and SCL lines.

## Supported Sensors

- SDP600 series differential pressure sensors
- Default I2C address: 0x25