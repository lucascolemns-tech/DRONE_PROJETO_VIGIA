# Firmware sensor and I2C diagnostics

## STM32 (PlatformIO default target)

Open the PlatformIO serial monitor at **115200 baud**. `STM32_DEBUG_UART` and USB CDC are already enabled for `blackpill_f411ce`. The STM32 startup log reports sensor initialization and the periodic log reports UART frame counts and sensor readiness. This firmware does not run an I2C address scan.

| Device | Firmware setting / expected 7-bit address | Interface |
|---|---:|---|
| BMP388 barometer | `0x77`, automatically retries `0x76` | I2C, SDA `PB7`, SCL `PB6` |
| VL53L0X range sensor | `0x29` | I2C; the STM32 library initializer takes `0x52` in its 8-bit convention |
| INA219 current sensor | `0x40` | I2C, optional and disabled by default |
| HMC5883L magnetometer | `0x1E` | I2C, optional and disabled by default |
| MPU6500 | no I2C address | SPI; `WHO_AM_I` register `0x75` should return `0x70` |

The BMP initialization checks its chip ID, calibration reads, and configuration readback. Check power, common ground, SDA/SCL wiring, pull-ups, and the selected address if an I2C sensor fails to initialize.

The periodic STM32 line reports MPU/BMP/range readiness and BMP sample age. BMP readings use the BMP388 data-ready flags (status bits 5 and 6); a transient poll with no new sample keeps the recent valid sample for up to 100 ms. Kalman receives a barometer update only when a fresh sample was acquired. If the BMP reports an error, the essential sensor gate keeps the aircraft in `DESLIGADO` until retry succeeds.

## ESP32 telemetry and TCP diagnostics

Flash `esp32doit-devkit-v1`, open its serial monitor at **115200 baud**, and check the startup Wi-Fi IP and TCP port. The interface in `Downloads/META_DRONE/INTERFACE_PYTHON` connects to `10.85.164.132:1244`.

The once-per-second ESP32 log reports UART bytes, headers, CRC-valid frames, CRC failures, rejected frames, the invalid field/value, telemetry freshness, and Wi-Fi/TCP transmit status. `RX15=` prints the last complete 15-float UART frame so the field order and values can be compared directly.

The 15 fields are: roll, pitch, yaw, altitude, vertical velocity, temperature, M1–M4, battery voltage, GPS altitude, sensor/system-ready flag, magnetometer status, and magnetometer calibration progress. `frame/s` above zero confirms complete CRC-valid STM32 frames are reaching the ESP32. `TCP=1` with `txB/s` above zero confirms the ESP32 is writing telemetry to the connected interface.

## Standalone ESP32 sketch (`Downloads/META_DRONE/DRONAO_ESP32`)

The sketch scans I2C during BMP startup on SDA GPIO 26 / SCL GPIO 27 at 100 kHz. Its BMP driver now checks both data-ready bits, transport results, calibration coefficients, compensated pressure/temperature ranges, and register readback. Serial output reports the last sample age; `STALE` means no successful sample for more than 500 ms.

## Barometric altitude reference

Both implementations now capture the first valid pressure sample at startup and report altitude relative to that launch pressure, so the grounded value starts near `0 m` at any local elevation. Weather and pressure drift still affect long flights; the current code does not fuse a GPS/barometer pressure reference.
