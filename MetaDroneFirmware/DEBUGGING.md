# Sensor and I2C startup debugger

Use the debug target to get sensor-by-sensor initialization results and a full I2C ACK scan at startup. The normal `blackpill_f411ce` target remains without debug prints.

## Build and connect

From `MetaDroneFirmware` run:

```powershell
pio run -e blackpill_f411ce_debug
pio run -e blackpill_f411ce_debug -t upload
pio device monitor -b 115200
```

The upload uses the configured ST-Link. Connect a 3.3 V USB-to-UART adapter to the STM32 debug port: adapter RX to PB10 (TX), common GND, and optionally adapter TX to PB11 (RX). Do not connect the adapter's VCC. PB10/PB11 keep debug separate from STM32-to-ESP32 UART on PA9/PA10 and ESC outputs on PA0..PA3.

## Expected I2C scan

The scan is on PB7/SDA and PB6/SCL and prints 7-bit addresses that ACK. Expected devices are BMP388 at 0x77 (0x76 is also reported as a possible address), INA219 at 0x40, VL53L0X at 0x29, and HMC5883-compatible magnetometer at 0x1E if fitted. The VL53L0X driver parameter 0x52 is its 8-bit address form; use 0x29 when interpreting the scan. MPU6500 is SPI on PA4..PA7, so its detection is reported from WHO_AM_I register 0x75 (expected value 0x70), not in the I2C scan.

For each sensor, distinguish an I2C ACK from successful initialization and a valid reading: the boot log reports the driver result and the scan separately. A listed address means a device ACKed on the live bus; it does not prove that it is the expected chip. No board is attached to this source-only review, so physical presence must be confirmed from the monitor output after flashing.

The recurring CSV line reports angles, fused altitude/velocity, BMP388 pressure in Pa, temperature, age of the latest valid barometer sample in ms, barometer readiness, battery voltage, range, and flight state. A barometer sample older than 100 ms marks the barometer not ready; brief data-ready gaps no longer count as a hard failure.
