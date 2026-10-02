#ifndef DIST_SENSOR_H
#define DIST_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "vl53l0x_class.h"

#define XSHUT_PIN PB8

#define CONDICAO_PARADO 300
#define CONDICAO_VOANDO 500

class DistSensor
{
public:
    DistSensor() : sensor(&Wire, XSHUT_PIN) {}

    bool VL53L0X_init()
    {
        // Wire já foi configurado em i2c_init(); não reinicialize o barramento aqui.
        sensor.begin();
        sensor.VL53L0X_On();
        _initStatus = sensor.InitSensor(0x52); // endereço 8-bit do driver; scan I2C mostra 0x29
        if (_initStatus != 0)
            return false;
        _initStatus = sensor.StartMeasurementSimplified(range_continuous_polling, NULL);
        _initialized = (_initStatus == 0);
        return _initialized;
    }

    int VL53L0X_status() const { return _initStatus; }

    bool VL53L0X_read(uint16_t &distance_mm)
    {
        if (!_initialized || sensor.GetMeasurementSimplified(range_continuous_polling, &data) != 0)
            return false;
        if (data.RangeStatus == 0)
            distance_mm = data.RangeMilliMeter;
        return true;
    }

    bool VL53L0X_detect()
    {
        uint16_t dist = 0;
        if (!VL53L0X_read(dist) || data.RangeStatus != 0)
            return parado;

        if (dist < CONDICAO_PARADO)
            parado = true;
        else if (dist > CONDICAO_VOANDO)
            parado = false;
        return parado;
    }

private:
    VL53L0X sensor;
    VL53L0X_RangingMeasurementData_t data;
    bool parado = false;
    bool _initialized = false;
    int _initStatus = -1;
};

#endif
