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

    void VL53L0X_init()
    {
        Wire.begin();
        Wire.setClock(400000);
        sensor.begin();
        sensor.VL53L0X_On();
        sensor.InitSensor(0x52);
        sensor.StartMeasurementSimplified(range_continuous_polling, NULL);
    }

    uint16_t VL53L0X_read()
    {
        sensor.GetMeasurementSimplified(range_continuous_polling, &data);
        return data.RangeMilliMeter;
    }

    bool VL53L0X_detect()
    {
        uint16_t dist = VL53L0X_read();
        
        if (data.RangeStatus == 0) {
            if (dist < CONDICAO_PARADO)
                parado = true;
            else if (dist > CONDICAO_VOANDO)
                parado = false;
        }
        return parado;
    }

private:
    VL53L0X sensor;
    VL53L0X_RangingMeasurementData_t data;
    bool parado = false;
};

#endif
