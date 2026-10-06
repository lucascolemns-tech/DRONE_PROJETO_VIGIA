#ifndef DIST_SENSOR_H
#define DIST_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "vl53l0x_class.h" //bibliteca do sensor

#define XSHUT_PIN PB8 //serve como pino de reset do sensor, para que não haja conflitos no sistema I2C

#define CONDICAO_PARADO 300
#define CONDICAO_VOANDO 500

class DistSensor
{
public:
    DistSensor() : sensor(&Wire, XSHUT_PIN) {} //construtor, preciso colocar endereço e o pino de reset

    bool VL53L0X_init()
    {
        sensor.begin();
        sensor.VL53L0X_On();
        if (sensor.InitSensor(0x52) != VL53L0X_ERROR_NONE) //se não conseguir inicializar o sensor, retorna falso, váriavel de chegagem do endereço encontrado, "erro".
            return false;
        sensorPronto = sensor.StartMeasurementSimplified(range_continuous_polling, NULL) == VL53L0X_ERROR_NONE;
        return sensorPronto;
    }

    uint16_t VL53L0X_read()
    {
        if (!sensorPronto)
            return ultimaDistancia;

        uint8_t pronto = 0;
        if (sensor.GetMeasurementDataReady(&pronto) != VL53L0X_ERROR_NONE || !pronto) //ambos retornam valor booleano
            return ultimaDistancia;

        if (sensor.GetRangingMeasurementData(&data) != VL53L0X_ERROR_NONE)
            return ultimaDistancia;

        sensor.ClearInterruptMask(VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY); //incialização da interrupção do sensor, para que ele saiba que a leitura foi feita e possa fazer a próxima
        if (data.RangeStatus == 0)
            ultimaDistancia = data.RangeMilliMeter; 
        return ultimaDistancia;
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
    VL53L0X sensor; //construtor da bibliteca utilizada
    VL53L0X_RangingMeasurementData_t data = {}; //estrutura de dados para armazenar os valores lidos do sensor
    uint16_t ultimaDistancia = 0;
    bool parado = false;
    bool sensorPronto = false; //checagem sensor ok, chamado de ACK ou "acknowledge"
};

#endif
