#ifndef INA219_SENSOR_H
#define INA219_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

class INA219_Sensor
{
public:
    INA219_Sensor(uint8_t addr = 0x40); //endereço I2C
    bool init();
    bool ler();
    bool leituraValida() const { return leitura_valida; }
    float getTensao() const { return tensao; }
    float getCorrente() const { return corrente; }
    float getPotencia() const { return potencia; }

private:
    Adafruit_INA219 ina;
    uint8_t _addr;
    float tensao = 0.0f;
    float corrente = 0.0f;
    float potencia = 0.0f;
    bool leitura_valida = false;
};

#endif