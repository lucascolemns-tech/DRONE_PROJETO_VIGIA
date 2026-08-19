//datasheet: https://www.alldatasheet.com/datasheet-pdf/pdf/1132069/BOSCH/BMP280.html

//bibliotecas
#include <Arduino.h>
#include <SPI.h>

//modularização do código 
#ifndef BMP280_H
#define BMP280_H

//pinos bmp280
    #define CS_BMP280 10

//registradores bmp280
    #define TEMP_XLSB  0xFC
    #define TEMP_LSB   0xFB
    #define TEMP_MSB   0xFA

    #define PRESS_XLSB 0xF9
    #define PRESS_LSB  0xF8
    #define PRESS_MSB  0xF7

    #define CONFIG     0xF5
    #define CTRL_MEAS  0xF4
    #define STATUS     0xF3

    #define RESET      0xE0
    #define ID_REG     0xD0

    #define CALIB_START 0x88
    #define CALIB_END   0xA1    

class BMP280
{
public:
    BMP280();

    bool inicializar();

    float lerTemperatura();
    float lerAltitude();
    float lerPressao();

    void calibragem();

    uint32_t lerTemperaturaBRUTA();
    uint32_t lerPressaoBRUTA();

private:
    uint8_t lerRegistrador(uint8_t endereco);
    void escreverRegistrador(uint8_t endereco, uint8_t valor);

    //registradores de calibração de temperatura 
        uint16_t dig_T1; //LSB = 0x88 e MSB = 0x89 
        int16_t  dig_T2; //LSB = 0x8A e MSB = 0x8B 
        int16_t  dig_T3; //LSB = 0x8C e MSB = 0x8D 
        
    //registradores de calibração de pressão
        uint16_t dig_P1; //LSB = 0x8E e MSB = 0x8F
        int16_t  dig_P2; //LSB = 0x90 e MSB = 0x91
        int16_t  dig_P3; //LSB = 0x92 e MSB = 0x93
        int16_t  dig_P4; //LSB = 0x94 e MSB = 0x95
        int16_t  dig_P5; //LSB = 0x96 e MSB = 0x97
        int16_t  dig_P6; //LSB = 0x98 e MSB = 0x99
        int16_t  dig_P7; //LSB = 0x9A e MSB = 0x9B
        int16_t  dig_P8; //LSB = 0x9C e MSB = 0x9D
        int16_t  dig_P9; //LSB = 0x9E e MSB = 0x9F
 
     //compensação de temperatura de acordo com datasheet
        int32_t t_compensado;       
};

#endif