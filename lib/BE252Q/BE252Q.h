//datasheet: https://www.beitian.com/en/sys-pd/837.html

//bibliotecas
#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>

//modularização do código 
#define BE252Q_H
#ifdef BE252Q_H

//I2C 
#define QMC5883_ADDRESS 0x0D
#define QMC5883_CONTROL_REG 0x09

//constantes
const double pi = 3.1415926535;

class BE252Q
{
public:
    void GPS_BE252Q();
    void BUSSOLA_init();
    bool BUSSOLA_BE252Q();

    void  calibragem();
    float direcaoGraus();

    double getLatitude() const { return latitude; };
    double getLongitude() const { return longitude; };
    double getVelocidade() const { return velocidade; };

private:
    //I2C
    void escreverRegistradorI2C(uint8_t endereco, uint8_t valor);

    //dados BE252Q
    double latitude;
    double longitude;
    double velocidade;

    uint16_t X_lsb;
    uint16_t X_msb; 
    uint16_t Y_lsb;
    uint16_t Y_msb;
    uint16_t Z_lsb;
    uint16_t Z_msb;

    float X;
    float Z;
    float Y;

    float offsetX, offsetY, offsetZ;
    float scaleX = 1, scaleY = 1, scaleZ = 1;

    int16_t minX = INT16_MAX, minY = INT16_MAX, minZ = INT16_MAX;
    int16_t maxX = INT16_MIN, maxY = INT16_MIN, maxZ = INT16_MIN;   
    
    float raioX;
    float raioY;
    float raioZ;
    float medio;

    float Xcalibrado;
    float Ycalibrado;

    float rumo;
};

#endif
