#include "BE252Q.h"

TinyGPSPlus gps;

void BE252Q::escreverRegistradorI2C(uint8_t registrador, uint8_t valor)
{
    Wire.beginTransmission(QMC5883_ADDRESS);

    Wire.write(registrador);
    Wire.write(valor);

    Wire.endTransmission();
}

void BE252Q::BUSSOLA_init()
{
    Wire.begin();

    escreverRegistradorI2C(QMC5883_CONTROL_REG, 0x1D);
}

bool BE252Q::BUSSOLA_BE252Q()
{
    Wire.beginTransmission(QMC5883_ADDRESS);

    Wire.write(0x00);

    Wire.endTransmission(false);

    if (Wire.requestFrom(QMC5883_ADDRESS, 6) != 6)
        return false;

        X_lsb = Wire.read();
        X_msb = Wire.read();

        Y_lsb = Wire.read();
        Y_msb = Wire.read();

        Z_lsb = Wire.read();
        Z_msb = Wire.read();

    X = (int16_t)((X_msb << 8) | X_lsb);
    Y = (int16_t)((Y_msb << 8) | Y_lsb);
    Z = (int16_t)((Z_msb << 8) | Z_lsb);

    return true;
}

void BE252Q::calibragem()
{
    minX = minY = minZ = INT16_MAX;
    maxX = maxY = maxZ = INT16_MIN;

    unsigned long inicio = millis();

    while (millis() - inicio < 30000UL)
    {
        if (BUSSOLA_BE252Q())
        {
            if (X < minX) minX = X;
            if (X > maxX) maxX = X;
            if (Y < minY) minY = Y;
            if (Y > maxY) maxY = Y;
            if (Z < minZ) minZ = Z;
            if (Z > maxZ) maxZ = Z;
        }

        delay(20);
    }

    
    raioX = (maxX - minX) / 2.0f;
    raioY = (maxY - minY) / 2.0f;
    raioZ = (maxZ - minZ) / 2.0f;

    offsetX = (maxX + minX) / 2.0f;
    offsetY = (maxY + minY) / 2.0f;
    offsetZ = (maxZ + minZ) / 2.0f;

    medio = (raioX + raioY + raioZ) / 3.0f;

    //evitar divisão por 0
    if (raioX <= 0 || raioY <= 0 || raioZ <= 0)
        return;

    scaleX = medio / raioX;
    scaleY = medio / raioY;
    scaleZ = medio / raioZ;
}

float BE252Q::direcaoGraus()
{
    if(!BUSSOLA_BE252Q())
        return -1;

    Xcalibrado = (X - offsetX) * scaleX;
    Ycalibrado = (Y - offsetY) * scaleY;

    rumo = atan2(Ycalibrado, Xcalibrado) * 180.0f / pi;

    if (rumo < 0.0f)
        rumo += 360.0f;

    return rumo;
}

void BE252Q::GPS_BE252Q()
{
    while (Serial.available())
        gps.encode(Serial.read());

    if (gps.location.isUpdated() && gps.location.isValid())
    {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
    }

    if (gps.speed.isUpdated() && gps.speed.isValid())
        velocidade = gps.speed.kmph();
}


