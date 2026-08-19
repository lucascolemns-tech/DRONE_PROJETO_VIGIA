//datasheet: https://www.alldatasheet.com/datasheet-pdf/pdf/1132035/TDK/MPU-9250.html

//bibliotecas
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

//modularização do código 
#ifndef MPU9250_H    
#define MPU9250_H    

//pinos mpu9250
#define CS_MPU 9

//endereco mpu9250
#define MPU9250_ADDRESS 0x68
#define MPU_ID 0x71
#define WHO_AM_I_MPU 0x75

//controle mpu9250
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define CONFIG 0x1A
#define INT_PIN_CFG 0x37

//acelerometro mpu9250
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D

//giroscopio mpu9250
#define GYRO_CONFIG 0x1B
#define GYRO_CONFIG2 0x1D

//endereço magnetrometro
#define AK8963_ADDRESS 0x0C
#define AK8963_ID 0x48
#define WHO_AM_I_AK89630 0x00

//controle magnetometro
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL2 0x0B

//registradores magnetometro
#define MAG_MX 0x03
#define MAG_ST1 0x02
#define MAG_ST2 0x09

class MPU9250 
{
public:
    MPU9250();

    bool inicializar();
    void lerMagnetometro();
    void lerAcelerometroGiroscopio();
    void MPUcalculos();
    void Sistemacoordenadas(
        float ax, float ay, float az, 
        float roll, float pitch, float yaw, 
        float &world_x, float &world_y, float &world_z);
    
private:
    uint8_t lerRegistrador(uint8_t endereco);
    void escreverRegistrador(uint8_t endereco, uint8_t valor);
    uint8_t lerRegistradorI2C(uint8_t endereco, uint8_t WHO_AM_I);

    //dados mpu9250
        byte ACC_X_H;
        byte ACC_X_L;

        byte ACC_Y_H;
        byte ACC_Y_L;

        byte ACC_Z_H;
        byte ACC_Z_L;

        byte TEMP_H;
        byte TEMP_L;

        byte GYRO_X_H;
        byte GYRO_X_L;

        byte GYRO_Y_H;
        byte GYRO_Y_L;

        byte GYRO_Z_H;
        byte GYRO_Z_L;
   
        int16_t ACC_X;
        int16_t ACC_Y;
        int16_t ACC_Z;

        int16_t TEMP;

        int16_t GYRO_X;
        int16_t GYRO_Y;
        int16_t GYRO_Z;

        int16_t mx_L;
        int16_t mx_H;
        int16_t my_L;
        int16_t my_H;
        int16_t mz_L;
        int16_t mz_H;

        int16_t mx; 
        int16_t my;
        int16_t mz;

    //calculados
        float angle_x = 0.0;
        float angle_y = 0.0;
        float angle_z = 0.0;

        float roll = 0.0;
        float pitch = 0.0;
        float yaw = 0.0;
        
        float vel_x = 0.0;
        float vel_y = 0.0;
        float vel_z = 0.0;

        float pos_x = 0.0;
        float pos_y = 0.0;
        float pos_z = 0.0;

     
    };

#endif  