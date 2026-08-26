//bibliotecas
#include <Arduino.h>
#include <Adafruit_AHRS.h>
#include <SPI.h>
#include <math.h>

#ifndef MPU_H
#define MPU_H

//pinos mpu (p/ ESP32)
#define PINO_SCK   18
#define PINO_MISO  19
#define PINO_MOSI  23
#define CS_MPU     5

//endereco mpu9250
#define MPU9250_ADDRESS 0x68
#define MPU_ID          0x70
#define MODELO_MPU      0x75

//controle mpu9250
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C
#define SMPLRT_DIV    0x19
#define FILTER_CONFIG 0x1A

//acelerometro mpu9250
#define ACCEL_CONFIG  0x1C
#define ACCEL_CONFIG2 0x1D

//giroscopio mpu9250
#define GYRO_CONFIG   0x1B
#define GYRO_CONFIG2  0x1D

//constantes do filtro
const float GAMMA = 0.4f;    // filtro exponencial para aceleração linear
const float TETHA = 0.6f;    // limiar para detecção de movimento 

class MPU
{
public:
    MPU();

    bool inicializar();
    void lerMPU();
    void calibrarMPU();
    void MPUcalculos();

    void rotacionarPorQuaternario(
        float ax, float ay, float az,
        float q0, float q1, float q2, float q3,
        float &world_x, float &world_y, float &world_z);

    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t temp;

    float media_gyro_x = 0.0f, media_gyro_y = 0.0f, media_gyro_z = 0.0f;
    float media_acc_x = 0.0f, media_acc_y = 0.0f, media_acc_z = 0.0f;

    float medio_world_x = 0.0f, medio_world_y = 0.0f, medio_world_z = 0.0f;

    float convertido_temp = 0.0f;
    float convertido_acc_x = 0.0f, convertido_acc_y = 0.0f, convertido_acc_z = 0.0f;
    float convertido_gyro_x = 0.0f, convertido_gyro_y = 0.0f, convertido_gyro_z = 0.0f;

    float acc_world_x = 0.0f, acc_world_y = 0.0f, acc_world_z = 0.0f;
    float linear_x = 0.0f, linear_y = 0.0f, linear_z = 0.0f;

    float angulo_x = 0.0f;   // roll
    float angulo_y = 0.0f;   // pitch
    float angulo_z = 0.0f;   // yaw

    float vel_x = 0.0f, vel_y = 0.0f, vel_z = 0.0f;
    float pos_x = 0.0f, pos_y = 0.0f, pos_z = 0.0f;

    float mag_xyz = 0.0f;

    bool calibrado = false;
    bool movimento = false;

private:
    uint8_t lerRegistrador(uint8_t endereco);
    void escreverRegistrador(uint8_t endereco, uint8_t valor);

    //registradores internos
    byte ACC_X_H, ACC_X_L;
    byte ACC_Y_H, ACC_Y_L;
    byte ACC_Z_H, ACC_Z_L;

    byte TEMP_H, TEMP_L;

    byte GYRO_X_H, GYRO_X_L;
    byte GYRO_Y_H, GYRO_Y_L;
    byte GYRO_Z_H, GYRO_Z_L;

    Adafruit_Madgwick filter;
};

#endif
