//bibliotecas
#include <Arduino.h>
#include <Adafruit_AHRS.h>
#include <SPI.h>
#include <math.h>

#ifndef MPU_H
#define MPU_H

//pinos mpu STM
#define PINO_SCK   PA5
#define PINO_MISO  PA6
#define PINO_MOSI  PA7
#define CS_MPU     PA4

//Identificação do MPU6500 via SPI (não há endereço I2C neste driver)
#define MPU_ID          0x70
#define MODELO_MPU      0x75

//controle mpu6050
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C
#define SMPLRT_DIV    0x19
#define FILTER_CONFIG 0x1A

//acelerometro mpu6050
#define ACCEL_CONFIG  0x1C
#define ACCEL_CONFIG2 0x1D

//giroscopio mpu6050
#define GYRO_CONFIG   0x1B
#define GYRO_CONFIG2  0x1D

//constantes do filtro
const float GAMMA = 0.4f;    // filtro exponencial para aceleração linear
const float TETHA = 0.8f;    // limiar para detecção de movimento 

class MPU6500
{
public:
  MPU6500();

  bool inicializar();
  bool lerMPU();
  bool calibrarMPU();
  void MPUcalculos(float mag_x, float mag_y, float mag_z, bool mag_valido);

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

  float filtro_x = 0.0f, filtro_y = 0.0f, filtro_z = 0.0f; 
  
  float mag_xyz = 0.0f;

  bool calibrado = false;
  bool movimento = false;

  bool movimento_obter() { return movimento; }

private:
  uint8_t lerRegistrador(uint8_t endereco);
  void escreverRegistrador(uint8_t endereco, uint8_t valor);

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
