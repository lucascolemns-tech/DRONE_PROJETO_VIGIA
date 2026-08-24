//bibliotecas
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

//modularização do código 
#ifndef MPU9250_H    
#define MPU9250_H    

//pinos mpu (p/ ESP32)
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define CS_MPU     5

//endereco mpu9250
#define MPU9250_ADDRESS 0x68
#define MPU_ID 0x70 // MPU_ID = 0x71
#define WHO_AM_I_MPU 0x75

//controle mpu9250
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define FILTER_CONFIG 0x1A
#define INT_PIN_CFG 0x37
#define SMPLRT_DIV    0x19

//acelerometro mpu9250
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D

//giroscopio mpu9250
#define GYRO_CONFIG 0x1B
#define GYRO_CONFIG2 0x1D

//constantes
const double pi = 3.1415926535;
const float GAMMA = 0.9999f; //não corresponde a um filtro real, solução pra falta de um magnetometro
const float ALPHA = 0.9995; //filtro complementar
const float acc_alpha = 0.3; //filtro passas-baixas, reduz as perdas por altas frequência do SPI
const float limiar_movimento = 0.1; //detecção estado (parado ou movimento)

class MPU9250 
{
public:

    MPU9250();

    bool inicializar();
    void lerMagnetometro();
    void lerAcelerometroGiroscopio();
    void calibrarAcelerometroGiroscopio();
    void calibrarGravidade();
    void MPUcalculos();
    void MPUresetar();
    void Sistemacoordenadas(
      float ax, float ay, float az, 
      float roll, float pitch, float yaw, 
      float &world_x, float &world_y, float &world_z);

    //dados construidos dos registradores
    int16_t ACC_X;
    int16_t ACC_Y;
    int16_t ACC_Z;

    int16_t GYRO_X;
    int16_t GYRO_Y;
    int16_t GYRO_Z;
    
    //dados calibração publicos
    long amostra_x =  0; //correspondente a amostra do giroscopio
    long amostra_y =  0; //correspondente a amostra do giroscopio
    long amostra_z =  0; //correspondente a amostra do giroscopio
    
    long amostra_x2 = 0; //correspondente a amostra do acelerometro
    long amostra_y2 = 0; //correspondente a amostra do acelerometro
    long amostra_z2 = 0; //correspondente a amostra do acelerometro
          
    float gyro_bias_x = 0; //têndencia dos eixos do giroscopio 
    float gyro_bias_y = 0;
    float gyro_bias_z = 0;

    float acc_bias_x = 0;
    float acc_bias_y = 0;
    float acc_bias_z = 0;

    float world_bias_x = 0;
    float world_bias_y = 0;
    float world_bias_z = 0;

    float somaX = 0; //correspondente a soma das amostras do sistema de coordenadas
    float somaY = 0; //correspondente a soma das amostras do sistema de coordenadas
    float somaZ = 0; //correspondente a soma das amostras do sistema de coordenadas

    //calculados
    float Converted_ACC_X; 
    float Converted_ACC_Y;
    float Converted_ACC_Z;

    float Converted_TEMP;

    float Converted_GYRO_X;
    float Converted_GYRO_Y;
    float Converted_GYRO_Z; 

    float ACC_X_World;
    float ACC_Y_World;
    float ACC_Z_World;

    float ACClinear_X_World;
    float ACClinear_Y_World;
    float ACClinear_Z_World;

    float acc_roll_deg  = 0.0;
    float acc_pitch_deg = 0.0;

    float roll_rad  = 0.0;
    float pitch_rad = 0.0;
    float yaw_rad   = 0.0;

    float angle_x = 0.0;
    float angle_y = 0.0;
    float angle_z = 0.0;

    float angle_x_rad = 0.0;
    float angle_y_rad = 0.0;
    float angle_z_rad = 0.0;

    float roll  = 0.0;
    float pitch = 0.0;
    float yaw   = 0.0;

    float vel_x = 0.0;
    float vel_y = 0.0;
    float vel_z = 0.0;

    float pos_x = 0.0;
    float pos_y = 0.0;
    float pos_z = 0.0;

    float mag_XYZ = 0.0;

    bool calibrado = false;
    bool movimento = false;

private:

  uint8_t lerRegistrador(uint8_t endereco);
  void escreverRegistrador(uint8_t endereco, uint8_t valor);
  uint8_t lerRegistradorI2C(uint8_t endereco, uint8_t WHO_AM_I);

  //dados obtidos dos registradores mpu 
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

  int16_t TEMP;
};

#endif  
