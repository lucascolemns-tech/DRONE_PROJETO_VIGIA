//modularização do código
#ifndef BMP_H
#define BMP_H

//bibliotecas
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

//registradores
#define REG_CHIP_ID  0x00
#define REG_CONFIG   0x1F
#define REG_STATUS   0x03
#define REG_DATA     0x04
#define PWR_CTRL     0x1B
#define CALIB        0x31
#define OSR          0x1C
#define ODR          0x1D

//I2C
#define BMP_ID 0x50
#define SDA 26
#define SCL 27 

class BMP
{
public:
    
  BMP(uint8_t endereco = 0x76);

  bool inicializar();
  void lerBMP();

  float temperatura;
  float pressao;


private:

  uint8_t _endereco;

  uint8_t lerRegistrador(uint8_t reg);
  void escreverRegistrador(uint8_t reg, uint8_t valor);
  void lerRegistradores(uint8_t  reg, uint8_t* buffer, uint8_t  tamanho);
  void lerCalibracao();

  float compensarTemperatura(uint32_t temp_bruta);
  float compensarPressao(uint32_t pres_bruta);

  double t_lin;
  double par_t1, par_t2, par_t3;
  double par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11; 
};

#endif