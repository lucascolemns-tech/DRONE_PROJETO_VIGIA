#include "MPU.h"

MPU::MPU()
{
  angulo_x = 0.0f;
  angulo_y = 0.0f;
  angulo_z = 0.0f;

  vel_x = 0.0f;
  vel_y = 0.0f;
  vel_z = 0.0f;

  pos_x = 0.0f;
  pos_y = 0.0f;
  pos_z = 0.0f;
}

uint8_t MPU::lerRegistrador(uint8_t endereco)
{
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_MPU, LOW);
  SPI.transfer(endereco | 0x80);
  uint8_t valor = SPI.transfer(0x00);
  digitalWrite(CS_MPU, HIGH);
  SPI.endTransaction();
  return valor;
}

void MPU::escreverRegistrador(uint8_t endereco, uint8_t valor)
{
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_MPU, LOW);
  SPI.transfer(endereco & 0x7F);
  SPI.transfer(valor);
  digitalWrite(CS_MPU, HIGH);
  SPI.endTransaction();
}

void MPU::lerMPU()
{
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_MPU, LOW);
  SPI.transfer(0x3B | 0x80);

  for (uint8_t endereco = 0x3B; endereco <= 0x48; endereco++)
  {
    uint8_t valor = SPI.transfer(0x00);
    switch (endereco)
    {
      case 0x3B: ACC_X_H = valor; break;
      case 0x3C: ACC_X_L = valor; break;
      case 0x3D: ACC_Y_H = valor; break;
      case 0x3E: ACC_Y_L = valor; break;
      case 0x3F: ACC_Z_H = valor; break;
      case 0x40: ACC_Z_L = valor; break;
      case 0x41: TEMP_H = valor; break;
      case 0x42: TEMP_L = valor; break;
      case 0x43: GYRO_X_H = valor; break;
      case 0x44: GYRO_X_L = valor; break;
      case 0x45: GYRO_Y_H = valor; break;
      case 0x46: GYRO_Y_L = valor; break;
      case 0x47: GYRO_Z_H = valor; break;
      case 0x48: GYRO_Z_L = valor; break;
    }
  }

  digitalWrite(CS_MPU, HIGH);
  SPI.endTransaction();

  // Montar valores de 16 bits
  acc_x = ((int16_t)ACC_X_H << 8) | ACC_X_L;
  acc_y = ((int16_t)ACC_Y_H << 8) | ACC_Y_L;
  acc_z = ((int16_t)ACC_Z_H << 8) | ACC_Z_L;

  temp = ((int16_t)TEMP_H << 8) | TEMP_L;

  gyro_x = ((int16_t)GYRO_X_H << 8) | GYRO_X_L;
  gyro_y = ((int16_t)GYRO_Y_H << 8) | GYRO_Y_L;
  gyro_z = ((int16_t)GYRO_Z_H << 8) | GYRO_Z_L;
}

bool MPU::inicializar()
{
  pinMode(CS_MPU, OUTPUT);
  digitalWrite(CS_MPU, HIGH);

  SPI.begin(PINO_SCK, PINO_MISO, PINO_MOSI, CS_MPU);

  if (lerRegistrador(MODELO_MPU) != MPU_ID)
    return false;

  escreverRegistrador(PWR_MGMT_1, 0x01);
  delay(100);
  escreverRegistrador(PWR_MGMT_2, 0x00);
  escreverRegistrador(ACCEL_CONFIG, 0x00);
  escreverRegistrador(ACCEL_CONFIG2, 0x03);
  escreverRegistrador(GYRO_CONFIG, 0x00);
  escreverRegistrador(FILTER_CONFIG, 0x03);
  escreverRegistrador(SMPLRT_DIV, 0x00);

  filter.begin(100.0f);

  return true;
}

/*
Os quarténios são a solução matématica para os problemas envolvendo os 
angulos de Euler (pitch, roll e yaw), é mais preciso, ele funciona com os
eixos XYZ como números imagínarios e uma parte real chamada de w, de forma
simples cada eixo tem sua propria rotação (pela propriedade dos números imaginários)
*/
void MPU::rotacionarPorQuaternario(
  float ax, float ay, float az,
  float q0, float q1, float q2, float q3,
  float &world_x, float &world_y, float &world_z)
{
  // Matriz de rotação a partir do quatérnio
  float m00 = 1.0f - 2.0f * (q2*q2 + q3*q3);
  float m01 = 2.0f * (q1*q2 - q0*q3);
  float m02 = 2.0f * (q1*q3 + q0*q2);

  float m10 = 2.0f * (q1*q2 + q0*q3);
  float m11 = 1.0f - 2.0f * (q1*q1 + q3*q3);
  float m12 = 2.0f * (q2*q3 - q0*q1);

  float m20 = 2.0f * (q1*q3 - q0*q2);
  float m21 = 2.0f * (q2*q3 + q0*q1);
  float m22 = 1.0f - 2.0f * (q1*q1 + q2*q2);

  world_x = m00 * ax + m01 * ay + m02 * az;
  world_y = m10 * ax + m11 * ay + m12 * az;
  world_z = m20 * ax + m21 * ay + m22 * az;
}

void MPU::MPUcalculos()
{
  //gerenciamento do tempo, a váriavel tempo percorre em micro-segundos
  static unsigned long tempo_antes = 0;
  unsigned long tempo_agora = micros();

  if (tempo_antes == 0)
  {
      tempo_antes = tempo_agora;
      return;
  }

  float dt = (tempo_agora - tempo_antes) * 0.000001f;
  tempo_antes = tempo_agora;

  if (dt <= 0.0f || dt > 0.1f)
      return;

  //conversão dos valores obtidos dos registradores + relação com gravidade terrestre
  float ax_g = (float)acc_x / 16384.0f; 
  float ay_g = (float)acc_y / 16384.0f;
  float az_g = (float)acc_z / 16384.0f;

  float acc_bruta_x = ax_g * 9.80665f; 
  float acc_bruta_y = ay_g * 9.80665f;
  float acc_bruta_z = az_g * 9.80665f;

  //dados da calibragem adicionados aos valores obtidos + relação com gravidade terrestre  
  convertido_acc_x = ((float)acc_x - media_acc_x) * 9.80665f / 16384.0f;
  convertido_acc_y = ((float)acc_y - media_acc_y) * 9.80665f / 16384.0f;
  convertido_acc_z = ((float)acc_z - media_acc_z) * 9.80665f / 16384.0f;

  float convertido_gyro_x = ((float)gyro_x - media_gyro_x) / 131.0f;
  float convertido_gyro_y = ((float)gyro_y - media_gyro_y) / 131.0f;
  float convertido_gyro_z = ((float)gyro_z - media_gyro_z) / 131.0f;

  //chamada do filtro da AHRS
  filter.updateIMU(convertido_gyro_x, convertido_gyro_y, convertido_gyro_z,
                    ax_g, ay_g, az_g, dt);

  angulo_x = filter.getRoll();
  angulo_y = filter.getPitch();
  angulo_z = filter.getYaw();

  //obtém os valores em quartenários  imaginários: x, y, z e reais: w
  float q0, q1, q2, q3;
  filter.getQuaternion(&q0, &q1, &q2, &q3);

  rotacionarPorQuaternario(acc_bruta_x, acc_bruta_y, acc_bruta_z,
                          q0, q1, q2, q3,
                          acc_world_x, acc_world_y, acc_world_z);

  linear_x = acc_world_x - medio_world_x;
  linear_y = acc_world_y - medio_world_y;
  linear_z = acc_world_z - medio_world_z;

  //filtro exponencial, responsável pela suavização dos picos de valores 
  static bool filtro_inicializado = false;

  if (!filtro_inicializado)
  {
    filtro_x = linear_x;
    filtro_y = linear_y;
    filtro_z = linear_z;
    filtro_inicializado = true;
  }

  filtro_x = GAMMA * linear_x + (1.0f - GAMMA) * filtro_x;
  filtro_y = GAMMA * linear_y + (1.0f - GAMMA) * filtro_y;
  filtro_z = GAMMA * linear_z + (1.0f - GAMMA) * filtro_z;

  mag_xyz = sqrtf(
    filtro_x*filtro_x 
  + filtro_y*filtro_y 
  + filtro_z*filtro_z
  );

  movimento = (mag_xyz > TETHA);

  if (calibrado && movimento)
  {
    vel_x += filtro_x * dt;
    vel_y += filtro_y * dt;
    vel_z += filtro_z * dt;

    pos_x += vel_x * dt;
    pos_y += vel_y * dt;
    pos_z += vel_z * dt;
  }
  else if (calibrado)
  {
    vel_x = 0.0f;
    vel_y = 0.0f;
    vel_z = 0.0f;
    pos_x = 0.0f;
    pos_y = 0.0f;
    pos_z = 0.0f;
  }
}

void MPU::calibrarMPU()
{
  const int amostras = 1000;
  calibrado = false;

  Serial.println("Calibrando...");
  Serial.println("NAO MOVA O MPU!");
  delay(3000);

  /*
  a calibração é feita a partir de uma constante conferida por meio da média de diversos valores,
  quanto maior for o número de amostras maior será o tempo de calibragem, também será mais preciso o sistema
  */

  //parte do giroscópio e aceleromêtro
  float soma_gx = 0.0f, soma_gy = 0.0f, soma_gz = 0.0f; 
  float soma_ax = 0.0f, soma_ay = 0.0f, soma_az = 0.0f; 

  for (int i = 0; i < amostras; i++)
  {
    lerMPU();

    soma_gx += gyro_x;
    soma_gy += gyro_y;
    soma_gz += gyro_z;
    soma_ax += acc_x;
    soma_ay += acc_y;
    soma_az += acc_z;

    delay(2);
  }

  media_gyro_x = soma_gx / amostras;
  media_gyro_y = soma_gy / amostras;
  media_gyro_z = soma_gz / amostras;

  media_acc_x = soma_ax / amostras;
  media_acc_y = soma_ay / amostras;
  media_acc_z = soma_az / amostras;

  Serial.println("Calibrando vetor gravidade...");
  delay(1000);

  //parte do sistema do eixo terrestre
  const int amostras_world = 200;
  float soma_wx = 0.0f, soma_wy = 0.0f, soma_wz = 0.0f;

  for (int i = 0; i < amostras_world; i++)
  {
    lerMPU();

    float ax_g = (float)acc_x / 16384.0f;
    float ay_g = (float)acc_y / 16384.0f;
    float az_g = (float)acc_z / 16384.0f;

    float gx_dps = ((float)gyro_x - media_gyro_x) / 131.0f;
    float gy_dps = ((float)gyro_y - media_gyro_y) / 131.0f;
    float gz_dps = ((float)gyro_z - media_gyro_z) / 131.0f;

    float dt = 0.01f;
    filter.updateIMU(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt);

    float q0, q1, q2, q3;
    filter.getQuaternion(&q0, &q1, &q2, &q3);

    float ax_m = ax_g * 9.80665f;
    float ay_m = ay_g * 9.80665f;
    float az_m = az_g * 9.80665f;

    float wx, wy, wz;

    rotacionarPorQuaternario(ax_m, ay_m, az_m, q0, q1, q2, q3, wx, wy, wz);

    soma_wx += wx;
    soma_wy += wy;
    soma_wz += wz;

    delay(10);
  }

  medio_world_x = soma_wx / amostras_world;
  medio_world_y = soma_wy / amostras_world;
  medio_world_z = soma_wz / amostras_world;

  calibrado = true;

  Serial.println("==============================");
  Serial.println(" CALIBRACAO CONCLUIDA");
  Serial.println("==============================");
}