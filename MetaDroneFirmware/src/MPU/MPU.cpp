#include <Arduino.h>
#include "MPU.h"

MPU6500::MPU6500() //construtor incial, coloca todos valores zerados pra evitar valores de "lixo"
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

uint8_t MPU6500::lerRegistrador(uint8_t endereco)
{
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3)); //msb first é o bit mais signficativo primeiro, modo3 borda de descida o ocorre mudança de estado
  digitalWrite(CS_MPU, LOW); //permite passagem para o barramento SPI do MPU 
  SPI.transfer(endereco | 0x80); //o primeiro bit mais significativo será sempre 1, no MPU, essa operação define se o registrador está em modo de leitura 
  uint8_t valor = SPI.transfer(0x00); //depois começa a desenvolver as leituras do registrador
  digitalWrite(CS_MPU, HIGH); //fecha
  SPI.endTransaction(); //fecha bus SPI
  return valor;
}

void MPU6500::escreverRegistrador(uint8_t endereco, uint8_t valor)
{
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_MPU, LOW); 
  SPI.transfer(endereco & 0x7F);//modo escrita o endereço do registrador recebe um 0 no bit mais significativo do endereço
  SPI.transfer(valor);
  digitalWrite(CS_MPU, HIGH);
  SPI.endTransaction(); //fecha bus SPI
}

bool MPU6500::lerMPU()
{
  if (lerRegistrador(MODELO_MPU) != MPU_ID)
    return false; //retorna falso se a checagem for falsa

  bool todos_zero = true; //caso um desses casos for verdadeiro a leitura é incorreta
  bool todos_um = true;
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_MPU, LOW);
  SPI.transfer(0x3B | 0x80); //ativamos o modo de leitura a partir do primeiro registrador da sequência, vamos percorrer os endereços

  for (uint8_t endereco = 0x3B; endereco <= 0x48; endereco++) //0x3B primerio reg, 0x48 últim oreg, todos valores de 8 bits
  {
    uint8_t valor = SPI.transfer(0x00);
    todos_zero = todos_zero && valor == 0x00; 
    todos_um = todos_um && valor == 0xFF;
    switch (endereco)
    {
      case 0x3B: ACC_X_H = valor; break; //atritibui a leitura a cada 
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

  if (todos_zero || todos_um)
    return false;

  // Montar valores de 16 bits
  acc_x = ((int16_t)ACC_X_H << 8) | ACC_X_L;
  acc_y = ((int16_t)ACC_Y_H << 8) | ACC_Y_L;
  acc_z = ((int16_t)ACC_Z_H << 8) | ACC_Z_L;

  temp = ((int16_t)TEMP_H << 8) | TEMP_L;
  
  gyro_x = ((int16_t)GYRO_X_H << 8) | GYRO_X_L;
  gyro_y = ((int16_t)GYRO_Y_H << 8) | GYRO_Y_L;
  gyro_z = ((int16_t)GYRO_Z_H << 8) | GYRO_Z_L;
  return true;
}

bool MPU6500::inicializar()
{
  pinMode(CS_MPU, OUTPUT);
  digitalWrite(CS_MPU, HIGH);

  SPI.begin();

  uint8_t who_am_i = lerRegistrador(MODELO_MPU); // o who_am_i indica o endereço do chip do mpu
  #if defined(STM32_DEBUG_UART)
    Serial.print("MPU6500 SPI WHO_AM_I=0x");
    if (who_am_i < 0x10) Serial.print("0"); //serve para evitar confusão, se o valor é menor em hex fica por ex 0x9 ao invés de 0x09
    Serial.println(who_am_i, HEX);
  #endif
    
  if (who_am_i != MPU_ID)
      return false;
  //escreve as configurações 
  escreverRegistrador(PWR_MGMT_1, 0x01);
  escreverRegistrador(PWR_MGMT_2, 0x00);
  escreverRegistrador(ACCEL_CONFIG, 0x00);
  escreverRegistrador(ACCEL_CONFIG2, 0x03);
  escreverRegistrador(GYRO_CONFIG, 0x00);
  escreverRegistrador(FILTER_CONFIG, 0x03);
  escreverRegistrador(SMPLRT_DIV, 0x00);
  filter.begin(100.0f); //inicialização filtro AHRS
  return true;
}

/*
Os quaterniões são a solução matemática para os problemas envolvendo os 
angulos de Euler (pitch, roll e yaw), é mais preciso, ele funciona com os
eixos XYZ como números imaginários e uma parte real chamada de w, de forma
simples cada eixo tem sua propria rotação (pela propriedade dos números imaginários)
*/
void MPU6500::rotacionarPorQuaternario(
  float ax, float ay, float az,
  float q0, float q1, float q2, float q3,
  float &world_x, float &world_y, float &world_z)
{
  //desenvolvimento de um conjunto de matrizes para calculo dos quatérnios
  float m00 = 1.0f - 2.0f * (q2*q2 + q3*q3); //coluna 1
  float m01 = 2.0f * (q1*q2 - q0*q3);
  float m02 = 2.0f * (q1*q3 + q0*q2);
  float m10 = 2.0f * (q1*q2 + q0*q3); //coluna 2
  float m11 = 1.0f - 2.0f * (q1*q1 + q3*q3);
  float m12 = 2.0f * (q2*q3 - q0*q1);
  float m20 = 2.0f * (q1*q3 - q0*q2); //coluna 3
  float m21 = 2.0f * (q2*q3 + q0*q1);
  float m22 = 1.0f - 2.0f * (q1*q1 + q2*q2);
  //rotação da matriz dos quatérnios
  world_x = m00 * ax + m01 * ay + m02 * az;
  world_y = m10 * ax + m11 * ay + m12 * az;
  world_z = m20 * ax + m21 * ay + m22 * az;
}

void MPU6500::MPUcalculos(float mag_x, float mag_y, float mag_z, bool mag_valido)
{
  //gerenciamento do tempo, a variável tempo percorre em micro-segundos
  static unsigned long tempo_antes = 0;
  unsigned long tempo_agora = micros();
  if (tempo_antes == 0) { tempo_antes = tempo_agora; return; }
  float dt = (tempo_agora - tempo_antes) * 0.000001f; //gerenciar os intervalos de tempo em microsegundos
  tempo_antes = tempo_agora;
  if (dt <= 0.0f || dt > 0.1f) { return; }

  //conversão dos valores obtidos dos registradores + relação com gravidade terrestre
  float ax_g = (float)acc_x / 16384.0f; 
  float ay_g = (float)acc_y / 16384.0f;
  float az_g = (float)acc_z / 16384.0f;
  //valores brutos
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

  //chamada do filtro da AHRS, e decide a configuração 9DOF (+ magnetometro) OU 6DOF sem magnetometro.
  if (mag_valido)
    filter.update(convertido_gyro_x, convertido_gyro_y, convertido_gyro_z,
                  ax_g, ay_g, az_g, mag_x, mag_y, mag_z, dt);
  else
    filter.updateIMU(convertido_gyro_x, convertido_gyro_y, convertido_gyro_z,
                     ax_g, ay_g, az_g, dt);

  angulo_x = filter.getRoll();
  angulo_y = filter.getPitch();
  angulo_z = filter.getYaw();

  //obtém os valores em quartenários  imaginários: x, y, z e reais: w
  float q0, q1, q2, q3;
  filter.getQuaternion(&q0, &q1, &q2, &q3); //salva nela mesma
  rotacionarPorQuaternario(acc_bruta_x, acc_bruta_y, acc_bruta_z,
                          q0, q1, q2, q3,
                          acc_world_x, acc_world_y, acc_world_z);

  linear_x = acc_world_x - medio_world_x;
  linear_y = acc_world_y - medio_world_y;
  linear_z = acc_world_z - medio_world_z;

  //filtro exponencial, responsável pela suavização dos picos de valores 
  static bool filtro_inicializado = false;
  //inicial não iniciado ele define o paramêtro inicial, que são os próprios valores calculados previamente
  if (!filtro_inicializado)
  {
    filtro_x = linear_x;
    filtro_y = linear_y;
    filtro_z = linear_z;
    filtro_inicializado = true;
  }
  //GAMMA é a constante que eu chamei o responsável pelo filtro exponencial
  filtro_x = GAMMA * linear_x + (1.0f - GAMMA) * filtro_x;
  filtro_y = GAMMA * linear_y + (1.0f - GAMMA) * filtro_y;
  filtro_z = GAMMA * linear_z + (1.0f - GAMMA) * filtro_z;

  mag_xyz = sqrtf(filtro_x*filtro_x + filtro_y*filtro_y + filtro_z*filtro_z); 
  movimento = (mag_xyz > TETHA); //THETA é a constante do movimento MÍNIMO para ser considerado parado

  if (calibrado && movimento)
  {
    vel_x += filtro_x * dt;
    vel_y += filtro_y * dt;
    vel_z += filtro_z * dt;
    pos_x += vel_x * dt;
    pos_y += vel_y * dt;
    pos_z += vel_z * dt;
  }
  else if (calibrado) //se não houver movimento zera
  {
    vel_x = 0.0f;
    vel_y = 0.0f;
    vel_z = 0.0f;
    pos_x = 0.0f;
    pos_y = 0.0f;
    pos_z = 0.0f;
  }
}

bool MPU6500::calibrarMPU()
{
  const int amostras = 1000;
  calibrado = false;

  #if defined(STM32_DEBUG_UART)
  Serial.println("===============================================================");
  Serial.println("Calibrando...");
  Serial.println("NAO MOVA O MPU!");
  #endif
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
    if (!lerMPU())
        return false;

    soma_gx += gyro_x;
    soma_gy += gyro_y;
    soma_gz += gyro_z;
    soma_ax += acc_x;
    soma_ay += acc_y;
    soma_az += acc_z;
    delay(2);
  }
  media_gyro_x = soma_gx / amostras; //tendênccia de cada eixo tanto do gyro quanto aceleromêtro
  media_gyro_y = soma_gy / amostras;
  media_gyro_z = soma_gz / amostras;
  media_acc_x = soma_ax / amostras;
  media_acc_y = soma_ay / amostras;
  media_acc_z = soma_az / amostras;
  delay(1000);

  //parte do sistema do eixo terrestre
  const int amostras_world = 200;
  float soma_wx = 0.0f, soma_wy = 0.0f, soma_wz = 0.0f;

  for (int i = 0; i < amostras_world; i++)
  {
    if (!lerMPU())
        return false;

    //Calibração, relativa aos vetores relativos à gravidade
    float ax_g = (float)acc_x / 16384.0f;
    float ay_g = (float)acc_y / 16384.0f;
    float az_g = (float)acc_z / 16384.0f;

    float gx_g = ((float)gyro_x - media_gyro_x) / 131.0f;
    float gy_g = ((float)gyro_y - media_gyro_y) / 131.0f;
    float gz_g = ((float)gyro_z - media_gyro_z) / 131.0f;

    float dt = 0.01f;
    filter.updateIMU(gx_g, gy_g, gz_g, ax_g, ay_g, az_g, dt);

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

  #if defined(STM32_DEBUG_UART)
  Serial.println("==============================");
  Serial.println(" CALIBRACAO CONCLUIDA");
  Serial.println("==============================");
  #endif
  return true;
}
