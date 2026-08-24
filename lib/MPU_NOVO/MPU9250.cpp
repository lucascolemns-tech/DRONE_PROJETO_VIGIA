#include "MPU9250.h"

MPU9250::MPU9250()
{
  angle_x = 0.0;
  angle_y = 0.0;
  angle_z = 0.0;

  vel_x = 0.0;
  vel_y = 0.0;
  vel_z = 0.0;

  pos_x = 0.0;
  pos_y = 0.0;
  pos_z = 0.0;
}

uint8_t MPU9250::lerRegistrador(uint8_t endereco) 
{
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_MPU, LOW);

  SPI.transfer(endereco | 0x80); 
  uint8_t valor = SPI.transfer(0x00); 

  digitalWrite(CS_MPU, HIGH);
  SPI.endTransaction();

  return valor;
}

void MPU9250::escreverRegistrador(uint8_t endereco, uint8_t valor) 
{
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_MPU, LOW);

  SPI.transfer(endereco & 0x7F); // Set the MSB to 0 for writing
  SPI.transfer(valor);

  digitalWrite(CS_MPU, HIGH);
  SPI.endTransaction();
}

uint8_t MPU9250::lerRegistradorI2C(uint8_t endereco, uint8_t registrador) 
{
  Wire.beginTransmission(endereco);
  Wire.write(registrador);

  Wire.endTransmission(false); 
  
  Wire.requestFrom((uint8_t) endereco, (uint8_t)1);

  if (Wire.available()) 
      return Wire.read();

  return 0xFF;
}

void MPU9250::lerAcelerometroGiroscopio() 
{
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_MPU, LOW);

  SPI.transfer(0x3B | 0x80); 

  for (uint8_t endereco = 0x3B; endereco <= 0x48; endereco++) 
  {
    uint8_t valor = SPI.transfer(0x00); 

    switch(endereco) 
    {
    //percorre endereços dedicados aos registradores acelerometro e giroscopio
    case 0x3B: ACC_X_H  =  valor; break;
    case 0x3C: ACC_X_L  =  valor; break;
    case 0x3D: ACC_Y_H  =  valor; break;
    case 0x3E: ACC_Y_L  =  valor; break;
    case 0x3F: ACC_Z_H  =  valor; break;
    case 0x40: ACC_Z_L  =  valor; break;
    case 0x41: TEMP_H   =  valor; break;
    case 0x42: TEMP_L   =  valor; break;
    case 0x43: GYRO_X_H =  valor; break;
    case 0x44: GYRO_X_L =  valor; break;
    case 0x45: GYRO_Y_H =  valor; break;
    case 0x46: GYRO_Y_L =  valor; break;
    case 0x47: GYRO_Z_H =  valor; break;
    case 0x48: GYRO_Z_L =  valor; break;
    }
  }

  //construir os valores de 16 bits a partir dos registradores de 8 bits
  ACC_X = (ACC_X_H << 8) | ACC_X_L;
  ACC_Y = (ACC_Y_H << 8) | ACC_Y_L;
  ACC_Z = (ACC_Z_H << 8) | ACC_Z_L;

  TEMP = (TEMP_H << 8) | TEMP_L;

  GYRO_X = (GYRO_X_H << 8) | GYRO_X_L;
  GYRO_Y = (GYRO_Y_H << 8) | GYRO_Y_L;
  GYRO_Z = (GYRO_Z_H << 8) | GYRO_Z_L;

  digitalWrite(CS_MPU, HIGH);
  SPI.endTransaction();
}

bool MPU9250::inicializar() 
{
  //SPI config
  pinMode(CS_MPU, OUTPUT);
  digitalWrite(CS_MPU, HIGH);

  SPI.begin();

  if (lerRegistrador(WHO_AM_I_MPU) != (MPU_ID))
  {
    Serial.println("ERRO: MPU9250 nao encontrado!");
    return false;
  }

  //tirar do modo sleep
  escreverRegistrador(PWR_MGMT_1, 0x00);

  //acelerometro config
  escreverRegistrador(ACCEL_CONFIG, 0x00);

  //giroscopio config
  escreverRegistrador(GYRO_CONFIG, 0x00);

  // Configurar filtro DLPF 
  escreverRegistrador(FILTER_CONFIG, 0x03);
  
  // Sample rate 
  escreverRegistrador(SMPLRT_DIV, 0x00);

  //I2C config
  Wire.begin();

  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(INT_PIN_CFG); 
  Wire.write(0x02); 

  Wire.endTransmission();

  return true;
}

void MPU9250::Sistemacoordenadas(
  float ax, float ay, float az,
  float roll, float pitch, float yaw,
  float &world_x, float &world_y, float &world_z) //PS: "float &world_xyz mantém o valor anterior salvo nela mesma
{
  float cr = cos(roll);
  float sr = sin(roll);

  float cp = cos(pitch);
  float sp = sin(pitch);

  float cy = cos(yaw);
  float sy = sin(yaw);

  world_x = (cy*cp)*ax
      + (cy*sp*sr - sy*cr)*ay
      + (cy*sp*cr + sy*sr)*az;

  world_y = (sy*cp)*ax
      + (sy*sp*sr + cy*cr)*ay
      + (sy*sp*cr - cy*sr)*az;

  world_z = (-sp)*ax
      + (cp*sr)*ay
      + (cp*cr)*az;
}

void MPU9250::calibrarAcelerometroGiroscopio()
{
  const int amostras = 1000;

  Serial.println("Calibrando...");
  Serial.println("NAO MOVA O MPU!");

  delay(5000);

  for (int i = 0; i < 1000; i++)
  {
    lerAcelerometroGiroscopio();

    amostra_x += GYRO_X;
    amostra_y += GYRO_Y;
    amostra_z += GYRO_Z;

    amostra_x2 += ACC_X;
    amostra_y2 += ACC_Y;
    amostra_z2 += ACC_Z;

    delay(2);
  }

  gyro_bias_x = (float)amostra_x / amostras;
  gyro_bias_y = (float)amostra_y / amostras;
  gyro_bias_z = (float)amostra_z / amostras;

  acc_bias_x = (float)amostra_x2 / amostras;
  acc_bias_y = (float)amostra_y2 / amostras;
  acc_bias_z = (float)amostra_z2 / amostras;

  Serial.println(" ");
  Serial.println(" ========== CALIBRAÇÃO 1 CONCLUÍDA COM ÊXITO ========== ");
}

void MPU9250::calibrarGravidade()
{
  const int amostras2 = 100;
  calibrado = false;

  for (int i = 0; i < amostras2; i++)
  {
    lerAcelerometroGiroscopio();
    MPUcalculos();

    somaX += ACC_X_World;
    somaY += ACC_Y_World;
    somaZ += ACC_Z_World;

    delay(2);
  }

  world_bias_x = somaX / amostras2;
  world_bias_y = somaY / amostras2;
  world_bias_z = somaZ / amostras2;

  calibrado = true;
  Serial.println(" ========== CALIBRAÇÃO 2 CONCLUÍDA COM ÊXITO ========== ");
}

void MPU9250::MPUcalculos()
{
  //gerenciamento da variavel de tempo para futura integração
  static unsigned long tempo_antes = 0;
  unsigned long tempo_depois = micros();
  
  if (tempo_antes == 0)
    {
    tempo_antes = tempo_depois;
    return;
    }

  float tempo = (tempo_depois - tempo_antes)/1000000.0f;

  if (tempo > 0.1)
    {
    tempo_antes = tempo_depois;
    return;
    }

  tempo_antes = tempo_depois;

  //Converter os valores dos registradores
  Converted_ACC_X = 9.81*(ACC_X - acc_bias_x)/16384.0; 
  Converted_ACC_Y = 9.81*(ACC_Y - acc_bias_y)/16384.0; 
  Converted_ACC_Z = 9.81*(ACC_Z - acc_bias_z)/16384.0;

  Converted_TEMP = (TEMP/333.87) + 21;

  Converted_GYRO_X = (GYRO_X - gyro_bias_x) / 131.0f;
  Converted_GYRO_Y = (GYRO_Y - gyro_bias_y) / 131.0f;
  Converted_GYRO_Z = (GYRO_Z - gyro_bias_z) / 131.0f;

  //Integrar os valores dos registradores
  angle_x += Converted_GYRO_X * tempo;
  angle_y += Converted_GYRO_Y * tempo;
  angle_z += Converted_GYRO_Z * tempo;

  //Calcular ângulos do acelerômetro em GRAUS (usando variáveis separadas)
  acc_roll_deg = atan2(Converted_ACC_Y, sqrt(pow(Converted_ACC_X, 2) + pow(Converted_ACC_Z, 2))) * 180.0 / pi;
  acc_pitch_deg = atan2(-Converted_ACC_X, sqrt(pow(Converted_ACC_Y, 2) + pow(Converted_ACC_Z, 2))) * 180.0 / pi;

  //Aplicar filtro complementar
  angle_x = ALPHA * angle_x + (1.0 - ALPHA) * acc_roll_deg;
  angle_y = ALPHA * angle_y + (1.0 - ALPHA) * acc_pitch_deg;
  angle_z *= GAMMA;

  //Converter para RADIANOS
  float roll_rad = angle_x * pi / 180.0f;
  float pitch_rad = angle_y * pi / 180.0f;
  float yaw_rad = angle_z * pi / 180.0f;

  roll  = roll_rad;
  pitch = pitch_rad;
  yaw   = yaw_rad;

  //rotacionar a matriz relativa a terra
  Sistemacoordenadas(
  Converted_ACC_X, Converted_ACC_Y, Converted_ACC_Z,
  roll, pitch, yaw, 
  ACC_X_World, ACC_Y_World, ACC_Z_World);

  //gravidade em cada eixo
  ACClinear_X_World = ACC_X_World - world_bias_x;
  ACClinear_Y_World = ACC_Y_World - world_bias_y;
  ACClinear_Z_World = ACC_Z_World - world_bias_z;

  static float filtered_acc_x = 0, filtered_acc_y = 0, filtered_acc_z = 0;
  if ((filtered_acc_x == 0) && (filtered_acc_y == 0) && (filtered_acc_z == 0))
  {
    filtered_acc_x = ACClinear_X_World;
    filtered_acc_y = ACClinear_Y_World;
    filtered_acc_z = ACClinear_Z_World;
  }

  //aplicar filtro EXPONENCIAL
  filtered_acc_x = acc_alpha * ACClinear_X_World + (1.0 - acc_alpha) * filtered_acc_x;
  filtered_acc_y = acc_alpha * ACClinear_Y_World + (1.0 - acc_alpha) * filtered_acc_y;
  filtered_acc_z = acc_alpha * ACClinear_Z_World + (1.0 - acc_alpha) * filtered_acc_z;

  mag_XYZ = sqrt(pow(ACClinear_X_World, 2) + 
                 pow(ACClinear_Y_World, 2) +
                 pow(ACClinear_Z_World, 2));
  
  if (mag_XYZ > limiar_movimento)
    movimento = true;
  else
    movimento = false;
  
  if (calibrado && movimento)
  {
    vel_x += filtered_acc_x * tempo;
    vel_y += filtered_acc_y * tempo;
    vel_z += filtered_acc_z * tempo;

    pos_x += vel_x * tempo;
    pos_y += vel_y * tempo;
    pos_z += vel_z * tempo;
  }
  else if (calibrado && !movimento)
  {
    vel_x *= 0.999;
    vel_y *= 0.999;
    vel_z *= 0.999;
  }
  else
  {
    vel_x = 0.0;
    vel_y = 0.0;
    vel_z = 0.0;

    pos_x = 0.0;
    pos_y = 0.0;
    pos_z = 0.0;
  }
}

