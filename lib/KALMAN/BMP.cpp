#include "BMP.h"

//curiosidade o underline antes da variavel é para dizer que é PRIVADO e EXCLUSIVA da classe
BMP::BMP(uint8_t endereco)
{
  /*
  porque não deixar o endereço fixo? Pois este bmp
  tem dois endereços possíveis, é preciso que esta
  parte possa transitar entre 0x76 e 0x77 e também se mantenha 
  durante todo o código, para isto inicializamos ela como _endereco 
  para que não se perca (caso fosse chamado somente endereco) ao final da chamada da classe
  */
   _endereco = endereco; 
  temperatura = 0.0;
  pressao = 0.0;
  t_lin = 0.0;
}

uint8_t BMP::lerRegistrador(uint8_t reg)
{
  Wire.beginTransmission(_endereco);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return 0xFF;

  uint8_t quantidade = Wire.requestFrom(_endereco, (uint8_t)1);

  //devolver 1 byte do registrador
  if (quantidade == 1 && Wire.available())
    return Wire.read();

  return 0xFF;
}

void BMP::escreverRegistrador(uint8_t reg, uint8_t valor)
{
  Wire.beginTransmission(_endereco);
  Wire.write(reg);
  Wire.write(valor);
  Wire.endTransmission();
}

void BMP::lerRegistradores(uint8_t reg, uint8_t *buffer, uint8_t tamanho)
{
  Wire.beginTransmission(_endereco);
  Wire.write(reg);

  //houve erro preencha todo o buffer com 0xFF
  if (Wire.endTransmission(false) != 0)
  {
    for (uint8_t i = 0; i < tamanho; i++)
      buffer[i] = 0xFF;
    
      return;
  }

  uint8_t quantidade = Wire.requestFrom(_endereco, tamanho);

  //ler os registradores e salvar no buffer 
  for (uint8_t i = 0; i < tamanho; i++)
  {
    if (Wire.available())
      buffer[i] = Wire.read();
    else
      buffer[i] = 0xFF;
  }
}

void BMP::lerCalibracao()
{
  //este algorítimo de calibração foi desenvolvido pela fabricante BOSCH
  uint8_t buf[21];

  lerRegistradores(CALIB, buf, 21);

  uint16_t nvm_t1 = ((uint16_t)buf[1] << 8) | buf[0];
  uint16_t nvm_t2 = ((uint16_t)buf[3] << 8) | buf[2];
  int8_t nvm_t3 = (int8_t)buf[4];
  int16_t nvm_p1 = (int16_t)(((uint16_t)buf[6] << 8) | buf[5]);
  int16_t nvm_p2 = (int16_t)(((uint16_t)buf[8] << 8) | buf[7]);
  int8_t nvm_p3 = (int8_t)buf[9];
  int8_t nvm_p4 = (int8_t)buf[10];
  uint16_t nvm_p5 = ((uint16_t)buf[12] << 8) | buf[11];
  uint16_t nvm_p6 = ((uint16_t)buf[14] << 8) | buf[13];
  int8_t nvm_p7 = (int8_t)buf[15];
  int8_t nvm_p8 = (int8_t)buf[16];
  int16_t nvm_p9 = (int16_t)(((uint16_t)buf[18] << 8) | buf[17]);
  int8_t nvm_p10 = (int8_t)buf[19];
  int8_t nvm_p11 = (int8_t)buf[20];

  par_t1 = (double)nvm_t1 * 256.0;
  par_t2 = (double)nvm_t2 / 1073741824.0;
  par_t3 = (double)nvm_t3 / 281474976710656.0;
  par_p1 = ((double)nvm_p1 - 16384.0) / 1048576.0;
  par_p2 = ((double)nvm_p2 - 16384.0) / 536870912.0;
  par_p3 = (double)nvm_p3 / 4294967296.0;
  par_p4 = (double)nvm_p4 / 137438953472.0;
  par_p5 = (double)nvm_p5 * 8.0;
  par_p6 = (double)nvm_p6 / 64.0;
  par_p7 = (double)nvm_p7 / 256.0;
  par_p8 = (double)nvm_p8 / 32768.0;
  par_p9 = (double)nvm_p9 / 281474976710656.0;
  par_p10 = (double)nvm_p10 / 281474976710656.0;
  par_p11 = (double)nvm_p11 / 36893488147419103232.0;
}

float BMP::compensarTemperatura(uint32_t raw_temp)
{
  //este algorítimo de compensação de temperatura foi desenvolvido pela fabricante BOSCH
  double partial1;
  double partial2;

  partial1 = (double)raw_temp - par_t1;
  partial2 = partial1 * par_t2;

  t_lin = partial2 + (partial1 * partial1) * par_t3;

  return (float)t_lin;
}

float BMP::compensarPressao(uint32_t raw_press)
{
  //este algorítimo de compensação de pressão foi desenvolvido pela fabricante BOSCH
  double p = (double)raw_press;
  double t = t_lin;

  double partial_data1;
  double partial_data2;
  double partial_data3;
  double partial_data4;

  double partial_out1;
  double partial_out2;

  //primeiro termo
  partial_data1 = par_p6 * t;
  partial_data2 = par_p7 * (t * t);
  partial_data3 = par_p8 * (t * t * t);
  
  partial_out1 = par_p5 + partial_data1 + partial_data2 + partial_data3;

  //segundo termo
  partial_data1 = par_p2 * t;
  partial_data2 = par_p3 * (t * t);
  partial_data3 = par_p4 * (t * t * t);
    
  partial_out2 = p * (par_p1 + partial_data1 + partial_data2 + partial_data3);

  //terceiro termo
  partial_data1 = p * p;
  partial_data2 = par_p9 + par_p10 * t;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 +(pow(p, 3)) * par_p11;

  //resultado
  double press = partial_out1 + partial_out2 + partial_data4;
  return (float)press;
}

bool BMP::inicializar()
{
  Wire.begin(SDA, SCL);
  Wire.setClock(100000);

  delay(50);
  Wire.beginTransmission(_endereco);

  if (Wire.endTransmission() != 0)
  {
    Serial.print("BMP388 nao encontrado em 0x");
    Serial.println(_endereco, HEX);
    return false;
  }

  Serial.print("BMP388 encontrado em 0x");
  Serial.println(_endereco, HEX);

  uint8_t id = lerRegistrador(REG_CHIP_ID);
  Serial.print("CHIP_ID = 0x");

 //garantir que sejam exibidos sempre dois digitos, se for menor que 16 exibe so um
  if (id < 0x10)
    Serial.print("0");

  Serial.println(id, HEX);

  if (id != BMP_ID)
  {
    Serial.println("CHIP_ID incorreto!");
    return false;
  }

  lerCalibracao();

  escreverRegistrador(OSR, 0x03); //ultra alta resolução
  escreverRegistrador(REG_CONFIG, 0x00);
  escreverRegistrador(ODR, 0x00); //taxa de amostras 
  escreverRegistrador(PWR_CTRL, 0x00); //tirar do sleep

  Serial.println("BMP inicializado!");

  return true;
}

void BMP::lerBMP()
{
  escreverRegistrador(PWR_CTRL,0x13); //modo forçado

  unsigned long tempoComeco = millis();

  while (millis() - tempoComeco < 100) 
  {
    uint8_t status = lerRegistrador(REG_STATUS);
    if (status & 0x10)
      break;
    delay(1);
  }

  uint8_t dados[6];
  lerRegistradores(REG_DATA, dados, 6);

  //construção dos valores brutos a partir dos registradores
  uint32_t press_bruta = ((uint32_t)dados[2] << 16) | ((uint32_t)dados[1] << 8) | ((uint32_t)dados[0]);
  uint32_t temp_bruta  = ((uint32_t)dados[5] << 16) | ((uint32_t)dados[4] << 8) | ((uint32_t)dados[3]);

  temperatura = compensarTemperatura(temp_bruta);
  pressao = compensarPressao(press_bruta);
}