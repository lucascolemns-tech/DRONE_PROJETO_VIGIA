#include "BMP.h"
#include <math.h>

//variáveis globais de gerenciamento de tempo
constexpr unsigned long BMP180_TEMPERATURE_CONVERSION_MS = 5;
constexpr unsigned long BMP180_PRESSURE_CONVERSION_MS    = 8;
constexpr unsigned long BMP180_TEMPERATURE_REFRESH_MS    = 1000;
constexpr unsigned long BMP180_SAMPLE_STALE_MS           = 200;

//"static_cast" é usado quando váriaveis são dois tipos diferentes e deseja-se fzr uma opração entre
int16_t  readS16(const uint8_t *p) { return static_cast<int16_t>(static_cast<uint16_t>(p[0]) << 8 | p[1]); }
uint16_t readU16(const uint8_t *p) { return static_cast<uint16_t>(p[0]) << 8 | p[1]; }

//construtor, coloqe o endereço correto a ser analisado
BMP::BMP(uint8_t endereco) : _endereco(endereco)
{}

bool BMP::lerRegistradores(uint8_t reg, uint8_t *buffer, uint8_t tamanho)
{
  Wire.beginTransmission(_endereco);
  Wire.write(reg); //você especifica o registrador q deseja
  if (Wire.endTransmission() != 0)
    return false; //checagem padrão i2c

  if (Wire.requestFrom(_endereco, tamanho) != tamanho)
    return false; //pediu endeço e pediu o tanto qe vai retornar

  for (uint8_t i = 0; i < tamanho; ++i) //construir o buffer
  {
    if (!Wire.available())
      return false;
    buffer[i] = static_cast<uint8_t>(Wire.read()); //operações entre unsigned e signed values dos registradores do bmp180
  }
  return true;
}

bool BMP::lerRegistrador(uint8_t reg, uint8_t &valor)
{
  uint8_t data = 0;
  if (!lerRegistradores(reg, &data, 1)) //caso não passe pelas checagem, retorna
    return false;
  valor = data; //valor lido no endereço do registrador salvo
  return true;
}

bool BMP::escreverRegistrador(uint8_t reg, uint8_t valor)
{
  Wire.beginTransmission(_endereco);
  Wire.write(reg); //primeiro endereço
  Wire.write(valor); //depois escreve o vaor
  return Wire.endTransmission() == 0; //fecha bus i2c
}

bool BMP::lerCalibracao()
{
  uint8_t c[22]; //tottal de valores a serem salvos
  if (!lerRegistradores(REG_CALIBRATION, c, sizeof(c)))
      return false;

  ac1 = readS16(&c[0]); //lê 16 bits do endereço 0xAA e 0xAB, e assim por diante
  ac2 = readS16(&c[2]);
  ac3 = readS16(&c[4]);
  ac4 = readU16(&c[6]);
  ac5 = readU16(&c[8]);
  ac6 = readU16(&c[10]);
  b1  = readS16(&c[12]);
  b2  = readS16(&c[14]);
  mb  = readS16(&c[16]);
  mc  = readS16(&c[18]);
  md  = readS16(&c[20]);

  bool todosZERO = true;
  bool todosUM = true;
  //for "value : x" para cada valor do array c, verifica se todos são 0x00 ou 0xFF, repete a cada valor do array c
  for (uint8_t value : c) { todosZERO = todosZERO && (value == 0x00); todosUM = todosUM && (value == 0xFF); }
  bool checagem = (!todosZERO && !todosUM && ac4 != 0 && ac5 != 0 && ac6 != 0 && md != 0);
  return checagem;
}

bool BMP::iniciarTemperatura() //definir os parâmetros de tempo para conversão da temperatura, e iniciar a conversão
{
  if (!escreverRegistrador(REG_CONTROL, REG_TEMP_COMMAND))
    return false;
  conversao           = Conversao_respectiva::Temperatura;
  conversao_pronta_ms = millis() + BMP180_TEMPERATURE_CONVERSION_MS;
  return true;
}

bool BMP::iniciarPressao() //definir os parâmetros de tempo para conversão da pressão, e iniciar a conversão
{
  if (!escreverRegistrador(REG_CONTROL, static_cast<uint8_t>(0x34 | (OSS << 6)))) //definir modo
    return false; //valida dados dos registradores
  conversao = Conversao_respectiva::Pressao; //retorna objeto
  conversao_pronta_ms = millis() + BMP180_PRESSURE_CONVERSION_MS;
  return true;
}

//formula para compensar temperatrura
bool BMP::concluirTemperatura()
{
  uint8_t brutalidade[2]; //valores brutos
  if (!lerRegistradores(REG_DATA, brutalidade, sizeof(brutalidade)))
    return false;

  const int32_t uncompensatedTemperature = static_cast<int32_t>(static_cast<uint16_t>(brutalidade[0]) << 8 | brutalidade[1]);
  const int64_t x1 = (static_cast<int64_t>(uncompensatedTemperature - ac6) * ac5) >> 15;
  const int64_t denominador = x1 + md;
  
  if (denominador == 0) //evitar divisão por 0
    return false;

  const int64_t x2 = (static_cast<int64_t>(mc) << 11) / denominador;
  const int64_t newB5 = x1 + x2;
  if (newB5 < INT32_MIN || newB5 > INT32_MAX)
    return false; //não permite que ultrapase valores 

  b5 = static_cast<int32_t>(newB5); //operações entre valores unsigned e signed
  const int32_t temperaturaT = (b5 + 8) >> 4;
  temperatura           = static_cast<float>(temperaturaT) / 10.0f;
  ultima_temperatura_ms = millis();
  return isfinite(temperatura) && temperatura >= -40.0f && temperatura <= 85.0f; //verifica se o valor é real
}

//função para compensar a pressão, com base na temperatura
bool BMP::concluirPressao()
{
  uint8_t brutalidade[3];
  if (!lerRegistradores(REG_DATA, brutalidade, sizeof(brutalidade)))
    return false;

  const uint32_t uncompensatedPressure =
    ((static_cast<uint32_t>(brutalidade[0]) << 16) |
    (static_cast<uint32_t>(brutalidade[1]) << 8)  |
    static_cast<uint32_t>(brutalidade[2])) >> (8 - OSS); //modo de obter dados dos registradores

  const int64_t b6 = static_cast<int64_t>(b5) - 4000;
  int64_t x1 = (static_cast<int64_t>(b2) * ((b6 * b6) >> 12)) >> 11;
  int64_t x2 = (static_cast<int64_t>(ac2) * b6) >> 11;
  int64_t x3 = x1 + x2;
  const int64_t b3 = (((static_cast<int64_t>(ac1) * 4 + x3) << OSS) + 2) >> 2;

  x1 = (static_cast<int64_t>(ac3) * b6) >> 13;
  x2 = (static_cast<int64_t>(b1) * ((b6 * b6) >> 12)) >> 16;
  x3 = (x1 + x2 + 2) >> 2;

  const int64_t b4Factor = x3 + 32768;
  if (b4Factor <= 0)
    return false;

  const uint64_t b4 = (static_cast<uint64_t>(ac4) * static_cast<uint64_t>(b4Factor)) >> 15;
  if (b4 == 0 || b3 < 0 || uncompensatedPressure < static_cast<uint64_t>(b3))
    return false;

  const uint64_t b7 = (static_cast<uint64_t>(uncompensatedPressure) - static_cast<uint64_t>(b3)) * (50000U >> OSS);
  int64_t pressure = (b7 < 0x80000000ULL) ? static_cast<int64_t>((b7 * 2U) / b4) : static_cast<int64_t>((b7 / b4) * 2U);

  x1 = ((pressure >> 8) * (pressure >> 8) * 3038) >> 16;
  x2 = (-7357 * pressure) >> 16;
  pressure += (x1 + x2 + 3791) >> 4;

  if (pressure < 30000 || pressure > 120000)
    return false;

  pressao                 = static_cast<float>(pressure);
  tempo_ultima_amostra_ms = millis();
  amostra_valida          = true;
  return isfinite(pressao);
}

bool BMP::inicializar()
{
  amostra_valida = false;
  temperatura = 0.0f;
  pressao = 0.0f;
  conversao = Conversao_respectiva::None; //atribui o valor de conversão como nenhum, para que seja iniciado a conversão de temperatura e pressão
  _endereco = BMP180_I2C_ADDRESS; //força o endereço padrão, é meio q para caso tenha passado o endereço errado no construtor
  uint8_t id = 0;

  if (!lerRegistrador(REG_CHIP_ID, id))
  {
  #if defined(STM32_DEBUG_UART)
    Serial.println("BMP180 NACK em 0x77"); //retornar o valor lido pela função de chamada
  #endif
    return false;
  }

  if (id != BMP180_CHIP_ID)
  {
  #if defined(STM32_DEBUG_UART)
    Serial.print("BMP180 chip ID incorreto: 0x");
    if (id < 0x10) Serial.print('0');
    Serial.println(id, HEX);
  #endif
    return false;
  }

  if (!lerCalibracao())
  {
  #if defined(STM32_DEBUG_UART)
    Serial.println("BMP180 calibracao invalida");
  #endif
    return false;
  }

  #if defined(STM32_DEBUG_UART)
    Serial.println("BMP180 detectado (ID=0x55), OSS=1"); //endereço padrão, modo de operação
  #endif
    return iniciarTemperatura();
}

bool BMP::lerBMP()
{
    //gerenciamento de tempo
    const unsigned long now = millis();
    const bool amostra_fresca = amostra_valida && (now - tempo_ultima_amostra_ms) <= BMP180_SAMPLE_STALE_MS;

    if (conversao == Conversao_respectiva::None) //atribui o valor de conversão como nenhum, para que seja iniciado a conversão de temperatura e pressão
    {
      const bool iniciado =
          (now - ultima_temperatura_ms >= BMP180_TEMPERATURE_REFRESH_MS)
              ? iniciarTemperatura() : iniciarPressao(); //gerencia o tempo que vai ler a temperatura e a pressão, se o tempo de leitura da temperatura for maior que 1s, inicia a leitura da temperatura, caso contrário, inicia a leitura da pressão
      return iniciado || amostra_fresca; //após a conversão, retorna se a amostra é válida ou não
    }

    if (static_cast<int32_t>(now - conversao_pronta_ms) < 0)
      return amostra_fresca; //retorna se a amostra válida ou não

    const Conversao_respectiva completed = conversao;
    conversao = Conversao_respectiva::None; //atribui um objeto vazio à conversão, para que seja iniciado a conversão de temperatura e pressão

    if (completed == Conversao_respectiva::Temperatura)
    {
      if (!concluirTemperatura() || !iniciarPressao())
        return false;
      return amostra_fresca;
    }

    if (completed == Conversao_respectiva::Pressao)
    {
      if (!concluirPressao())
        return false;
      const bool iniciado = (millis() - ultima_temperatura_ms >= BMP180_TEMPERATURE_REFRESH_MS) ? iniciarTemperatura() : iniciarPressao();
      return iniciado;
    }
    return false;
}
