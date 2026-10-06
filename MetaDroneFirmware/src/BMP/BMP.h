#ifndef BMP_H
#define BMP_H

#include <Arduino.h>
#include <Wire.h>

//endereço fixo bmp180
constexpr uint8_t BMP180_I2C_ADDRESS = 0x77;

class BMP
{
public:
  explicit BMP(uint8_t endereco = BMP180_I2C_ADDRESS); //explicit evita conversão implícita de inteiro para BMP
  bool lerRegistradores(uint8_t reg, uint8_t *buffer, uint8_t tamanho);
  bool lerRegistrador(uint8_t reg, uint8_t &valor);
  bool escreverRegistrador(uint8_t reg, uint8_t valor);
  bool lerCalibracao();
  bool iniciarTemperatura();
  bool iniciarPressao();
  bool concluirTemperatura();
  bool concluirPressao();
  bool inicializar();
  bool lerBMP();
  uint8_t endereco() const { return _endereco; }
  bool amostraValida() const { return amostra_valida; }
  unsigned long idadeAmostraMs() const { return amostra_valida ? millis() - tempo_ultima_amostra_ms : 0xFFFFFFFFUL; } //se não for válida retorna o valor máximo de 32 bits
  unsigned long tempoAmostraMs() const { return tempo_ultima_amostra_ms; }
  float temperatura = 0.0f;
  float pressao = 0.0f;

private:
  enum class Conversao_respectiva : uint8_t { None, Temperatura, Pressao };

  static constexpr uint8_t OSS = 1; // taxa de 7.5ms
  static constexpr uint8_t REG_CHIP_ID = 0xD0;
  static constexpr uint8_t BMP180_CHIP_ID = 0x55;
  static constexpr uint8_t REG_CALIBRATION = 0xAA;
  static constexpr uint8_t REG_CONTROL = 0xF4;
  static constexpr uint8_t REG_DATA = 0xF6;
  static constexpr uint8_t REG_TEMP_COMMAND = 0x2E;

  uint8_t _endereco;
  bool amostra_valida = false;
  unsigned long tempo_ultima_amostra_ms = 0;
  unsigned long conversao_pronta_ms = 0;
  unsigned long ultima_temperatura_ms = 0;
  Conversao_respectiva conversao = Conversao_respectiva::None;
  int32_t b5 = 0;

  int16_t ac1 = 0, ac2 = 0, ac3 = 0;
  uint16_t ac4 = 0, ac5 = 0, ac6 = 0;
  int16_t b1 = 0, b2 = 0, mb = 0, mc = 0, md = 0;
};

#endif
