#ifndef ESC_ESP32_H
#define ESC_ESP32_H

#include <Arduino.h>

class ESC_ESP32
{
public:
  void inicializarUART(long baud = 115200)
  {
    Serial2.begin(baud);
    Serial.println("UART ESP32 OK");   // <-- CORRIGIDO: println
  }

  void enviarUART(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
  {
    Serial2.printf("%d,%d,%d,%d\n", m1, m2, m3, m4);
  }
};

#endif   