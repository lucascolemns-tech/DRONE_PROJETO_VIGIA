#ifndef ESC_ESP32_H
#define ESC_ESP32_H
#define PINO_M1 12
#define PINO_M2 13
#define PINO_M3 14
#define PINO_M4 15 // colocar os pinos corretos


#include <Arduino.h>
#include <Servo.h>
class ESC_ESP32
{
private:
    Servo ESC_m1;
    Servo ESC_m2;
    Servo ESC_m3;
    Servo ESC_m4;
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
  void armarESC(){
    ESC_m1.attach(PINO_M1);
    ESC_m2.attach(PINO_M2);
    ESC_m3.attach(PINO_M3);
    ESC_m4.attach(PINO_M4);

    ESC_m1.writeMicroseconds(1000);  // throttle mínimo
    ESC_m2.writeMicroseconds(1000);
    ESC_m3.writeMicroseconds(1000);
    ESC_m4.writeMicroseconds(1000);

    delay(10000);
  }
};

#endif   
