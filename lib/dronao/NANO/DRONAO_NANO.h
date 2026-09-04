//modularização do código
#ifndef ESC_NANO_H
#define ESC_NANO_H

//bibliotecas
#include <Servo.h>
#include <SoftwareSerial.h>

class ESC_NANO
{
public:
  int m1 = 0, m2 = 0, m3 = 0, m4 = 0; // CORRIGIDO: Ponto e vírgula!

  ESC_NANO(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4, 
             uint8_t rx, uint8_t tx, long baud = 115200)
  {
    pinos[0] = m1; pinos[1] = m2; pinos[2] = m3; pinos[3] = m4;
    for (int j = 0; j < 4; j++)
    {
      motores_pwm[j].attach(pinos[j], 1000, 2000); 
      motores_pwm[j].writeMicroseconds(1000); 
    }
    serial = new SoftwareSerial(rx, tx);
    serial->begin(baud);
    armazenador.reserve(32);
  }

  void ESCatualizar()
  {
    while(serial->available())
    {
      char dado_recebido = serial->read(); 
      if (dado_recebido == '\n')
      {
        if(armazenador.length() > 0)
        {
          if (sscanf(armazenador.c_str(), "%d,%d,%d,%d", &m1, &m2, &m3, &m4) == 4)
          {
            motores_pwm[0].writeMicroseconds(constrain(m1, 1000, 2000));
            motores_pwm[1].writeMicroseconds(constrain(m2, 1000, 2000));
            motores_pwm[2].writeMicroseconds(constrain(m3, 1000, 2000));
            motores_pwm[3].writeMicroseconds(constrain(m4, 1000, 2000));
          }
          armazenador = ""; 
        }
      }
      else
      {
        armazenador += dado_recebido; 
      }
    }
  }

  void parar()
  {
    for (int k = 0; k < 4; k++)
      motores_pwm[k].writeMicroseconds(1000);
  }

private:
  Servo motores_pwm[4];
  SoftwareSerial* serial;
  String armazenador;
  uint8_t pinos[4];
};

#endif