#include "DRONAO_NANO.h"

#define M1_PINO 9
#define M2_PINO 10
#define M3_PINO 11
#define M4_PINO 12

#define RX_PINO 7
#define TX_PINO 6

ESC_NANO esc(M1_PINO, M2_PINO, M3_PINO, M4_PINO, RX_PINO, TX_PINO);

void setup()
{
  Serial.begin(115200);
  Serial.print("NANO OK");
  delay(3000); // Tempo para os ESCs armarem
}

void loop()
{
  esc.ESCatualizar(); // Lê os dados do ESP32
  
  Serial.print(esc.m1); 
  Serial.print(",");
  Serial.print(esc.m2); 
  Serial.print(","); 
  Serial.print(esc.m3); 
  Serial.print(",");  
  Serial.print(esc.m4); 
  Serial.println(); 

  delay(10); 
}