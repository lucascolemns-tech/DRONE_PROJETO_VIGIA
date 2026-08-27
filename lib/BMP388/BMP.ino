#include "BMP.h"

BMP bmp(0x77); //incializar seu endereço!

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (bmp.inicializar()) {
    Serial.println("BMP388 pronto!");
  } 
  else 
  {
    Serial.println("Falha!");
    while (1);
  }
}

void loop() 
{
  bmp.lerBMP();
  Serial.printf("Temp: %.2f °C | Press: %.2f hPa\n", bmp.temperatura, bmp.pressao / 100.0);
}