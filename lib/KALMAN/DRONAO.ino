#include "BMP.h"
#include "MPU.h"
#include "KALMAN.h"   

MPU mpu;
BMP bmp(0x77); //incializar seu endereço!
KALMAN_LINEAR kf(0.1f, 0.5f);

bool MPUiniciado = false;
unsigned long tempoAnterior = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  //MPU6500
  while (!MPUiniciado) 
  {
    MPUiniciado = mpu.inicializar();
    if (!MPUiniciado) 
    {
      Serial.println("Falha ao inicializar o MPU. Tentando novamente...");
      delay(500);
    }
  }
  mpu.calibrarMPU();

  Serial.println("CALIBRADO");

  Serial.println("\n=== VALORES DE CALIBRAÇÃO ===");
  
  Serial.print("ACC X: ");   Serial.println(mpu.media_acc_x);
  Serial.print("ACC Y: ");   Serial.println(mpu.media_acc_y);
  Serial.print("ACC Z: ");   Serial.println(mpu.media_acc_z);
  Serial.print("GYRO X: ");  Serial.println(mpu.media_gyro_x);
  Serial.print("GYRO Y: ");  Serial.println(mpu.media_gyro_y);
  Serial.print("GYRO Z: ");  Serial.println(mpu.media_gyro_z);

  Serial.println("============================\n");

  //BMP388
  if (bmp.inicializar()) 
    Serial.println("BMP388 pronto!");
  else 
  {
    Serial.println("Falha!");
    while (1);
  }

  bmp.lerBMP();

  float alt_inicial = 44330.0 * (1.0 - pow(bmp.pressao / 101325.0, 0.1903));
  kf.setAlt(alt_inicial);  
  kf.setVel(0.0);
  tempoAnterior = micros(); 
}

void loop() 
{
  unsigned long tempoAgora = micros();
  float dt = (tempoAgora - tempoAnterior)/1000000.0;
  tempoAnterior = tempoAgora;

  mpu.lerMPU();
  mpu.MPUcalculos();
  bmp.lerBMP();

  kf.CHUTE(mpu.filtro_z, dt); 

  float baro_alt = 44330.0 * (1.0 - pow(bmp.pressao / 101325.0, 0.1903)); //converter para metros 
  kf.atualizarKALMAN(baro_alt);

  float altura_filtrada = kf.getAlt();
  float velocidade_filtrada = kf.getVel();
  
  Serial.printf("Temp: %.2f °C | Press: %.2f hPa\n", bmp.temperatura, bmp.pressao / 100.0); //

  Serial.println("");
  
  if (mpu.calibrado) 
  {
    //ângulos
    Serial.print(mpu.angulo_x); Serial.print(",");
    Serial.print(mpu.angulo_y); Serial.print(",");
    Serial.print(mpu.angulo_z); Serial.print(",");

    Serial.print("  |  ");

    //velocidade do MPU 
    Serial.print(mpu.vel_x, 3); Serial.print(",");
    Serial.print(mpu.vel_y, 3); Serial.print(",");
    Serial.print(mpu.vel_z, 3); Serial.print(",");
    
    Serial.print("  |  ");

    //velocidade e altura filtradas pelo Kalman
    Serial.print(altura_filtrada, 4); Serial.print(",");
    Serial.print(velocidade_filtrada, 4); Serial.print(",");
    
    Serial.print("  |  ");
    Serial.println(" ");
  }
}