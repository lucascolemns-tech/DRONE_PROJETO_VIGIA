#include "MPU.h"

MPU mpu;
bool MPUiniciado = false;

void setup()
{
  Serial.begin(115200);
  Serial.println("Inicializando...");

  while (!MPUiniciado) 
  {
    MPUiniciado = mpu.inicializar();
    if (!MPUiniciado) 
    {
      Serial.println("Falha ao inicializar o MPU. Tentando novamente...");
      delay(500);
    }
  }
  Serial.println("MPU OK");

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
}

void loop()
{
    mpu.lerMPU();
    mpu.MPUcalculos();

    if (mpu.calibrado) 
    {
      // ângulos
      Serial.print(mpu.angulo_x); Serial.print(",");
      Serial.print(mpu.angulo_y); Serial.print(",");
      Serial.print(mpu.angulo_z); Serial.print(",");

      Serial.print("  |  ");

      // velocidade
      Serial.print(mpu.vel_x, 3); Serial.print(",");
      Serial.print(mpu.vel_y, 3); Serial.print(",");
      Serial.print(mpu.vel_z, 3); Serial.print(",");
      
      Serial.print("  |  ");
      Serial.println(" ");
    }
  }
