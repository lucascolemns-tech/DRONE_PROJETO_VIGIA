#include "MPU9250.h"

MPU9250 mpu;

bool MPUiniciado  = false;

void setup()
{
  Serial.begin(9600);
  Serial.println("Inicializando...");

  while (!MPUiniciado)
  {
    MPUiniciado = mpu.inicializar();
    if (!MPUiniciado) 
      delay(500);
  }
  Serial.println("MPU OK");

  mpu.calibrarAcelerometroGiroscopio();
  mpu.calibrarGravidade();
  
  Serial.println("CALIBRADO");
    // VERIFICAR VALORES DE CALIBRAÇÃO

  Serial.println("\n=== VALORES DE CALIBRAÇÃO ===");
  Serial.print("ACC Bias X: "); Serial.println(mpu.acc_bias_x);
  Serial.print("ACC Bias Y: "); Serial.println(mpu.acc_bias_y);
  Serial.print("ACC Bias Z: "); Serial.println(mpu.acc_bias_z);
  Serial.print("GYRO Bias X: "); Serial.println(mpu.gyro_bias_x);
  Serial.print("GYRO Bias Y: "); Serial.println(mpu.gyro_bias_y);
  Serial.print("GYRO Bias Z: "); Serial.println(mpu.gyro_bias_z);
  Serial.print("World Bias X: "); Serial.println(mpu.world_bias_x, 3);
  Serial.print("World Bias Y: "); Serial.println(mpu.world_bias_y, 3);
  Serial.print("World Bias Z: "); Serial.println(mpu.world_bias_z, 3);
  Serial.println("============================\n");
}
  

void loop()
{
  mpu.lerAcelerometroGiroscopio();
  mpu.MPUcalculos();

  if (mpu.calibrado) 
  {
    //angulo
    Serial.print(mpu.angle_x); Serial.print(",");
    Serial.print(mpu.angle_y); Serial.print(",");
    Serial.print(mpu.angle_z); Serial.print(",");

    Serial.print(" "); Serial.print(" ");  
    Serial.print("|"); 
    Serial.print(" "); Serial.print(" ");  

    //velocidade
    Serial.print(mpu.vel_x, 3); Serial.print(",");
    Serial.print(mpu.vel_y, 3); Serial.print(",");
    Serial.print(mpu.vel_z, 3); Serial.print(",");

    Serial.print(" "); Serial.print(" ");  
    Serial.print("|"); 
    Serial.print(" "); Serial.print(" ");  

    //posição
    Serial.print(mpu.pos_x, 3); Serial.print(",");
    Serial.print(mpu.pos_y, 3); Serial.print(",");
    Serial.println(mpu.pos_z, 3);
  }

  delay(50);

}