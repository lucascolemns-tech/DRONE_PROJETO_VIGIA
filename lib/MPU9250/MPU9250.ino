//Por enquanto esse código só está testando o MPU, eu recomendo testar todos os 3 sensores nesse código, mas também dá pra testar em outro

#include "MPU9250.h"

unsigned long tempo = millis();
MPU9250 mpu;

bool MPUStarted;

int Matriz_XYZ[6];

void setup()
{
  Serial.begin(9600);
  while (!MPUStarted)
  {
    Serial.println("Inicializando...");
    Serial.println(MPUStarted);

    MPUStarted = mpu.inicializar();
  }
  Serial.print("MPU9250 INICIALIZADOOOO");
}

void loop()
{

  mpu.lerMagnetometro();
  
  mpu.lerAcelerometroGiroscopio();

  mpu.MPUcalculos();

  Matriz_XYZ[0] = mpu.ACC_X;
  Matriz_XYZ[1] = mpu.ACC_Y;
  Matriz_XYZ[2] = mpu.ACC_Z;

  Matriz_XYZ[3] = mpu.GYRO_X;
  Matriz_XYZ[4] = mpu.GYRO_Y;
  Matriz_XYZ[5] = mpu.GYRO_Z;

  Serial.print("Aaa");
             
for (int i = 0; i < 6; i++)
{
  Serial.print(Matriz_XYZ[i]);
}

}
