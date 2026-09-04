#include "BMP.h"
#include "ESC.h"
#include "MPU.h"
#include "PID.h"
#include "KALMAN.h"   
#include "INTERNET_ESP.h"
#include <ESP32Servo.h>

MPU mpu;
BMP bmp(0x77); //incializar seu endereço!
PID pid;
KALMAN_LINEAR kf(0.1f, 0.5f);
ESC_ESP32 esc_uart;
INTERNET_ESP_H internet; 

bool MPUiniciado = false;
unsigned long tempoAnterior = 0;

void setup() 
{
  Serial.begin(115200);
  while (!Serial);

  //WiFi
  internet.internet_init();

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

  //Nano
  esc_uart.inicializarUART();

  //throttle, 0% desligado motores, 50% pairar, 100% subir força total
  pid.SetBaseThrottle(1500); //50% mapeação do servo é de 1000 a 2000
}

void loop() 
{
  internet.internet_verificarCliente();

  String comando_recebido = internet.internet_receber();

  double comando_x = 0.0;
  double comando_y = 0.0;
  double comando_z = 0.0;
  double throttle = 0.0;

  if (comando_recebido.length() > 0)
  {
    if (comando_recebido.indexOf(',') != -1)
    {
      int i1 = comando_recebido.indexOf(',');
      int i2 = comando_recebido.indexOf(',', i1 + 1);
      int i3 = comando_recebido.indexOf(',', i2 + 1);

      if (i1 != -1 && i2 != -1 && i3 != -1)
      {
        comando_x = comando_recebido.substring(0, i1).toFloat();
        comando_y = comando_recebido.substring(i1 + 1, i2).toFloat();
        comando_z = comando_recebido.substring(i2 + 1, i3).toFloat();
        throttle = comando_recebido.substring(i3 + 1).toFloat();

        int throttle_pwm = 1000 + (int)(throttle * 10.0);
        pid.SetBaseThrottle(throttle_pwm);

        Serial.printf("Setpoint recebido: Roll=%.2f, Pitch=%.2f, Yaw=%.2f, Throttle=%.2f\n", 
                      comando_x, comando_y, comando_z, throttle);
      }
    }

    if (comando_recebido.indexOf(';') != -1)
    {
      int i1 = comando_recebido.indexOf(';');
      int i2 = comando_recebido.indexOf(';', i1 + 1);

      if (i1 != -1 && i2 != -1)
      {
        float kp = comando_recebido.substring(0, i1).toFloat();
        float ki = comando_recebido.substring(i1 + 1, i2).toFloat();
        float kd = comando_recebido.substring(i2 + 1).toFloat();

        pid.Config(kp, kd, kp, kd, kp, kd, kp, kd);

        Serial.printf("PID Recebido do Python: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp, ki, kd);
      }
    }
  }

  // Sensores e Kalman
  unsigned long tempoAgora = micros();
  float dt = (tempoAgora - tempoAnterior)/1000000.0;
  tempoAnterior = tempoAgora;

  mpu.lerMPU();
  mpu.MPUcalculos();

  unsigned long tempoBMP = 0;
  // Lê o BMP apenas a cada 100ms (sem bloquear o loop)
  if (millis() - tempoBMP >= 100) {
    tempoBMP = millis();
    bmp.lerBMP();
  }

  kf.CHUTE(mpu.filtro_z, dt); 

  float baro_alt = 44330.0 * (1.0 - pow(bmp.pressao / 101325.0, 0.1903)); //converter para metros 
  kf.atualizarKALMAN(baro_alt);

  float altura_filtrada = kf.getAlt();
  float velocidade_filtrada = kf.getVel();

  pid.Input(mpu.angulo_x, mpu.angulo_y, mpu.angulo_z, kf.getAlt());
  pid.Setpoint(comando_x, comando_y, comando_z, 0.0);
  pid.RunPID(true, true, true, false);

  double m1 = pid.GetM1();
  double m2 = pid.GetM2();
  double m3 = pid.GetM3();
  double m4 = pid.GetM4(); 

  esc_uart.enviarUART(m1, m2, m3, m4);

  char buffer_telemetria[128];
  sprintf(buffer_telemetria, "%.2f,%.2f,%.2f,%.4f,%.4f,%.2f,%.0f,%.0f,%.0f,%.0f\n",
          mpu.angulo_x, mpu.angulo_y, mpu.angulo_z,
          altura_filtrada, velocidade_filtrada, bmp.temperatura,
          m1, m2, m3, m4);

  internet.internet_enviar(buffer_telemetria);

  Serial.printf("Temp: %.2f °C | Press: %.2f hPa\n", bmp.temperatura, bmp.pressao / 100.0);
  Serial.println("");
  
  if (mpu.calibrado) 
  {
    Serial.print(mpu.angulo_x); Serial.print(",");
    Serial.print(mpu.angulo_y); Serial.print(",");
    Serial.print(mpu.angulo_z); Serial.print(",");
    Serial.print("  |  ");
    Serial.print(mpu.vel_x, 3); Serial.print(",");
    Serial.print(mpu.vel_y, 3); Serial.print(",");
    Serial.print(mpu.vel_z, 3); Serial.print(",");
    Serial.print("  |  ");
    Serial.print(altura_filtrada, 4); Serial.print(",");
    Serial.print(velocidade_filtrada, 4); Serial.print(",");
    Serial.print("  |  ");
    Serial.println(" ");
  }
}
