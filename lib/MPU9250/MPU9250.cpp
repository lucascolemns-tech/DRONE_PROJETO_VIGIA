#include "MPU9250.h"

const double pi = 3.1415926535;

float Time_ = 0.0;

uint8_t MPU9250::lerRegistrador(uint8_t endereco) 
{
    digitalWrite(CS_MPU, LOW);

    SPI.transfer(endereco | 0x80); 
    uint8_t valor = SPI.transfer(0x00); 

    digitalWrite(CS_MPU, HIGH);

    return valor;
}

void MPU9250::escreverRegistrador(uint8_t endereco, uint8_t valor) 
{
    digitalWrite(CS_MPU, LOW);

    SPI.transfer(endereco & 0x7F); // Set the MSB to 0 for writing
    SPI.transfer(valor);

    digitalWrite(CS_MPU, HIGH);
}

uint8_t MPU9250::lerRegistradorI2C(uint8_t endereco, uint8_t registrador) 
{
    Wire.beginTransmission(endereco);
    Wire.write(registrador);

    Wire.endTransmission(false); 
    
    Wire.requestFrom(endereco, 1);

    if (Wire.available()) 
        return Wire.read();

    return 0xFF;
}

void MPU9250::lerAcelerometroGiroscopio() 
{
    digitalWrite(CS_MPU, LOW);

    SPI.transfer(0x3B | 0x80); 

    for (uint8_t endereco = 0x3B; endereco <= 0x48; endereco++) 
    {
        uint8_t valor = SPI.transfer(0x00); 

        switch(endereco) 
        {
            case 0x3B: ACC_X_H  =  valor; break;
            case 0x3C: ACC_X_L  =  valor; break;
            case 0x3D: ACC_Y_H  =  valor; break;
            case 0x3E: ACC_Y_L  =  valor; break;
            case 0x3F: ACC_Z_H  =  valor; break;
            case 0x40: ACC_Z_L  =  valor; break;
            case 0x41: TEMP_H   =  valor; break;
            case 0x42: TEMP_L   =  valor; break;
            case 0x43: GYRO_X_H =  valor; break;
            case 0x44: GYRO_X_L =  valor; break;
            case 0x45: GYRO_Y_H =  valor; break;
            case 0x46: GYRO_Y_L =  valor; break;
            case 0x47: GYRO_Z_H =  valor; break;
            case 0x48: GYRO_Z_L =  valor; break;
        }

    digitalWrite(CS_MPU, HIGH);
    }

    //construir os valores de 16 bits a partir dos registradores de 8 bits
    ACC_X = (ACC_X_H << 8) | ACC_X_L;
    ACC_Y = (ACC_Y_H << 8) | ACC_Y_L;
    ACC_Z = (ACC_Z_H << 8) | ACC_Z_L;

    TEMP = (TEMP_H << 8) | TEMP_L;

    GYRO_X = (GYRO_X_H << 8) | GYRO_X_L;
    GYRO_Y = (GYRO_Y_H << 8) | GYRO_Y_L;
    GYRO_Z = (GYRO_Z_H << 8) | GYRO_Z_L;

    digitalWrite(CS_MPU, HIGH);
}

void MPU9250::lerMagnetometro()
{
    //checar estado de partida a partir do primeiro bit do registrador ST1 do magnetometro
    if (!lerRegistradorI2C(AK8963_ADDRESS, MAG_ST1) & 0x01) 
        return;

    Wire.beginTransmission(AK8963_ADDRESS);

    Wire.write(MAG_MX); //primeiro registrador do magnetometro

    Wire.endTransmission(false);
    
    Wire.requestFrom(AK8963_ADDRESS, 6);

    if (Wire.available() < 6)
        return;

    mx_L = Wire.read();
    mx_H = Wire.read();
    my_L = Wire.read();
    my_H = Wire.read();
    mz_L = Wire.read();
    mz_H = Wire.read();

    mx = mx_H | (mx_H<< 8); 
    my = my_H | (my_H << 8);
    mz = mz_H | (mz_H << 8);
 
    Wire.endTransmission();
}

bool MPU9250::inicializar() 
{
    //SPI config
    pinMode(CS_MPU, OUTPUT);
    digitalWrite(CS_MPU, HIGH);

    SPI.begin();

    if (lerRegistrador(WHO_AM_I_MPU) != MPU_ID)
        return false;

    //acelerometro config
    digitalWrite(CS_MPU, LOW);

    SPI.transfer(ACCEL_CONFIG);
    SPI.transfer(0x00);

    digitalWrite(CS_MPU, HIGH);

    //giroscopio config
    digitalWrite(CS_MPU, LOW);

    SPI.transfer(GYRO_CONFIG);
    SPI.transfer(0x00);

    digitalWrite(CS_MPU, HIGH);
    
    //I2C config
    Wire.begin();

    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(INT_PIN_CFG); 
    Wire.write(0x02); 

    Wire.endTransmission();

    //magnetometro config
    if (lerRegistradorI2C(AK8963_ADDRESS, WHO_AM_I_AK89630) != AK8963_ID)
        return false;

    Wire.beginTransmission(AK8963_ADDRESS);
    
    Wire.write(AK8963_CNTL1);
    Wire.write(0x16); 

    Wire.endTransmission();

    return true;
}

void MPU9250::Sistemacoordenadas(
  float ax, float ay, float az,
  float roll, float pitch, float yaw,
  float &world_x, float &world_y, float &world_z) //PS: "float &world_xyz mantém o valor anterior salvo nela mesma
    {
        float cr = cos(roll);
        float sr = sin(roll);

        float cp = cos(pitch);
        float sp = sin(pitch);

        float cy = cos(yaw);
        float sy = sin(yaw);

        world_x = (cy*cp)*ax
            + (cy*sp*sr - sy*cr)*ay
            + (cy*sp*cr + sy*sr)*az;

        world_y = (sy*cp)*ax
            + (sy*sp*sr + cy*cr)*ay
            + (sy*sp*cr - cy*sr)*az;

        world_z = (-sp)*ax
            + (cp*sr)*ay
            + (cp*cr)*az;
    }

void MPU9250::MPUcalculos()
{
    static unsigned long tempo_antes = micros();
    unsigned long tempo_depois = micros();
    float tempo = (tempo_depois - tempo_antes)/1000000.0;
    tempo_antes = tempo_depois;

    float Converted_ACC_X = 9.81*ACC_X/16384.0; 
    float Converted_ACC_Y = 9.81*ACC_Y/16384.0; 
    float Converted_ACC_Z = 9.81*ACC_Z/16384.0;

    float Converted_TEMP = (TEMP/333.87) + 21;

    float Converted_GYRO_X = GYRO_X/131.0; 
    float Converted_GYRO_Y = GYRO_Y/131.0; 
    float Converted_GYRO_Z = GYRO_Z/131.0;

    angle_x += Converted_GYRO_X * tempo;
    angle_y += Converted_GYRO_Y * tempo;
    angle_z += Converted_GYRO_Z * tempo;

    pitch= atan2(-Converted_ACC_X, sqrt(pow(Converted_ACC_Y,2) + pow(Converted_ACC_Z,2)));
    roll= atan2(Converted_ACC_Y, sqrt(pow(Converted_ACC_X,2) + pow(Converted_ACC_Z,2)));

    roll = (180 / pi) * roll;
    pitch = (180 / pi) * pitch;

    angle_x = 0.98*(angle_x) + roll * 0.02;
    angle_y = 0.98*(angle_y) + pitch * 0.02;

    float angle_x_rad = angle_x * pi / 180;
    float angle_y_rad = angle_y * pi / 180;

    //magnetometro
    float mx_comp = mx * cos(angle_y_rad) + mz * sin(angle_y_rad);
    float my_comp = mx * sin(angle_x_rad) * sin(angle_y_rad) + my * cos(angle_x_rad) - mz * sin(angle_x_rad) * cos(angle_y_rad);
    
    yaw = atan2(my_comp, mx_comp);
    yaw = yaw * 180.0 / pi;

    angle_z = 0.98*(angle_z) + yaw*0.02;
    float angle_z_rad = angle_z*pi/180;

    roll  = angle_x_rad;
    pitch = angle_y_rad;
    yaw   = angle_z_rad;

    //gravidade em cada eixo
    float gx = -9.81f * sin(pitch);
    float gy =  9.81f * sin(roll) * cos(pitch);
    float gz =  9.81f * cos(roll) * cos(pitch);

    //sem a relação a gravidade-
    float linear_ax = Converted_ACC_X - gx;
    float linear_ay = Converted_ACC_Y - gy;
    float linear_az = Converted_ACC_Z - gz;

    //relativo à terra
    float ACClinear_x_World;
    float ACClinear_y_World;
    float ACClinear_z_World;

    Sistemacoordenadas
    (linear_ax,        linear_ay,         linear_az,
    roll,              pitch,             yaw, 
    ACClinear_x_World, ACClinear_y_World, ACClinear_z_World);

    vel_x += ACClinear_x_World*Time_;
    vel_y += ACClinear_y_World*Time_;
    vel_z += ACClinear_z_World*Time_;

    pos_x += vel_x * Time_;
    pos_y += vel_y * Time_;
    pos_z += vel_z * Time_;
}


