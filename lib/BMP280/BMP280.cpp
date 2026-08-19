#include "BMP280.h"

uint8_t BMP280::lerRegistrador(uint8_t endereco)
{
    digitalWrite(CS_BMP280, LOW);

        //no datasheet o bit 7 sempre está setado para 1 quando se quer ler um registrador
        SPI.transfer(endereco | 0x80);

        //enviar 0x00 para receber o valor do registrador
        uint8_t valor = SPI.transfer(0x00); 

    digitalWrite(CS_BMP280, HIGH);

    return valor;
}

void BMP280::escreverRegistrador(uint8_t endereco, uint8_t valor)
{
    digitalWrite(CS_BMP280, LOW);

    //no datasheet o bit 7 sempre está setado para 0 quando se quer escrever em um registrador
    SPI.transfer(endereco & 0x7F); 
    SPI.transfer(valor);

    digitalWrite(CS_BMP280, HIGH);
}

void BMP280::calibragem()
{
    dig_T1 = ((uint16_t)lerRegistrador(0x89) << 8) |
              lerRegistrador(0x88);

    dig_T2 = ((int16_t)lerRegistrador(0x8B) << 8) |
              lerRegistrador(0x8A);

    dig_T3 = ((int16_t)lerRegistrador(0x8D) << 8) |
              lerRegistrador(0x8C);

    dig_P1 = ((uint16_t)lerRegistrador(0x8F) << 8) |
              lerRegistrador(0x8E);

    dig_P2 = ((int16_t)lerRegistrador(0x91) << 8) |
              lerRegistrador(0x90);

    dig_P3 = ((int16_t)lerRegistrador(0x93) << 8) |
              lerRegistrador(0x92);

    dig_P4 = ((int16_t)lerRegistrador(0x95) << 8) |
              lerRegistrador(0x94);

    dig_P5 = ((int16_t)lerRegistrador(0x97) << 8) |
              lerRegistrador(0x96);

    dig_P6 = ((int16_t)lerRegistrador(0x99) << 8) |
              lerRegistrador(0x98);

    dig_P7 = ((int16_t)lerRegistrador(0x9B) << 8) |
              lerRegistrador(0x9A);

    dig_P8 = ((int16_t)lerRegistrador(0x9D) << 8) |
              lerRegistrador(0x9C);

    dig_P9 = ((int16_t)lerRegistrador(0x9F) << 8) |
              lerRegistrador(0x9E);
}

uint32_t BMP280::lerTemperaturaBRUTA()
{
    uint8_t a1, b1, c1;
    a1= lerRegistrador(TEMP_XLSB);
    b1= lerRegistrador(TEMP_LSB);
    c1= lerRegistrador(TEMP_MSB);

    //combinar os registradores para obter o valor de temperatura
    uint32_t resultadoTEMP = 
    ((uint32_t)c1 << 12) | 
    ((uint32_t)b1 << 4)  | 
    ((uint32_t)a1 >> 4);

    return resultadoTEMP;
}

uint32_t BMP280::lerPressaoBRUTA()
{
    uint32_t resultadoPRESS = 0;
    uint8_t a2, b2, c2;
    a2= lerRegistrador(PRESS_XLSB);
    b2= lerRegistrador(PRESS_LSB);
    c2= lerRegistrador(PRESS_MSB);
    
    //combinar os registradores para obter o valor de pressão
    resultadoPRESS = 
    ((uint32_t)c2 << 12) | 
    ((uint32_t)b2 << 4)  | 
    ((uint32_t)a2 >> 4); 

    return resultadoPRESS;
}

//BOSCH ALGORITHM, função para correção de valores de temperatura fornecido pelo datasheet do BMP280
float BMP280::lerTemperatura()
{
    double var1, var2;
    double adc_T = lerTemperaturaBRUTA();
    double T;

    var1 = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);

    var2 = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) * (((double)adc_T) / 131072.0 -
            ((double)dig_T1) / 8192.0)) *((double)dig_T3);

    t_compensado = (int32_t)(var1 + var2);

    T = (var1 + var2) / 5120.0;
    return (float)T;
}

//BOSCH ALGORITHM, função para correção de valores de pressão fornecido pelo datasheet do BMP280
float BMP280::lerPressao()
{
    int32_t adc_P = lerPressaoBRUTA();

    double var1;
    double var2;
    double p;

    var1 = ((double)t_compensado / 2.0) - 64000.0;

    var2 = var1 * var1 * ((double)dig_P6) / 32768.0;

    var2 = var2 + var1 * ((double)dig_P5) * 2.0;

    var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);

    var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;

    var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);

    if (var1 == 0.0)
        return 0; 
    
    p = 1048576.0 - (double)adc_P;

    p = (p - (var2 / 4096.0)) * 6250.0 / var1;

    var1 = ((double)dig_P9) * p * p / 2147483648.0;

    var2 = p * ((double)dig_P8) / 32768.0;

    p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;

    return (float)p;
}

bool BMP280::inicializar()
{
    //comunicação SPI
    pinMode(CS_BMP280, OUTPUT);
    digitalWrite(CS_BMP280, HIGH);

    SPI.begin();

    //ID
    if (lerRegistrador(ID_REG) != 0x58) 
        return false;     //incorreto mudar qnd tiver um tempo


    //calibragem
    calibragem(); 

    //CONFIGS
    escreverRegistrador(CTRL_MEAS, 0x2F); // 001 | 011 | 11 correspondem a oversampling de temperatura, pressão e modo de operação 
    escreverRegistrador(CONFIG, 0x00); // xxx yyy zz correspondem a standby time, filtro e 3-wire SPI  
    
        return true;
}