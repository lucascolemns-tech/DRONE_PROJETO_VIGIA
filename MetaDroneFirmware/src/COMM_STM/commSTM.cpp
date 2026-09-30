#include "commSTM.h"

static Uart SerialCom(PA10, PA9);

/*
lembrando que a comunicação UART é um registrador de deslocamento, então, ao salvarmos desta forma
condizemos com a forma que ela transmite os dados bit a bit.
*/
static inline void float_p_bin(float f, uint8_t* buf)
{
    memcpy(buf, &f, 4);
}

static inline float bin_p_float(const uint8_t* buf)
{
    float f;
    memcpy(&f, buf, 4);
    return f;
}

static uint16_t calcularCRC16(const uint8_t* dados, size_t tamanho)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < tamanho; i++)
    {
        crc ^= dados[i];
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

void commSTM::UART_init()
{
    SerialCom.begin(TAXA_UART);
    rx_state = 0;
    rx_idx = 0;
}

void commSTM::UART_enviar(float ax, float ay, float az, float alt,
                          float vel, float temp,
                          float m1, float m2, float m3, float m4,
                          float tensao, float gps_alt, bool sensores_ok,
                          float mag_status, float mag_progresso)
{
    uint8_t buf[BYTES_ENVIADOS];

    float_p_bin(ax,   &buf[0]);
    float_p_bin(ay,   &buf[4]);
    float_p_bin(az,   &buf[8]);
    float_p_bin(alt,  &buf[12]);
    float_p_bin(vel,  &buf[16]);
    float_p_bin(temp, &buf[20]);
    float_p_bin(m1,   &buf[24]);
    float_p_bin(m2,   &buf[28]);
    float_p_bin(m3,   &buf[32]);
    float_p_bin(m4,   &buf[36]);
    float_p_bin(tensao, &buf[40]);
    float_p_bin(gps_alt, &buf[44]);
    float_p_bin(sensores_ok ? 1.0f : 0.0f, &buf[48]);
    float_p_bin(mag_status, &buf[52]);
    float_p_bin(mag_progresso, &buf[56]);
    uint16_t crc = calcularCRC16(buf, BYTES_ENVIADOS - 2);
    buf[BYTES_ENVIADOS - 2] = (uint8_t)(crc & 0xFF);
    buf[BYTES_ENVIADOS - 1] = (uint8_t)(crc >> 8);
    
    //manda os indicadores e depois os dados
    SerialCom.write(HEADER_1);
    SerialCom.write(HEADER_2);
    SerialCom.write(buf, BYTES_ENVIADOS);
}


bool commSTM::UART_receber()
{
    while (SerialCom.available())
    {
        uint8_t b = SerialCom.read();
        unsigned long agora = micros();
        if (rx_state != 0 && agora - tempoUltimoByte > 5000)
        {
            rx_state = 0;
            rx_idx = 0;
        }
        tempoUltimoByte = agora;
        switch (rx_state)
        {
            //máquina de estados 
            case 0:
                if (b == HEADER_1) rx_state = 1;
                break;
            case 1:
                if (b == HEADER_2) { rx_state = 2; rx_idx = 0; } //se os dois primeiros casos forem ok roda o funcionamento
                else
                    rx_state = 0;
                break;
            case 2:
                rx_buf[rx_idx++] = b;
                if (rx_idx >= BYTES_RECEBIDOS)
                {
                    rx_state = 0;
                    uint16_t crc_recebido = (uint16_t)rx_buf[BYTES_RECEBIDOS - 2] |
                                            ((uint16_t)rx_buf[BYTES_RECEBIDOS - 1] << 8);
                    if (calcularCRC16(rx_buf, BYTES_RECEBIDOS - 2) != crc_recebido)
                        break;
                    comando[0] = bin_p_float(&rx_buf[0]);
                    comando[1] = bin_p_float(&rx_buf[4]);
                    comando[2] = bin_p_float(&rx_buf[8]);
                    comando[3] = bin_p_float(&rx_buf[12]);
                    comando[4] = bin_p_float(&rx_buf[16]);
                    comando[5] = bin_p_float(&rx_buf[20]);
                    tempoUltimoPacote = millis();
                    pronto = true;
                }
                break;
        }
    }
    if (pronto) { pronto = false; return true; }
    return false;
}
