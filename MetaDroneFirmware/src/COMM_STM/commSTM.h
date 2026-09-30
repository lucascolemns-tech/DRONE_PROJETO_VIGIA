#ifndef COMM_STM_H
#define COMM_STM_H

#include <Arduino.h>

#define TAXA_UART 115200
#define HEADER_1 0xAA
#define HEADER_2 0x55
#define BYTES_RECEBIDOS 26
#define BYTES_ENVIADOS 62
#define TEMPO_MAX 200

class commSTM
{
public:
    void UART_init();
    bool UART_receber();
    void UART_enviar(float ax, float ay, float az, float alt,
                     float vel, float temp,
                     float m1, float m2, float m3, float m4,
                     float tensao, float gps_alt, bool sensores_ok,
                     float mag_status, float mag_progresso);
    float UART_receber(int idx) const { return (idx >= 0 && idx < 6) ? comando[idx] : 0.0f; }
    unsigned long UART_TempoUltimoPacote() const { return tempoUltimoPacote; }

private:
    uint8_t rx_state = 0;
    uint8_t rx_idx = 0;
    unsigned long tempoUltimoByte = 0;
    uint8_t rx_buf[BYTES_RECEBIDOS];
    float comando[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    bool pronto = false;
    unsigned long tempoUltimoPacote = 0;
};

#endif
