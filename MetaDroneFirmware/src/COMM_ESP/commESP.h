#ifndef COMM_ESP_H
#define COMM_ESP_H

#include <Arduino.h>

#define TAXA_UART 115200
#define HEADER_1 0xAA
#define HEADER_2 0x55
#define BYTES_ENVIADOS 26 //32 bytes são um float
#define BYTES_RECEBIDOS 62
#define UART_TX_PIN 17
#define UART_RX_PIN 16

class commESP
{
public:
    void UART_init();
    bool UART_receber();
    void UART_enviar(float roll, float pitch, float yaw, float throttle, bool controle_valido, bool calibrar_mag);
    void UART_enviarPID(float kp, float ki, float kd);
    float UART_receber(int idx) const { return (idx >= 0 && idx < 15) ? telemetria[idx] : 0.0f; }
    uint32_t UART_BytesRecebidos() const { return bytesRecebidos; }
    uint32_t UART_Cabecalhos() const { return cabecalhosRecebidos; }
    uint32_t UART_Frames() const { return framesRecebidos; }
    uint32_t UART_CRCFailures() const { return falhasCRC; }
    uint32_t UART_Timeouts() const { return timeoutsRecepcao; }

private:
    //idx é o index percorrido, state autoexplicativo, buf tamanho do dado total
    uint8_t rx_state = 0;
    uint8_t rx_idx = 0; 
    unsigned long tempoUltimoByte = 0;
    uint8_t rx_buf[BYTES_RECEBIDOS];
    float telemetria[15] = {0};
    bool pronto = false;
    uint32_t bytesRecebidos = 0;
    uint32_t cabecalhosRecebidos = 0;
    uint32_t framesRecebidos = 0;
    uint32_t falhasCRC = 0;
    uint32_t timeoutsRecepcao = 0;
};

#endif
