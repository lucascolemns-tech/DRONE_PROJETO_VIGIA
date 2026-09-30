#include <commESP.h>
#include <Arduino.h>

static HardwareSerial SerialCom(2);

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

void commESP::UART_init()
{
    SerialCom.begin(TAXA_UART, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    rx_state = 0;
    rx_idx = 0;
}

void commESP::UART_enviar(float roll, float pitch, float yaw, float throttle, bool controle_valido, bool calibrar_mag)
{
    uint8_t buf[BYTES_ENVIADOS];

    //para ser possível armazenar os dados correspondentes 
    float_p_bin(roll,     &buf[0]);
    float_p_bin(pitch,    &buf[4]);
    float_p_bin(yaw,      &buf[8]);
    float_p_bin(throttle, &buf[12]);
    float_p_bin(controle_valido ? 1.0f : 0.0f, &buf[16]);
    float_p_bin(calibrar_mag ? 1.0f : 0.0f, &buf[20]);
    uint16_t crc = calcularCRC16(buf, BYTES_ENVIADOS - 2);
    buf[BYTES_ENVIADOS - 2] = (uint8_t)(crc & 0xFF);
    buf[BYTES_ENVIADOS - 1] = (uint8_t)(crc >> 8);

    //manda o indicador e depois os bytes
    SerialCom.write(HEADER_1);
    SerialCom.write(HEADER_2);
    SerialCom.write(buf, BYTES_ENVIADOS);
}

bool commESP::UART_receber()
{
    while (SerialCom.available())
    {
        uint8_t leitura = SerialCom.read();
        unsigned long agora = micros();
        if (rx_state != 0 && agora - tempoUltimoByte > 5000)
        {
            rx_state = 0;
            rx_idx = 0;
        }
        tempoUltimoByte = agora;
        //vai pegar o estado do "receiver", checar se chegou os indicadores corretos, se os dois chegarem avança e percorre o index
        switch (rx_state)
        {
            case 0:
                if (leitura == HEADER_1) rx_state = 1;
                break;
            case 1:
                if (leitura == HEADER_2) { rx_state = 2; rx_idx = 0; }
                else rx_state = 0;
                break;
            case 2:
                rx_buf[rx_idx++] = leitura; //começa a percorrer o buffer 
                if (rx_idx >= BYTES_RECEBIDOS)
                {
                    rx_state = 0;
                    uint16_t crc_recebido = (uint16_t)rx_buf[BYTES_RECEBIDOS - 2] |
                                            ((uint16_t)rx_buf[BYTES_RECEBIDOS - 1] << 8);
                    if (calcularCRC16(rx_buf, BYTES_RECEBIDOS - 2) != crc_recebido)
                        break;
                    for (int i = 0; i < 15; i++)
                        telemetria[i] = bin_p_float(&rx_buf[i * 4]); //copia a partir de onde o ponteiro indicar os 4 valores anteriores armazenados
                    pronto = true;
                }
                break;
        }
    }
    if (pronto) { pronto = false; return true; }
    return false;

}
