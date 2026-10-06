#include <commESP.h>
#include <Arduino.h>

static HardwareSerial SerialCom(2);

//converte dentro da clase um valor de float para binarios (em arrays)
static inline void float_p_bin(float f, uint8_t* buf)
{
    memcpy(buf, &f, 4);
}

//o contrario puxa 4 array de 8 bits e transforma em float, para ser usado na função de receber
static inline float bin_p_float(const uint8_t* buf)
{
    float f;
    memcpy(&f, buf, 4);
    return f;
}

//forma de identificar erros na comunicação via Internet
static uint16_t calcularCRC16(const uint8_t* dados, size_t tamanho)
{ //CRC-16 ou chamado de modbus é um algoritmo de verificação de erros que é usado para detectar alterações acidentais em dados digitais. Ele é amplamente utilizado em protocolos de comunicação
    uint16_t crc = 0xFFFF; //constante incial 1111, 1111, 1111, 1111
    for (size_t i = 0; i < tamanho; i++)
    {
        crc ^= dados[i]; //xor de cada bit do dado, e depois checa a uma imagem da saída
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001) 
                crc = (crc >> 1) ^ 0xA001; //se o bit menos significativo for 1, desloca para a direita e aplica o polinômio
            else 
                crc >>= 1;         
        }
    }
    return crc;
}

void commESP::UART_init() //incialização comm uart
{
    SerialCom.begin(TAXA_UART, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    rx_state = 0; //estado do "receiver"
    rx_idx = 0; //index (do array) que o UART está mandando
}

void commESP::UART_enviar(float roll, float pitch, float yaw, float throttle, bool controle_valido, bool calibrar_mag)
{
    uint8_t buf[BYTES_ENVIADOS];

    //para ser possível armazenar os dados correspondentes 
    float_p_bin(roll, &buf[0]);
    float_p_bin(pitch, &buf[4]);
    float_p_bin(yaw, &buf[8]);
    float_p_bin(throttle, &buf[12]);
    float_p_bin(controle_valido ? 1.0f : 0.0f, &buf[16]);
    float_p_bin(calibrar_mag ? 1.0f : 0.0f, &buf[20]);
    uint16_t crc = calcularCRC16(buf, BYTES_ENVIADOS - 2);
    buf[BYTES_ENVIADOS - 2] = (uint8_t)(crc & 0xFF);
    buf[BYTES_ENVIADOS - 1] = (uint8_t)(crc >> 8);

    //manda o indicador e depois os bytes
    SerialCom.write(HEADER_1);
    SerialCom.write(HEADER_2);
    SerialCom.write(buf, BYTES_ENVIADOS); //para ser checado do outro lado UART
}

void commESP::UART_enviarPID(float kp, float ki, float kd)
{
    uint8_t buf[BYTES_ENVIADOS];

    float_p_bin(kp, &buf[0]);
    float_p_bin(ki, &buf[4]);
    float_p_bin(kd, &buf[8]);
    float_p_bin(0.0f, &buf[12]);
    float_p_bin(2.0f, &buf[16]);
    float_p_bin(0.0f, &buf[20]);
    uint16_t crc = calcularCRC16(buf, BYTES_ENVIADOS - 2);
    buf[BYTES_ENVIADOS - 2] = (uint8_t)(crc & 0xFF);
    buf[BYTES_ENVIADOS - 1] = (uint8_t)(crc >> 8);

    SerialCom.write(HEADER_1);
    SerialCom.write(HEADER_2);
    SerialCom.write(buf, BYTES_ENVIADOS);
}

bool commESP::UART_receber()
{
    while (SerialCom.available())
    {
        uint8_t leitura = SerialCom.read();
        bytesRecebidos++; //percorrendo
        unsigned long agora = micros();
        tempoUltimoByte = agora;
        //vai pegar o estado do "receiver", checar se chegou os indicadores corretos, se os dois chegarem avança e percorre o index
        switch (rx_state)
        {
            case 0:
                if (leitura == HEADER_1) rx_state = 1;
                break;
            case 1:
                if (leitura == HEADER_2) { rx_state = 2; rx_idx = 0; cabecalhosRecebidos++; }
                else rx_state = leitura == HEADER_1 ? 1 : 0;
                break;
            case 2:
                rx_buf[rx_idx++] = leitura; //começa a percorrer o buffer 
                if (rx_idx >= BYTES_RECEBIDOS)
                {
                    rx_state = 0;
                    uint16_t crc_recebido = (uint16_t)rx_buf[BYTES_RECEBIDOS - 2] |
                                            ((uint16_t)rx_buf[BYTES_RECEBIDOS - 1] << 8);
                    if (calcularCRC16(rx_buf, BYTES_RECEBIDOS - 2) != crc_recebido) 
                    { falhasCRC++; break; }

                    for (int i = 0; i < 15; i++)
                        telemetria[i] = bin_p_float(&rx_buf[i * 4]); //copia a partir de onde o ponteiro indicar os 4 valores anteriores armazenados
                    framesRecebidos++; //repete isto para quantos frames chegaram
                    pronto = true;
                }
                break;
        }
    }
    if (rx_state != 0 && !SerialCom.available() && micros() - tempoUltimoByte > 10000)
    {
        rx_state = 0;
        rx_idx = 0;
        timeoutsRecepcao++;
    }
    if (pronto) { pronto = false; return true; }
    return false;

}
