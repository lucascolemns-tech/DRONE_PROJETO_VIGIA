#ifndef INTERNET_ESP_CLASS_H
#define INTERNET_ESP_CLASS_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>

#define SERVER_PORT 1244

class INTERNET_ESP_H {
public:
    INTERNET_ESP_H();

    void internet_init();
    void internet_manterConexao();
    bool internet_conectado();
    uint32_t internet_sessao() const { return idSessao; }
    uint32_t internet_bytesRecebidos() const { return bytesRecebidos; }
    uint32_t internet_linhasDescartadas() const { return linhasDescartadas; }
    uint32_t internet_bytesTransmitidos() const { return bytesTransmitidos; }
    uint32_t internet_txWouldBlock() const { return txWouldBlock; }
    size_t internet_txPendente() const { return txPronto ? tamanhoTx - offsetTx : 0; }
    void internet_verificarCliente();
    void internet_enviar(const char* dados);
    void internet_enviar(float valor);
    void internet_enviar(float v1, float v2);
    void internet_enviar(float v1, float v2, float v3);
    String internet_receber();

private:
    const char* rede;
    const char* pass;
    IPAddress local_IP;
    IPAddress gateway;
    IPAddress subnet;

    WiFiServer server;
    WiFiClient cliente;
    char linhaRx[96] = {};
    uint8_t linhaIdx = 0;
    bool descartarLinha = false;
    bool wifiConectado = false;
    bool clienteAtivo = false;
    unsigned long tempoRetryWifi = 0;
    uint32_t idSessao = 0;
    uint32_t bytesRecebidos = 0;
    uint32_t linhasDescartadas = 0;
    uint32_t bytesTransmitidos = 0;
    uint32_t txWouldBlock = 0;
    unsigned long ultimoRxTcpMs = 0;
    char bufferTx[1024] = {};
    size_t tamanhoTx = 0;
    size_t offsetTx = 0;
    uint8_t linhasTxAcumuladas = 0;
    bool txPronto = false;
    unsigned long tempoUltimoProgressoTx = 0;

    void internet_tentarEnviar();
};

#endif
