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
    unsigned long tempoRetryWifi = 0;
    uint32_t idSessao = 0;
};

#endif
