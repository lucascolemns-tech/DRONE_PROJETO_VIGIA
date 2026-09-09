#ifndef INTERNET_ESP_CLASS_H   
#define INTERNET_ESP_CLASS_H

#include <Arduino.h>
#include <WiFi.h>

#define SERVER_PORT 1244

class INTERNET_ESP_H {
public:
    INTERNET_ESP_H();

    void internet_init();
    bool internet_conectado();
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
};
