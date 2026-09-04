//favor após checar retirar os comentários no github
//a modularização precisa ser o mesmo nome do arquivo, da erro de compilação
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

#endif

/*
COOOODIGOOOO ANTERIORRRRR:

#ifndef INTERNET_H
#define INTERNET_H

#include <Arduino.h>
#include <WiFi.h>

// Configurações

#define SERVER_PORT 1234

extern const char* rede;
extern const char* pass;

extern WiFiServer server;
extern WiFiClient cliente;

// IP fixo
extern IPAddress local_IP;
extern IPAddress gateway;
extern IPAddress subnet;

// Funções

void internet_init();

bool internet_conectado();

bool internet_novoCliente();

void internet_enviar(const char* dados);

String internet_receber();

#endif
*/