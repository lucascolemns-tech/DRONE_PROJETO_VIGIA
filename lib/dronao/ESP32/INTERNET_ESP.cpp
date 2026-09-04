/*
mudanças feitas dia 1/9/2026
Oi Sábio, dei uma mexida no seu código, da uma olhada e vê se você concorda porfavor
A versão original vai estar comentada tanto no arquivo.cpp quanto no .h
//favor após checar retirar os comentários no github
*/

#include "INTERNET_ESP.h"

//Ajustar conforme necessidade
INTERNET_ESP_H::INTERNET_ESP_H()
     : rede("BL_BRUNO_ADV"),          
      pass("Lucas2007**"),       
      local_IP(192, 168, 18, 201),       
      gateway(192, 168, 18, 1), 
      subnet(255, 255, 255, 0),       
      server(SERVER_PORT)
{}

void INTERNET_ESP_H::internet_init() 
{
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(rede, pass);

  unsigned long comeco = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - comeco < 10000)
      delay(100); //dar um tempo para conectar com o WiFi
  
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi conectado!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }
  else
    Serial.println("FALHA CONECTAR WIFI");

  server.begin();
  Serial.printf("Servidor TCP na porta %d\n", SERVER_PORT);
}

bool INTERNET_ESP_H::internet_conectado() 
{
    return cliente.connected(); //cliente&& não funciona ele não não retorna um valor bool
}

void INTERNET_ESP_H::internet_verificarCliente() 
{
  if (!internet_conectado())
  {
    if (cliente) cliente.stop(); //remove se havia um antigo
      cliente = server.available();
    if (cliente)
      Serial.println("Novo cliente ok");
  }
}

void INTERNET_ESP_H::internet_enviar(const char* dados) 
{
  if (internet_conectado()) 
    cliente.print(dados); // Trocado de println para print
    cliente.print("\n");   // Adiciona apenas a quebra de linha simples
}

void INTERNET_ESP_H::internet_enviar(float dados)
{
  if (internet_conectado()) 
    cliente.printf("%f\n", dados);
}

void INTERNET_ESP_H::internet_enviar(float dados1, float dados2) 
{
  if (internet_conectado()) 
    cliente.printf("%f,%f\n", dados1, dados2);
}

/* 
Oi sábio, sei que isso nem necessário é kk, mas da uma lida nisto, você escreveu:
"The way choosed to send data is a more easy way to python decode it."

Dicas pra melhorar a frase:
The way "chosen" to send data is an easier way for python to decode it.

PS: "to python decode it não está errado", mas me soa estranho
pois: você "more easy way" isso indica que algo é mais fácil, assim
a preposição mais ideal seria for (para o) do que to (para)

choose | chose | chosen ... chosed não existe!


No entanto até com essa correção fica estranho, pois chosen para ter um 
sentido mais completo na lingua inglesa precisa de um complemento posterior

A forma que eu escreveria seria:

The way *that || was chosen || to send data is an easier way for python to decode
*/ 

void INTERNET_ESP_H::internet_enviar(float gx, float gy, float gz) 
{
  if (internet_conectado())
    cliente.printf("%f;%f;%f;\n",gx,gy,gz);
}

String INTERNET_ESP_H::internet_receber() 
{
  if (internet_conectado() && cliente.available()) 
    return cliente.readStringUntil('\n');

    return "";
}

/*
COOOODIGOOOO ANTERIORRRRR:

#include "internet.h"

const char* rede = "IPHONE";
const char* pass = "messi2015";

IPAddress local_IP(192, 168, 43, 201);
IPAddress gateway(192, 168, 43, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiServer server(SERVER_PORT);
WiFiClient cliente;
void internet_init() {

    WiFi.config(local_IP, gateway, subnet);

    WiFi.begin(rede, pass);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    server.begin();
}
bool internet_conectado() {

    return cliente && cliente.connected();
}
void internet_novoCliente() {

    if (!internet_conectado()) {
        cliente = server.available();
    }
}
void internet_enviar(float dados) {

    if (internet_conectado()) {
        cliente.println(dados);
    }
}
void internet_enviar(float dados1, float dados2) {
    if (internet_conectado()) {
        cliente.printf(
            "%f,%f\n",
            dados1,
            dados2
        );
    }
}
// The way choosed to send data is a more easy way to python decode it.
void internet_enviar(float gx, float gy, float gz) {
    if (internet_conectado()) {
        cliente.printf("%f;%f;%f;\n",gx,gy,gz);
    }
}
String internet_receber() {
    if (internet_conectado() && cliente.available()) {
        return cliente.readStringUntil('\n');
    }
    return "";
}
*/
/*
COOOODIGOOOO ANTERIORRRRR:

#include "internet.h"

const char* rede = "IPHONE";
const char* pass = "messi2015";

IPAddress local_IP(192, 168, 43, 201);
IPAddress gateway(192, 168, 43, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiServer server(SERVER_PORT);
WiFiClient cliente;
void internet_init() {

    WiFi.config(local_IP, gateway, subnet);

    WiFi.begin(rede, pass);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    server.begin();
}
bool internet_conectado() {

    return cliente && cliente.connected();
}
void internet_novoCliente() {

    if (!internet_conectado()) {
        cliente = server.available();
    }
}
void internet_enviar(float dados) {

    if (internet_conectado()) {
        cliente.println(dados);
    }
}
void internet_enviar(float dados1, float dados2) {
    if (internet_conectado()) {
        cliente.printf(
            "%f,%f\n",
            dados1,
            dados2
        );
    }
}
// The way choosed to send data is a more easy way to python decode it.
void internet_enviar(float gx, float gy, float gz) {
    if (internet_conectado()) {
        cliente.printf("%f;%f;%f;\n",gx,gy,gz);
    }
}
String internet_receber() {
    if (internet_conectado() && cliente.available()) {
        return cliente.readStringUntil('\n');
    }
    return "";
}


*/