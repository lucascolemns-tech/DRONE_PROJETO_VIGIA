// 1/9/2026

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
