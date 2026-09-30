#include "INTERNET_ESP.h"

//Ajustar conforme necessidade
INTERNET_ESP_H::INTERNET_ESP_H()
     : rede("nunes"),          
      pass("12345678"),       
      local_IP(192, 168, 43, 132),   
      gateway(192, 168, 43, 1), 
      subnet(255, 255, 255, 0),       
      server(SERVER_PORT)
{}

void INTERNET_ESP_H::internet_init() 
{
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  if (!WiFi.config(local_IP, gateway, subnet))
    Serial.println("Falha ao configurar IP estatico");
  WiFi.begin(rede, pass);

  unsigned long comeco = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - comeco < 10000)
      delay(100); 
  
  wifiConectado = WiFi.status() == WL_CONNECTED;
  if (wifiConectado)
  {
    server.begin();
    Serial.println("\nWiFi conectado!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("FALHA CONECTAR WIFI");
  }

  tempoRetryWifi = millis();
  Serial.printf("Servidor TCP na porta %d\n", SERVER_PORT);
}

bool INTERNET_ESP_H::internet_conectado() 
{
    return WiFi.status() == WL_CONNECTED && cliente.connected(); 
}

void INTERNET_ESP_H::internet_manterConexao()
{
  bool conectado = WiFi.status() == WL_CONNECTED;
  if (conectado)
  {
    if (!wifiConectado)
    {
      wifiConectado = true;
      server.begin();
      Serial.print("WiFi reconectado. IP: ");
      Serial.println(WiFi.localIP());
    }
    return;
  }

  if (wifiConectado)
  {
    wifiConectado = false;
    if (cliente) cliente.stop();
    linhaIdx = 0;
    descartarLinha = false;
  }

  if (millis() - tempoRetryWifi >= 5000)
  {
    tempoRetryWifi = millis();
    WiFi.reconnect();
  }
}

void INTERNET_ESP_H::internet_verificarCliente() 
{
  internet_manterConexao();
  if (WiFi.status() != WL_CONNECTED)
    return;

  if (!internet_conectado())
  {
    if (cliente) cliente.stop(); 
    linhaIdx = 0;
    descartarLinha = false;
    cliente = server.available();
    if (cliente)
    {
      idSessao++;
      cliente.setNoDelay(true);
      Serial.println("Novo cliente ok");
    }
  }
}

void INTERNET_ESP_H::internet_enviar(const char* dados) 
{
  if (internet_conectado())
  {
    cliente.print(dados); 
    cliente.print("\n");
  }
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
  while (internet_conectado() && cliente.available())
  {
    char caractere = (char)cliente.read();

    if (caractere == '\n')
    {
      if (descartarLinha)
      {
        descartarLinha = false;
        linhaIdx = 0;
        return "";
      }

      linhaRx[linhaIdx] = '\0';
      String linha(linhaRx);
      linhaIdx = 0;
      return linha;
    }

    if (caractere == '\r' || descartarLinha)
      continue;

    if (linhaIdx < sizeof(linhaRx) - 1)
      linhaRx[linhaIdx++] = caractere;
    else
    {
      linhaIdx = 0;
      descartarLinha = true;
    }
  }

  return "";
}

