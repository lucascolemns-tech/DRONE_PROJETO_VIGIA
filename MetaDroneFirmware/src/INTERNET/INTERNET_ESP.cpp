#include "INTERNET_ESP.h"
#include <errno.h>
#include <string.h>
#include <lwip/sockets.h>

//Ajustar conforme necessidade
INTERNET_ESP_H::INTERNET_ESP_H()
     : rede("Colem"),          
      pass("lucas2007"),       
      local_IP(10, 200, 79, 132), // deve corresponder ao IP usado pela interface TCP
      gateway(10, 200, 79, 29),
      subnet(255, 255, 255, 0),
      server(SERVER_PORT)
{}

void INTERNET_ESP_H::internet_init() 
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  if (!WiFi.config(local_IP, gateway, subnet))
  {
    return;
  }

  WiFi.begin(rede, pass);

  unsigned long comeco = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - comeco < 10000)
      delay(100); 
  
  wifiConectado = WiFi.status() == WL_CONNECTED;
  if (wifiConectado)
  {
    server.begin();
  }

  tempoRetryWifi = millis();
}

bool INTERNET_ESP_H::internet_conectado() 
{
    return WiFi.status() == WL_CONNECTED && clienteAtivo;
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
    }
    return;
  }

  if (wifiConectado)
  {
    wifiConectado = false;
    cliente.stop();
    clienteAtivo = false;
    linhaIdx = 0;
    descartarLinha = false;
    tamanhoTx = 0;
    offsetTx = 0;
    linhasTxAcumuladas = 0;
    txPronto = false;
    tempoUltimoProgressoTx = 0;
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

  // Flush pending TCP bytes without blocking the UART/flight-control loop.
  internet_tentarEnviar();

  // Accept a pending reconnect even if a half-open older TCP peer still
  // reports connected(). Otherwise telemetry can keep going to the stale peer.
  if (server.hasClient())
  {
    WiFiClient novoCliente = server.available();
    if (!novoCliente)
      return;

    if (clienteAtivo && cliente.connected() && millis() - ultimoRxTcpMs <= 1000)
    {
      novoCliente.stop();
      return;
    }

    cliente.stop();
    cliente = novoCliente;
    cliente.setNoDelay(true);
    clienteAtivo = true;
    linhaIdx = 0;
    descartarLinha = false;
    tamanhoTx = 0;
    offsetTx = 0;
    linhasTxAcumuladas = 0;
    txPronto = false;
    tempoUltimoProgressoTx = 0;
    ultimoRxTcpMs = millis();
    ++idSessao;
    return;
  }

  if (clienteAtivo && !cliente.connected())
  {
    cliente.stop();
    clienteAtivo = false;
    linhaIdx = 0;
    descartarLinha = false;
    tamanhoTx = 0;
    offsetTx = 0;
    linhasTxAcumuladas = 0;
    txPronto = false;
    tempoUltimoProgressoTx = 0;
  }
}

void INTERNET_ESP_H::internet_enviar(const char* dados) 
{
  if (!internet_conectado() || dados == nullptr)
    return;

  size_t tamanho = strlen(dados);
  if (tamanho + 1 > sizeof(bufferTx) / 2)
  {
    cliente.stop();
    clienteAtivo = false;
    tamanhoTx = 0;
    offsetTx = 0;
    linhasTxAcumuladas = 0;
    txPronto = false;
    tempoUltimoProgressoTx = 0;
    return;
  }

  // Send two complete rows together. Small individual TCP writes can fill the
  // ESP32 socket queue while the peer delays ACKs. Keep the UART/control loop
  // non-blocking and retain any partial batch until it can be completed.
  if (txPronto)
  {
    internet_tentarEnviar();
    if (txPronto)
      return; // Never place a new row behind a partially-sent TCP frame.
  }

  const size_t tamanhoLinha = tamanho + 1;
  if (tamanhoTx + tamanhoLinha > sizeof(bufferTx))
  {
    tamanhoTx = 0;
    offsetTx = 0;
    linhasTxAcumuladas = 0;
  }
  memcpy(bufferTx + tamanhoTx, dados, tamanho);
  bufferTx[tamanhoTx + tamanho] = '\n';
  tamanhoTx += tamanhoLinha;
  if (++linhasTxAcumuladas >= 2)
  {
    linhasTxAcumuladas = 0;
    offsetTx = 0;
    txPronto = true;
    tempoUltimoProgressoTx = millis();
    internet_tentarEnviar();
  }
}

void INTERNET_ESP_H::internet_tentarEnviar()
{
  if (!clienteAtivo || !txPronto || tamanhoTx <= offsetTx)
    return;

  // A TCP socket can remain logically connected after Wi-Fi stops delivering
  // acknowledgements. Close it after one second with no send progress so the
  // Python receiver sees EOF and reconnects instead of waiting on stale data.
  if (millis() - tempoUltimoProgressoTx > 750)
  {
    cliente.stop();
    clienteAtivo = false;
    tamanhoTx = 0;
    offsetTx = 0;
    linhasTxAcumuladas = 0;
    txPronto = false;
    tempoUltimoProgressoTx = 0;
    return;
  }

  const int socketFd = cliente.fd();
  if (socketFd < 0)
  {
    cliente.stop();
    clienteAtivo = false;
    tamanhoTx = 0;
    offsetTx = 0;
    linhasTxAcumuladas = 0;
    txPronto = false;
    tempoUltimoProgressoTx = 0;
    return;
  }

  const size_t restante = tamanhoTx - offsetTx;
  const int enviados = send(socketFd, bufferTx + offsetTx, restante, MSG_DONTWAIT);
  if (enviados > 0)
  {
    offsetTx += static_cast<size_t>(enviados);
    tempoUltimoProgressoTx = millis();
    bytesTransmitidos += static_cast<uint32_t>(enviados);
    if (offsetTx == tamanhoTx)
    {
      tamanhoTx = 0;
      offsetTx = 0;
      txPronto = false;
      tempoUltimoProgressoTx = 0;
    }
    return;
  }

  const int erroEnvio = enviados < 0 ? errno : 0;
  if (enviados < 0 && (erroEnvio == EAGAIN || erroEnvio == EWOULDBLOCK))
  {
    ++txWouldBlock;
    return;
  }
  if (enviados < 0 && erroEnvio == EINTR)
    return;

  if (enviados == 0 || enviados < 0)
  {
    cliente.stop();
    clienteAtivo = false;
    tamanhoTx = 0;
    offsetTx = 0;
    linhasTxAcumuladas = 0;
    txPronto = false;
    tempoUltimoProgressoTx = 0;
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
    ++bytesRecebidos;
    ultimoRxTcpMs = millis();

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
      ++linhasDescartadas;
    }
  }

  return "";
}
