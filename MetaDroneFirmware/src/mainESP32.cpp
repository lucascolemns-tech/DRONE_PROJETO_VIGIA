#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <INTERNET_ESP.h>
#include <commESP.h>

INTERNET_ESP_H internet;
commESP cspi;

float roll_ref = 0.0f, pitch_ref = 0.0f, yaw_ref = 0.0f, throttle_ref = 0.0f;
float telemetria[15] = {0};
bool telemetriaValida = false;
unsigned long tempoUltimaTelemetria = 0;

//gerenciamento de tempo
unsigned long tempoEnvioComando = 0;
unsigned long tempoTelemetria = 0;
unsigned long tempoUltimoComando = 0;
bool comandoClienteValido = false;
bool clienteConectadoAnterior = false;
uint32_t ultimaSessaoCliente = 0;
bool calibracaoMagPendente = false;
unsigned long tempoPedidoCalibracaoMag = 0;
float statusMagNoPedido = 0.0f;

static const unsigned long TEMPO_MAX_COMANDO = 250;
static const unsigned long TEMPO_MAX_TELEMETRIA = 250;

//função não desenvolvida por mim. 
bool parseComando(String cmd)
{
    if (cmd.length() == 0 || cmd.length() >= 96)
        return false;

    //converte a string cmd para o array dados
    char dados[96];
    cmd.toCharArray(dados, sizeof(dados));

    float valores[4];
    char* cursor = dados;

    for (int i = 0; i < 4; i++)
    {
        char* fim = nullptr; //ponteiro que corresponde ao último caractere de um array
        valores[i] = strtof(cursor, &fim); //percorre o array pelo "cursor"  até o "fim"
        if (fim == cursor || !isfinite(valores[i])) //checa se os valores são reais
            return false;

        if (i < 3)
        {
            if (*fim != ',')
                return false;
            cursor = fim + 1; //percorrer enquanto não atingir o fim do array
        }
        else if (*fim != '\0')
            return false;
    }
    //checar se o valor do comando é válido
    if (fabsf(valores[0]) > 45.0f || fabsf(valores[1]) > 45.0f ||
        fabsf(valores[2]) > 180.0f || valores[3] < 0.0f || valores[3] > 1.0f)
        return false;

    //armazena nos arrays
    roll_ref = valores[0];
    pitch_ref = valores[1];
    yaw_ref = valores[2];
    throttle_ref = valores[3];
    tempoUltimoComando = millis(); //importante comparar dois valores para ver se o intervalo entre os comandos excedeu o limite
    comandoClienteValido = true;
    return true;
}

void enviarComando()
{
    unsigned long agora = millis();
    if (agora - tempoEnvioComando < 20)
        return;
    tempoEnvioComando = agora;

    bool controle_fresco = comandoClienteValido &&
                           (agora - tempoUltimoComando <= TEMPO_MAX_COMANDO);
    float throttle = throttle_ref;
    if (!controle_fresco)
    {
        roll_ref = 0.0f;
        pitch_ref = 0.0f;
        yaw_ref = 0.0f;
        throttle_ref = 0.0f;
        throttle = 0.0f;
    }

    cspi.UART_enviar(roll_ref, pitch_ref, yaw_ref, throttle, controle_fresco, calibracaoMagPendente);
}

void processarTelemetria()
{
    if (!cspi.UART_receber())
        return;

    //recebe valores do STM
    float novos_dados[15];
    for (int i = 0; i < 15; i++)
    {
        novos_dados[i] = cspi.UART_receber(i);
        if (!isfinite(novos_dados[i]))
        {
            telemetriaValida = false;
            return;
        }
    }

    if (fabsf(novos_dados[0]) > 360.0f || fabsf(novos_dados[1]) > 360.0f ||
        fabsf(novos_dados[2]) > 180.0f || fabsf(novos_dados[3]) > 10000.0f ||
        fabsf(novos_dados[4]) > 10000.0f || novos_dados[5] < -40.0f || novos_dados[5] > 85.0f ||
        novos_dados[6] < 0.0f || novos_dados[6] > 2500.0f ||
        novos_dados[7] < 0.0f || novos_dados[7] > 2500.0f ||
        novos_dados[8] < 0.0f || novos_dados[8] > 2500.0f ||
        novos_dados[9] < 0.0f || novos_dados[9] > 2500.0f ||
        novos_dados[10] < 0.0f || novos_dados[10] > 32.0f || fabsf(novos_dados[11]) > 100000.0f ||
        (novos_dados[12] != 0.0f && novos_dados[12] != 1.0f) ||
        novos_dados[13] < 0.0f || novos_dados[13] > 4.0f ||
        floorf(novos_dados[13]) != novos_dados[13] ||
        novos_dados[14] < 0.0f || novos_dados[14] > 100.0f)
    {
        telemetriaValida = false;
        return;
    }

    for (int i = 0; i < 15; i++)
        telemetria[i] = novos_dados[i];
    telemetriaValida = true;
    tempoUltimaTelemetria = millis();

    if (calibracaoMagPendente &&
        (telemetria[13] == 1.0f ||
         ((telemetria[13] == 3.0f || telemetria[13] == 4.0f) && telemetria[13] != statusMagNoPedido) ||
         millis() - tempoPedidoCalibracaoMag > 2000))
        calibracaoMagPendente = false;
}

void enviarTelemetriaWiFi()
{
    unsigned long agora = millis();
    if (agora - tempoTelemetria < 50)
        return;
    tempoTelemetria = agora;

    //manda dados via WiFi
    char msg[250];
    snprintf(msg, sizeof(msg),
             "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f,%.2f,%.2f,%d,%d,%.0f,%.1f",
             telemetria[0], telemetria[1], telemetria[2], telemetria[3],
             telemetria[4], telemetria[5], telemetria[6], telemetria[7],
             telemetria[8], telemetria[9], telemetria[10], telemetria[11],
             telemetria[12] == 1.0f ? 1 : 0,
             telemetriaValida && agora - tempoUltimaTelemetria <= TEMPO_MAX_TELEMETRIA ? 1 : 0,
             telemetria[13], telemetria[14]);

    internet.internet_enviar(msg);
}

void setup()
{
    Serial.begin(115200);
    cspi.UART_init();
    internet.internet_init();
}

void loop()
{
    internet.internet_verificarCliente();

    bool cliente_conectado = internet.internet_conectado();
    uint32_t sessao = internet.internet_sessao();
    if (sessao != ultimaSessaoCliente || cliente_conectado != clienteConectadoAnterior)
    {
        roll_ref = 0.0f;
        pitch_ref = 0.0f;
        yaw_ref = 0.0f;
        throttle_ref = 0.0f;
        comandoClienteValido = false;
        tempoUltimoComando = 0;
        ultimaSessaoCliente = sessao;
        clienteConectadoAnterior = cliente_conectado;
    }

    String cmd = internet.internet_receber();
    cmd.trim();
    if (cmd == "CAL_MAG")
    {
        calibracaoMagPendente = true;
        tempoPedidoCalibracaoMag = millis();
        statusMagNoPedido = telemetria[13];
    }
    else if (cmd.length() > 0 && !parseComando(cmd))
        comandoClienteValido = false;

    enviarComando();
    processarTelemetria();
    enviarTelemetriaWiFi();
    delay(1);
}
