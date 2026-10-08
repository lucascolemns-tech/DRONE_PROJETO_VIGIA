#ifndef AUX_ESP_H
#define AUX_ESP_H

#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ESC.h>
#include <INTERNET_ESP.h>
#include <commESP.h>

class AUX_ESP
{
public:
    void iniciar();       
    void rodar();        
    void debug_serial(); 

    //flags
    bool wifi_ok();        // WiFi conectado
    bool cliente_ok();     // cliente TCP (interface Python) conectado
    bool comando_ok();     // comando válido na sessão TCP ativa
    bool telemetria_ok();  // última telemetria do STM32 é válida e tem menos de 250 ms

    //etapas do loop
    void verificar_sessao();
    void processar_comando_tcp();
    void enviarComando();
    void atualizarESC();
    void processarTelemetria();
    void enviarTelemetriaWiFi();

private:
    //validações
    static int  validarFaixaTelemetria(const float* dados); // não usa estado
    static bool parseCamposFloat(char* cursor, float* valores, uint8_t quantidade); // não usa estado
    //parsers (você "cortar os dados" da string e armazenar em registradores de memória)
    bool parseComando(String cmd);
    bool parseConfiguracaoPID(String cmd);
    bool parseReferenciaGrafico(String cmd);
    float valorReferenciaGrafico();

    //construtoresbibliotecas
    INTERNET_ESP_H internet;
    commESP cspi;
    ESC_ESP32 esc;

    //comandos e telemetria
    float roll_ref = 0.0f, pitch_ref = 0.0f, yaw_ref = 0.0f, throttle_ref = 0.0f;
    int motoresESC_us[4] = {ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US};
    unsigned long tempoNeutralESC = 0;
    bool temporizandoNeutralESC = false;
    bool controleESCArmado = false;
    float telemetria[15] = {0};
    float ultimoFrameRecebido[15] = {0};
    unsigned long tempoUltimaTelemetria = 0;
    bool houveFrameRecebido = false;
    bool telemetriaValida = false;

    //gerenciamento de tempo
    unsigned long tempoEnvioComando = 0;
    unsigned long tempoTelemetria = 0;
    unsigned long tempoPedidoCalibracaoMag = 0;
    unsigned long tempoMaximoLoopUs = 0;
    unsigned long tempoMaximoEnvioTcpUs = 0;
    
    //variáveis globais
    static constexpr unsigned long TEMPO_MAX_TELEMETRIA = 250;

   
    uint32_t ultimaSessaoCliente = 0;
    uint32_t telemetriasRejeitadas = 0;
    uint32_t linhasTcpRecebidas = 0;
    uint32_t linhasTcpAceitas = 0;
    uint32_t linhasTcpRejeitadas = 0;
    uint32_t framesComandoUart = 0;
    int ultimoCampoInvalido = -1;
    float valorCampoInvalido = 0.0f;
    float statusMagNoPedido = 0.0f;
    bool comandoClienteValido = false;
    bool clienteConectadoAnterior = false;
    char ultimaLinhaTcp[96] = {};
    float ultimoComandoUart[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool ultimoComandoUartFresco = false;
    bool calibracaoMagPendente = false;
    bool pidConfiguracaoPendente = false;
    float pidKpPendente = 0.0f, pidKiPendente = 0.0f, pidKdPendente = 0.0f;
    bool referenciaGraficoAtiva = false;
    uint8_t modoReferenciaGrafico = 0;
    float amplitudeReferenciaGrafico = 0.0f, periodoReferenciaGrafico = 1.0f, offsetReferenciaGrafico = 0.0f;
    unsigned long inicioReferenciaGrafico = 0;
};

#endif
