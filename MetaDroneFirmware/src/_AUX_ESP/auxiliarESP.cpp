#include "auxiliarESP.h"
#include <esc_range.h>

//checagem inicial do estado do WiFi e do cliente TCP
bool AUX_ESP::wifi_ok()
{
    return WiFi.status() == WL_CONNECTED;
}

bool AUX_ESP::cliente_ok()
{
    return internet.internet_conectado();
}

//comando do cliente válido e recente
bool AUX_ESP::comando_ok()
{
    return comandoClienteValido && cliente_ok();
}

//telemetria do STM32 válida e recente
bool AUX_ESP::telemetria_ok()
{
    return telemetriaValida && (millis() - tempoUltimaTelemetria <= TEMPO_MAX_TELEMETRIA);
}

//validações
int AUX_ESP::validarFaixaTelemetria(const float* dados)
{
    if (fabsf(dados[0]) > 360.0f) return 0;
    if (fabsf(dados[1]) > 360.0f) return 1;
    // O yaw pode cruzar +/-180 graus por erro de ponto flutuante ou vir em
    // 0..360 graus. Aceite uma volta completa; o valor será normalizado abaixo.
    if (fabsf(dados[2]) > 360.0f) return 2;
    if (fabsf(dados[3]) > 10000.0f) return 3;
    if (fabsf(dados[4]) > 10000.0f) return 4;
    if (dados[5] < -40.0f || dados[5] > 85.0f) return 5;
    if (dados[6] < ESC_MIN_US || dados[6] > ESC_MAX_US) return 6;
    if (dados[7] < ESC_MIN_US || dados[7] > ESC_MAX_US) return 7;
    if (dados[8] < ESC_MIN_US || dados[8] > ESC_MAX_US) return 8;
    if (dados[9] < ESC_MIN_US || dados[9] > ESC_MAX_US) return 9;
    if (dados[10] < -1.0f || dados[10] > 32.0f) return 10;
    if (fabsf(dados[11]) > 100000.0f) return 11;
    if (dados[12] != 0.0f && dados[12] != 1.0f) return 12;
    if (dados[13] < 0.0f || dados[13] > 5.0f || floorf(dados[13]) != dados[13]) return 13;
    if (dados[14] < 0.0f || dados[14] > 100.0f) return 14;
    return -1;
}

//função para extrair dados de uma string e armazena-los em registradores de memória
bool AUX_ESP::parseComando(String cmd) 
{
    if (cmd.length() == 0 || cmd.length() >= 96)
        return false; //se o tamanho não corresponde ao mínimo e máximo do array, retorna falso 

    //converte a string cmd para o array dados
    char dados[96];
    cmd.toCharArray(dados, sizeof(dados));

    float valores[4];
    char* cursor = dados;

    for (int i = 0; i < 4; i++)
    {
        char* fim = nullptr; //ponteiro que corresponde ao último caractere lido
        valores[i] = strtof(cursor, &fim); //percorre o array pelo "cursor" até o "fim"
        if (fim == cursor || !isfinite(valores[i])) //checa se os valores são reais
            return false;

        if (i < 3)
        {
            if (*fim != ',')
                return false;
            cursor = fim + 1;
        }
        else if (*fim != '\0')
            return false;
    }
    //checar se o valor do comando é válido
    if (fabsf(valores[0]) > 45.0f || fabsf(valores[1]) > 45.0f ||
        fabsf(valores[2]) > 180.0f || valores[3] < 0.0f || valores[3] > 1.0f)
        return false;

    //armazena nos membros
    roll_ref = valores[0];
    pitch_ref = valores[1];
    yaw_ref = valores[2];
    throttle_ref = valores[3];
    comandoClienteValido = true;
    return true;
}

bool AUX_ESP::parseCamposFloat(char* cursor, float* valores, uint8_t quantidade) //bytes enviados, cursor (percorre o arrya)
{
    for (uint8_t i = 0; i < quantidade; i++)
    {
        char* fim = nullptr;
        valores[i] = strtof(cursor, &fim); //convert to float
        if (fim == cursor || !isfinite(valores[i]))
            return false;
        if (i + 1 < quantidade)
        {
            if (*fim != ',')
                return false;
            cursor = fim + 1; //salva tudo após a "," para o próximo valor
        }
        else if (*fim != '\0') // chegou ao fim e não correspondeu ao caractér nulo, retorna falso
            return false;
    }
    return true;
}

bool AUX_ESP::parseConfiguracaoPID(String cmd)
{
    if (cmd.length() <= 4 || cmd.length() >= 96)
        return false;

    char dados[96];
    cmd.toCharArray(dados, sizeof(dados));
    if (strncmp(dados, "PID,", 4) != 0)
        return false; //salva dados PID em um array 

    float valores[3];
    if (!parseCamposFloat(dados + 4, valores, 3) || //válida kp, ki e kd
        valores[0] < 0.0f || valores[0] > 10.0f ||
        valores[1] < 0.0f || valores[1] > 1.0f ||
        valores[2] < 0.0f || valores[2] > 2.0f)
        return false;

    pidKpPendente = valores[0];
    pidKiPendente = valores[1];
    pidKdPendente = valores[2];
    pidConfiguracaoPendente = true; //permite ser mandado via TCP
    return true;
}

bool AUX_ESP::parseReferenciaGrafico(String cmd)
{
    if (cmd.length() <= 4 || cmd.length() >= 96)
        return false;

    char dados[96];
    cmd.toCharArray(dados, sizeof(dados));
    if (strncmp(dados, "REF,", 4) != 0)
        return false; //salva qual será a refêrencia em um array (constante, senoidal, quadrada ou triangular)

    char* modo = dados + 4;
    char* separador = strchr(modo, ',');
    if (separador == nullptr)
        return false;
    *separador = '\0';

    uint8_t novoModo;
    if (strcmp(modo, "CONST") == 0) novoModo = 0; //se a leitura do array for "const" define o modo
    else if (strcmp(modo, "SENO") == 0) novoModo = 1;
    else if (strcmp(modo, "QUAD") == 0) novoModo = 2;
    else if (strcmp(modo, "TRI") == 0) novoModo = 3;
    else return false; //se fugir do escopo retorna falso

    float valores[3];
    if (!parseCamposFloat(separador + 1, valores, 3) ||
        valores[1] < 0.5f || valores[1] > 300.0f ||
        fabsf(valores[0]) + fabsf(valores[2]) > 45.0f)
        return false;

    modoReferenciaGrafico = novoModo;
    amplitudeReferenciaGrafico = valores[0];
    periodoReferenciaGrafico = valores[1];
    offsetReferenciaGrafico = valores[2];
    inicioReferenciaGrafico = millis();
    referenciaGraficoAtiva = true;
    return true;
}

float AUX_ESP::valorReferenciaGrafico()
{
    if (!referenciaGraficoAtiva)
        return roll_ref;

    float fase = 6.28318530718f * ((millis() - inicioReferenciaGrafico) * 0.001f) /
                 periodoReferenciaGrafico;
    float forma = 0.0f;
    if (modoReferenciaGrafico == 1)
        forma = sinf(fase);
    else if (modoReferenciaGrafico == 2)
        forma = sinf(fase) >= 0.0f ? 1.0f : -1.0f;
    else if (modoReferenciaGrafico == 3)
        forma = 0.63661977236f * asinf(sinf(fase));
    return offsetReferenciaGrafico + amplitudeReferenciaGrafico * forma;
}

// ===================== etapas do loop =====================

// Zera os comandos quando o cliente TCP troca de sessão ou desconecta
void AUX_ESP::verificar_sessao()
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
        referenciaGraficoAtiva = false;
        temporizandoNeutralESC = false;
        controleESCArmado = false;
        esc.parar();
        for (int i = 0; i < 4; ++i)
            motoresESC_us[i] = ESC_MIN_US;
        ultimaSessaoCliente = sessao;
        clienteConectadoAnterior = cliente_conectado;
    }
}

// Lê uma linha do cliente TCP e decide o que fazer com ela
void AUX_ESP::processar_comando_tcp()
{
    String cmd = internet.internet_receber();
    cmd.trim();
    if (cmd.length() == 0)
        return;

    ++linhasTcpRecebidas;
    strncpy(ultimaLinhaTcp, cmd.c_str(), sizeof(ultimaLinhaTcp) - 1);
    ultimaLinhaTcp[sizeof(ultimaLinhaTcp) - 1] = '\0';
    bool aceito = false;
    if (cmd == "CAL_MAG") //comando para iniciar calibração do magnetometro, caso o cliente queira
    {
        calibracaoMagPendente = true;
        tempoPedidoCalibracaoMag = millis();
        statusMagNoPedido = telemetria[13];
        aceito = true;
    }
    else if (cmd == "REF_OFF")
    {
        referenciaGraficoAtiva = false;
        aceito = true;
    }
    else if (cmd.startsWith("PID,"))
        aceito = parseConfiguracaoPID(cmd);
    else if (cmd.startsWith("REF,"))
        aceito = parseReferenciaGrafico(cmd);
    else
    {
        aceito = parseComando(cmd);
    }
    if (aceito)
        ++linhasTcpAceitas;
    else
        ++linhasTcpRejeitadas;
}

void AUX_ESP::enviarComando()
{
    unsigned long agora = millis();
    if (agora - tempoEnvioComando < 20)
        return;
    tempoEnvioComando = agora;

    if (pidConfiguracaoPendente)
    {
        cspi.UART_enviarPID(pidKpPendente, pidKiPendente, pidKdPendente);
        pidConfiguracaoPendente = false;
        return;
    }

    bool controle_fresco = comando_ok();
    float throttle = throttle_ref;
    if (!controle_fresco)
    {
        roll_ref = 0.0f;
        pitch_ref = 0.0f;
        yaw_ref = 0.0f;
        throttle_ref = 0.0f;
        throttle = 0.0f;
    }

    ultimoComandoUart[0] = valorReferenciaGrafico();
    ultimoComandoUart[1] = pitch_ref;
    ultimoComandoUart[2] = yaw_ref;
    ultimoComandoUart[3] = throttle;
    ultimoComandoUartFresco = controle_fresco;
    cspi.UART_enviar(ultimoComandoUart[0], ultimoComandoUart[1], ultimoComandoUart[2],
                     ultimoComandoUart[3], controle_fresco, calibracaoMagPendente);
    ++framesComandoUart;
}

void AUX_ESP::atualizarESC()
{
    if (!esc.prontoParaControle() || !comando_ok() || !telemetria_ok())
    {
        esc.parar();
        temporizandoNeutralESC = false;
        controleESCArmado = false;
        for (int i = 0; i < 4; ++i)
            motoresESC_us[i] = ESC_MIN_US;
        return;
    }

    if (!controleESCArmado)
    {
        if (throttle_ref > 0.02f)
        {
            temporizandoNeutralESC = false;
            esc.parar();
            for (int i = 0; i < 4; ++i)
                motoresESC_us[i] = ESC_MIN_US;
            return;
        }

        if (!temporizandoNeutralESC)
        {
            tempoNeutralESC = millis();
            temporizandoNeutralESC = true;
        }

        if (millis() - tempoNeutralESC < 500)
        {
            esc.parar();
            for (int i = 0; i < 4; ++i)
                motoresESC_us[i] = ESC_MIN_US;
            return;
        }

        controleESCArmado = true;
    }

    for (int i = 0; i < 4; ++i)
        motoresESC_us[i] = constrain((int)lroundf(telemetria[6 + i]), ESC_MIN_US, ESC_MAX_US);
    esc.ESCRodar(motoresESC_us[0], motoresESC_us[1],
                 motoresESC_us[2], motoresESC_us[3]);
}

void AUX_ESP::processarTelemetria()
{
    if (!cspi.UART_receber()) //se não há frame novo, retorna
        return;

    //recebe valores do STM
    float novos_dados[15];
    for (int i = 0; i < 15; i++)
    {
        novos_dados[i] = cspi.UART_receber(i);
        ultimoFrameRecebido[i] = novos_dados[i];
    }
    houveFrameRecebido = true;

    for (int i = 0; i < 15; i++)
    {
        if (!isfinite(novos_dados[i])) //checa se todos os valores do array são finitos
        {
            telemetriaValida = false;
            telemetriasRejeitadas++;
            ultimoCampoInvalido = i;
            valorCampoInvalido = novos_dados[i];
            return;
        }
    }

    //checagem de segurança, para não enviar valores absurdos; corrige problemas durante a telemetria
    int campoInvalido = validarFaixaTelemetria(novos_dados);
    if (campoInvalido >= 0)
    {
        telemetriaValida = false;
        telemetriasRejeitadas++;
        ultimoCampoInvalido = campoInvalido;
        valorCampoInvalido = novos_dados[campoInvalido];
        return;
    }

    // Mantém o yaw no intervalo usado pelo painel e pelo controle, sem rejeitar
    // uma amostra válida que passou ligeiramente de +180/-180 graus.
    if (novos_dados[2] > 180.0f)
        novos_dados[2] -= 360.0f;
    else if (novos_dados[2] < -180.0f)
        novos_dados[2] += 360.0f;

    //se passou pela checagem, armazena os valores no array telemetria
    for (int i = 0; i < 15; i++)
        telemetria[i] = novos_dados[i];
    telemetriaValida = true;
    tempoUltimaTelemetria = millis();
    ultimoCampoInvalido = -1;
    valorCampoInvalido = 0.0f;

    //se houver pedido de calibração, o ESP32 precisa saber se a calibração foi aceita ou rejeitada
    if (calibracaoMagPendente && (telemetria[13] == 1.0f || ((telemetria[13] == 3.0f || telemetria[13] == 4.0f || telemetria[13] == 5.0f) && telemetria[13] != statusMagNoPedido) || millis() - tempoPedidoCalibracaoMag > 2000))
        calibracaoMagPendente = false;
}

void AUX_ESP::enviarTelemetriaWiFi()
{
    unsigned long agora = millis();
    // Dez linhas de telemetria por segundo; UART/controle continuam a 50 Hz.
    if (agora - tempoTelemetria < 100)
        return;
    tempoTelemetria = agora;

    //manda dados via WiFi
    char msg[250];
    const float motores1a4[4] = {
        (float)motoresESC_us[0], (float)motoresESC_us[1],
        (float)motoresESC_us[2], (float)motoresESC_us[3]};
    snprintf(msg, sizeof(msg),
             "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f,%.2f,%.2f,%d,%d,%.0f,%.1f",
             telemetria[0], telemetria[1], telemetria[2], telemetria[3],
             telemetria[4], telemetria[5], motores1a4[0], motores1a4[1],
             motores1a4[2], motores1a4[3], telemetria[10], telemetria[11],
             telemetria[12] == 1.0f ? 1 : 0,
             telemetria_ok() ? 1 : 0, //checar se a telemetria é válida e recente
             telemetria[13], telemetria[14]);

    unsigned long inicioEnvioUs = micros();
    internet.internet_enviar(msg);
    unsigned long duracaoEnvioUs = micros() - inicioEnvioUs;
    if (duracaoEnvioUs > tempoMaximoEnvioTcpUs)
        tempoMaximoEnvioTcpUs = duracaoEnvioUs;
}

// ===================== debug =====================
void AUX_ESP::debug_serial()
{
    static unsigned long tempoDebug = 0;
    static uint8_t etapaDebug = 0;
    static uint32_t bytesTcpAnteriores = 0;
    static uint32_t txWouldBlockAnterior = 0;
    static uint32_t bytesAnterior = 0;
    static uint32_t cabecalhosAnteriores = 0;
    static uint32_t framesAnteriores = 0;
    static uint32_t crcAnterior = 0;
    static uint32_t timeoutsAnteriores = 0;
    static uint32_t rejeicoesAnteriores = 0;
    static uint32_t linhasTcpAnteriores = 0;
    static uint32_t linhasTcpAceitasAnteriores = 0;
    static uint32_t linhasTcpRejeitadasAnteriores = 0;
    static uint32_t bytesTcpRecebidosAnteriores = 0;
    static uint32_t linhasTcpDescartadasAnteriores = 0;
    static uint32_t framesComandoUartAnteriores = 0;
    if (millis() - tempoDebug < 250)
        return;
    tempoDebug = millis();

    uint32_t bytesAtuais = cspi.UART_BytesRecebidos();
    uint32_t cabecalhosAtuais = cspi.UART_Cabecalhos();
    uint32_t framesAtuais = cspi.UART_Frames();
    uint32_t crcAtual = cspi.UART_CRCFailures();
    uint32_t timeoutsAtuais = cspi.UART_Timeouts();
    uint32_t bytesTcpRecebidosAtuais = internet.internet_bytesRecebidos();
    uint32_t linhasTcpDescartadasAtuais = internet.internet_linhasDescartadas();
    uint32_t bytesTcpAtuais = internet.internet_bytesTransmitidos();
    uint32_t txWouldBlockAtual = internet.internet_txWouldBlock();

    char mensagem[192];
    int tamanhoMensagem = 0;
    if (etapaDebug == 0)
    {
        tamanhoMensagem = snprintf(mensagem, sizeof(mensagem),
                                   "[ESP32 TCP RX] bytes/s=%lu linhas/s=%lu ok/s=%lu rejeitadas/s=%lu overflow/s=%lu ultimo=%s\n",
                                   (unsigned long)(bytesTcpRecebidosAtuais - bytesTcpRecebidosAnteriores),
                                   (unsigned long)(linhasTcpRecebidas - linhasTcpAnteriores),
                                   (unsigned long)(linhasTcpAceitas - linhasTcpAceitasAnteriores),
                                   (unsigned long)(linhasTcpRejeitadas - linhasTcpRejeitadasAnteriores),
                                   (unsigned long)(linhasTcpDescartadasAtuais - linhasTcpDescartadasAnteriores),
                                   ultimaLinhaTcp[0] ? ultimaLinhaTcp : "(nenhum)");
        bytesTcpRecebidosAnteriores = bytesTcpRecebidosAtuais;
        linhasTcpAnteriores = linhasTcpRecebidas;
        linhasTcpAceitasAnteriores = linhasTcpAceitas;
        linhasTcpRejeitadasAnteriores = linhasTcpRejeitadas;
        linhasTcpDescartadasAnteriores = linhasTcpDescartadasAtuais;
    }
    else if (etapaDebug == 1)
    {
        tamanhoMensagem = snprintf(mensagem, sizeof(mensagem),
                                   "[ESP32 UART TX] frames/s=%lu roll=%.2f pitch=%.2f yaw=%.2f throttle=%.3f fresh=%d esc8s=%d armed=%d pwm=%d,%d,%d,%d\n",
                                   (unsigned long)(framesComandoUart - framesComandoUartAnteriores),
                                   ultimoComandoUart[0], ultimoComandoUart[1],
                                   ultimoComandoUart[2], ultimoComandoUart[3],
                                   ultimoComandoUartFresco, esc.prontoParaControle(), controleESCArmado,
                                   motoresESC_us[0], motoresESC_us[1], motoresESC_us[2], motoresESC_us[3]);
        framesComandoUartAnteriores = framesComandoUart;
    }
    else if (etapaDebug == 2)
    {
        tamanhoMensagem = snprintf(mensagem, sizeof(mensagem),
                                   "[ESP32 UART RX] B=%lu H=%lu F=%lu CRC=%lu TO=%lu bad=%lu field=%d M=%.0f,%.0f,%.0f,%.0f\n",
                                   (unsigned long)(bytesAtuais - bytesAnterior),
                                   (unsigned long)(cabecalhosAtuais - cabecalhosAnteriores),
                                   (unsigned long)(framesAtuais - framesAnteriores),
                                   (unsigned long)(crcAtual - crcAnterior),
                                   (unsigned long)(timeoutsAtuais - timeoutsAnteriores),
                                   (unsigned long)(telemetriasRejeitadas - rejeicoesAnteriores),
                                   ultimoCampoInvalido,
                                   ultimoFrameRecebido[6], ultimoFrameRecebido[7],
                                   ultimoFrameRecebido[8], ultimoFrameRecebido[9]);
        bytesAnterior = bytesAtuais;
        cabecalhosAnteriores = cabecalhosAtuais;
        framesAnteriores = framesAtuais;
        crcAnterior = crcAtual;
        timeoutsAnteriores = timeoutsAtuais;
        rejeicoesAnteriores = telemetriasRejeitadas;
        houveFrameRecebido = false;
    }
    else
    {
        tamanhoMensagem = snprintf(mensagem, sizeof(mensagem),
                                   "[ESP32 TCP TX] wifi=%d rssi=%d client=%d bytes/s=%lu pending=%u eagain/s=%lu\n",
                                   wifi_ok(), WiFi.RSSI(),
                                   cliente_ok(),
                                   (unsigned long)(bytesTcpAtuais - bytesTcpAnteriores),
                                   (unsigned)internet.internet_txPendente(),
                                   (unsigned long)(txWouldBlockAtual - txWouldBlockAnterior));
        bytesTcpAnteriores = bytesTcpAtuais;
        txWouldBlockAnterior = txWouldBlockAtual;
        tempoMaximoLoopUs = 0;
        tempoMaximoEnvioTcpUs = 0;
    }
    etapaDebug = (etapaDebug + 1) % 4;
    if (tamanhoMensagem > 0 && tamanhoMensagem < (int)sizeof(mensagem) &&
        Serial.availableForWrite() >= tamanhoMensagem)
        Serial.write((const uint8_t*)mensagem, tamanhoMensagem);
}

// ===================== antigo setup() =====================
void AUX_ESP::iniciar()
{
    Serial.begin(115200);
    if (esc.armarESC())
        Serial.println("ESP32 ESC PWM started: 50Hz, 1000us minimum, arming for 8 seconds");
    else
        Serial.println("ESP32 ESC PWM attach failed; motors held at minimum");
    cspi.UART_init();
    internet.internet_init();
}

// ===================== antigo loop() =====================
void AUX_ESP::rodar()
{
    unsigned long inicioLoopUs = micros();

    verificar_sessao();
    processar_comando_tcp();
    atualizarESC();

    enviarComando();
    processarTelemetria();
    enviarTelemetriaWiFi();
    delay(1);

    unsigned long duracaoLoopUs = micros() - inicioLoopUs;
    if (duracaoLoopUs > tempoMaximoLoopUs)
        tempoMaximoLoopUs = duracaoLoopUs;
    // debug_serial() agora é chamado no main
}
