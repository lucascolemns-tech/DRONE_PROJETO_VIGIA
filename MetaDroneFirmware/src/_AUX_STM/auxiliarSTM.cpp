#include "auxiliarSTM.h"

//incialização dos construtores
AUX_STM::AUX_STM() 
    : bmp(0x77),
      kf(0.1f, 0.5f),
      SerialGPS(PA_12, PA_11),
      gps(SerialGPS)
{}

void AUX_STM::indicarEtapaSetup(uint8_t etapa)
{
    #if defined(STM32_DEBUG_UART)
        if (Serial && Serial.availableForWrite() >= 32) 
        { Serial.print("etapa: "); Serial.println(etapa); }
    #endif
    digitalWrite(LED_BUILTIN, (etapa % 2) ? LOW : HIGH); //debug visual do setup
}

float AUX_STM::altitude_barometrica_relativa(float pressao_pa)
{
    if (!isfinite(pressao_pa) || pressao_pa <= 0.0f ||
        !isfinite(pressao_referencia_pa) || pressao_referencia_pa <= 0.0f)
        return NAN;
    return 44330.0f * (1.0f - powf(pressao_pa / pressao_referencia_pa, 0.1903f));
}

//módulo opcional inicialização: magnetometro
bool AUX_STM::mag_requisito_ok()
{
    #if USAR_MAG
        return mag_pronto;
    #else
        return true;
    #endif
}

bool AUX_STM::mag_voo_ok()
{
    #if USAR_MAG
        return mag_pronto && millis() - tempo_mag_valido <= 250;
    #else
        return true;
    #endif
}

bool AUX_STM::mag_calibrando()
{
    #if USAR_MAG
        return magnetometro.getCalibrando();
    #else
        return false;
    #endif
}

//checa modulos essenciais
bool AUX_STM::pode_ficar_pronto()
{
    return esc_pronto && mpu_pronto && bmp_pronto && mag_requisito_ok();
}

//i2c começo
void AUX_STM::i2c_init()
{
    Wire.setSDA(PB7);
    Wire.setSCL(PB6);
    Wire.begin();
    Wire.setClock(400000);
    //debug via uart, adicione outro cabo usb ao stm32
    #if defined(STM32_DEBUG_UART)
        Serial.println("I2C pins: SDA=PB7, SCL=PB6, clock=400kHz");
    #endif
}

//bmp incializador padrão
bool AUX_STM::bmp_init()
{
    bmp.pressao = 0.0f;
    bmp.temperatura = 0.0f;
    if (!bmp.inicializar())
        return false;

    unsigned long inicio = millis();
    while (millis() - inicio < 2000)
    {
        bmp.lerBMP();
        if (isfinite(bmp.pressao) && bmp.pressao > 0.0f) //checa por valores válido do bmp
            return true; //o kalman necessita de um valor inicial
        delay(20);
    }
    return false;
}

void AUX_STM::sensores_init() //inicialização dos sensores
{
    //módulo essencial: mpu
    mpu_pronto = mpu.inicializar() && mpu.calibrarMPU(); // MPU6500 usa SPI, para rodar em frequência máxima
    #if defined(STM32_DEBUG_UART)
        Serial.print("MPU6500: "); Serial.println(mpu_pronto ? "OK" : "FALHA");
    #endif

    //módulo essencial: bmp
    bmp_pronto = bmp_init();
    #if defined(STM32_DEBUG_UART)
        Serial.print("BMP180 em 0x"); Serial.print(bmp.endereco(), HEX);
        Serial.print(": "); Serial.println(bmp_pronto ? "OK" : "FALHA");
    #endif

    //módulo opcional: vl53l0x
    #if USAR_DISTSENSOR
        dist_pronto = distsensor.VL53L0X_init(); 
        #if defined(STM32_DEBUG_UART)
            Serial.print("VL53L0X: "); Serial.println(dist_pronto ? "OK" : "FALHA");
        #endif
    #else 
        #if defined(STM32_DEBUG_UART)
            Serial.println("VL53L0X: DESABILITADO");
        #endif
    #endif

    //módulo opcional: neo6m
    #if USAR_GPS
        gps.init();
    #else
        #if defined(STM32_DEBUG_UART)
            Serial.println("NEO6M: DESABILITADO");
        #endif
    #endif

    //módulo opcional: magnetômetro
    #if USAR_MAG
        mag_pronto = magnetometro.mag_init();
        #if defined(STM32_DEBUG_UART)
            Serial.print("HMC5883L: "); Serial.println(mag_pronto ? "OK" : "FALHA");
        #endif
    #else
        #if defined(STM32_DEBUG_UART)
            Serial.println("HMC5883L: DESABILITADO");
        #endif
    #endif

    //checa se pode inicalizar o kalman
    if (bmp_pronto)
    {
        // Captura a pressão no solo como referência para altitude relativa ao ponto de partida.
        if (pressao_referencia_pa <= 0.0f) pressao_referencia_pa = bmp.pressao;
        kf.reset(0.0f, 0.0f); //mostra 0 na telemetria e varia de acordo com a mudança de pressão
    }
    estado = pode_ficar_pronto() ? PRONTO : DESLIGADO; //retorna se passou todas as verificações
}

//configuração base PID, kp, ki e kd
void AUX_STM::pid_init()
{
    pid.Config(5.0, 0.8, 0.03, //definições iniciais PID
               5.0, 0.8, 0.03,
               3.0, 0.5, 0.01,
               0.0, 0.0, 0.0);
    pid.SetPeriod(10);
    pid.SetBaseThrottle(ESC_MIN_US); //manter ESC_MIN_US dentro de 1000 +- 5%
    pid.Setpoint(0.0, 0.0, 0.0, 0.0); //inicial zerado
}

void AUX_STM::sensores_ler()
{
    //checamos o millis para controlar o intervalo entre leituras
    unsigned long agora_ms = millis();
    bool mag_amostra_nova = false;

    #if USAR_MAG
        static uint8_t falhas_mag = 0; //permite contagem das falhas admissíveis
        unsigned long agora_us = micros();
        if (agora_us - tempo_mag_us >= 20000) //leitura do magnetometro a cada 20 ms
        {
            tempo_mag_us = agora_us;
            mag_amostra_valida = magnetometro.mag_ler();
            mag_amostra_nova = mag_amostra_valida; //temos uma amostra nova
            if (mag_amostra_valida)
            {
                falhas_mag = 0;
                tempo_mag_valido = millis();
            }
            else if (++falhas_mag >= 5 && millis() - tempo_mag_valido > 250)
                mag_pronto = false; //5 falhas seguidas e sem amostra válida há 250 ms
        }
    #endif

    //mpu roda sempre que possível via SPI
    if (mpu_pronto)
    {
        if (mpu.lerMPU())
        {
            tempo_mpu_valido = millis();
            //getXYZ retornam sempre 0 se não houver leitura, se a config do mpu for sem mag, esses valores são descartados
            mpu.MPUcalculos(magnetometro.getX(), magnetometro.getY(), magnetometro.getZ(),
                            USAR_MAG && mag_amostra_nova && magnetometro.getCalibrado());
        }
        else if (millis() - tempo_mpu_valido > 50) //se exceder, o mpu não está em estado ideal
            mpu_pronto = false;
    }

    //roda a cada 10ms via i2c
    if (bmp_pronto && (agora_ms - tempoBMP >= 10))
    {
        tempoBMP = agora_ms;
        if (!bmp.lerBMP() && bmp.idadeAmostraMs() > 250)
            bmp_pronto = false;
    }

    //roda a cada 50ms via i2c
    #if USAR_DISTSENSOR
        if (dist_pronto && agora_ms - tempoRange >= 50)
        {
            tempoRange = agora_ms;
            distancia_mm = distsensor.VL53L0X_read();
        }
    #endif
    //gps roda sempre que possível via UART
    #if USAR_GPS
        gps.atualizar();
    #endif
}

void AUX_STM::kalman_atualizar(float dt)
{
    if (estado != VOANDO) //comparamos com os estados do drone
    {
        kf.setAlt(0.0f); //se estiver parado setamos tudo em 0
        kf.setVel(0.0f);
    }

    //sem sensores válidos não altera nenhuma variável
    if (!mpu_pronto || !bmp_pronto || !isfinite(bmp.pressao) || bmp.pressao <= 0.0f)
        return;

    kf.CHUTE(mpu.filtro_z, dt); //intervalo ao qual o kalman roda

    //gerenciar tempo do Kalman
    static unsigned long tempoBaroAnterior = 0;
    unsigned long tempoAmostra = bmp.tempoAmostraMs();
    if (tempoAmostra != 0 && tempoAmostra != tempoBaroAnterior)
    {
        if (!pressao_filtrada_pronta)
        {
            pressao_filtrada_pa = bmp.pressao;
            pressao_referencia_pa = bmp.pressao;
            pressao_filtrada_pronta = true;
        }
        else
            //constante alpha para suavizar a pressão filtrada, evita picos de leitura do bmp
            pressao_filtrada_pa += FILTRO_PRESSAO_ALPHA * (bmp.pressao - pressao_filtrada_pa);

        if (estado != VOANDO)
        {
            if (!isfinite(pressao_referencia_pa) || pressao_referencia_pa <= 0.0f) //checa se a referência do solo é válida
                pressao_referencia_pa = pressao_filtrada_pa;
            else
                //atualiza a referência no solo enquanto ainda recebe leituras válidas
                pressao_referencia_pa += FILTRO_REFERENCIA_SOLO_ALPHA * (pressao_filtrada_pa - pressao_referencia_pa);
        }

        float baro_alt = altitude_barometrica_relativa(pressao_filtrada_pa);
        if (isfinite(baro_alt))
            kf.atualizarKALMAN(baro_alt);
        tempoBaroAnterior = tempoAmostra; //tempo entre leituras
    }
    if (estado != VOANDO)
    {
        kf.setAlt(0.0f);
        kf.setVel(0.0f);
    }
}

//começo da comunicação UART, stm32 p esp32 e vice-versa
void AUX_STM::comm_receber()
{
    if (!cstm.UART_receber())
        return;

    float roll = cstm.UART_receber(0);
    float pitch = cstm.UART_receber(1);
    float yaw = cstm.UART_receber(2);
    float throttle = cstm.UART_receber(3);
    bool controle_fresco = cstm.UART_receber(4) == 1.0f; //controle sem movimentos bruscos, permite calibração
    float calibrar_mag = cstm.UART_receber(5); //envio comando de calibração

    if (cstm.UART_receber(4) == 2.0f)
    {
        if (isfinite(roll) && isfinite(pitch) && isfinite(yaw) &&
            roll >= 0.0f && roll <= 10.0f &&
            pitch >= 0.0f && pitch <= 1.0f &&
            yaw >= 0.0f && yaw <= 2.0f &&
            estado != VOANDO && throttle_ref <= 0.02f) //valores válidos e stick do throttle embaixo
        {
            pid.Config(roll, yaw, pitch,
                       roll, yaw, pitch,
                       roll, yaw, pitch,
                       0.0, 0.0, 0.0);
            pid.Reset();
        }
        return;
    }

    //pedido para poder calibrar magnetometro
    if (!isfinite(calibrar_mag) || (calibrar_mag != 0.0f && calibrar_mag != 1.0f))
    {
        comando_valido = false;
        return;
    }

    #if USAR_MAG
        bool pedido_calibracao = calibrar_mag == 1.0f;
        if (pedido_calibracao && !calibracao_mag_anterior)
        {
            bool controle_neutro_valido = controle_fresco && isfinite(roll) && isfinite(pitch) && isfinite(yaw) &&
                                        isfinite(throttle) && fabsf(roll) <= 2.0f && fabsf(pitch) <= 2.0f &&
                                        fabsf(yaw) <= 2.0f && throttle >= 0.0f && throttle <= 0.02f; //checa se controle está parado
            if (estado != VOANDO && controle_neutro_valido) //somente dessa forma pode aceitar calibração
            {
                if (!magnetometro.iniciarCalibracao())
                    magnetometro.rejeitarCalibracao();
            }
            else
                magnetometro.rejeitarCalibracao();
        }
        calibracao_mag_anterior = pedido_calibracao;
    #endif

    //isfinite checa se o valor é "real", fabsf retorna valor absoluto, checa antes de aceitar os comandos
    if (!controle_fresco || !isfinite(roll) || !isfinite(pitch) || !isfinite(yaw) || !isfinite(throttle) ||
        fabsf(roll) > 45.0f || fabsf(pitch) > 45.0f || fabsf(yaw) > 180.0f ||
        throttle < 0.0f || throttle > 1.0f)
    {
        comando_valido = false;
        throttle_ref = 0.0f;
        temporizando_neutro = false;
        neutro_armar_confirmado = false; //não pode armar se os controles retornam movimentos bruscos
        return;
    }

    //se passar na checagem
    roll_ref = roll;
    pitch_ref = pitch;
    yaw_ref = yaw;
    throttle_ref = throttle;
    tempoComando = millis();
    comando_valido = true;

    //durante a calibração os controles precisam estar parados, senão ocorre erro que se mantém por todo o funcionamento
    bool controles_neutros = fabsf(roll_ref) <= 2.0f && fabsf(pitch_ref) <= 2.0f && fabsf(yaw_ref) <= 2.0f;
    if (estado == PRONTO && !mag_calibrando() && throttle_ref <= 0.02f && controles_neutros)
    {
        if (!temporizando_neutro)
        {
            tempo_neutro = millis();
            temporizando_neutro = true;
        }
        //controles parados por 500 ms: pode armar
        if (millis() - tempo_neutro >= 500)
            neutro_armar_confirmado = true;
    }
    else if (estado == PRONTO && !mag_calibrando() && throttle_ref > 0.02f)
    {
        temporizando_neutro = false;
        //essenciais + módulos ligados (cada um responde por si)
        if (neutro_armar_confirmado && pode_ficar_pronto() &&
            (!USAR_MAG || mag_amostra_valida))
        {
            estado = VOANDO;
            pressao_referencia_pa = pressao_filtrada_pronta ? pressao_filtrada_pa : bmp.pressao;
            kf.reset(0.0f, 0.0f);
            yaw_alvo = mpu.angulo_z; //sem mag, com o tempo o drift fica alto
            neutro_armar_confirmado = false;
            pid.Reset();
        }
    }
    else
    {
        temporizando_neutro = false;
        neutro_armar_confirmado = false;
    }
}

bool AUX_STM::verificar_queda()
{
    bool link_caiu = !comando_valido || millis() - tempoComando > TEMPO_MAX_COMANDO;
    bool sensores_falharam = !mpu_pronto || millis() - tempo_mpu_valido > 50 ||
                             !bmp_pronto || bmp.idadeAmostraMs() > 250;
    bool magnetometro_falhou = !mag_voo_ok();

    if (estado == VOANDO && (link_caiu || sensores_falharam || magnetometro_falhou || throttle_ref <= 0.02f))
    {
        estado = PRONTO;
        throttle_ref = 0.0f;
        temporizando_neutro = false;
        neutro_armar_confirmado = false;
        pid.Reset();
    }
    if (link_caiu || sensores_falharam || magnetometro_falhou ||
        estado != VOANDO || throttle_ref <= 0.02f)
    {
        esc.ESCRodar(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
        throttle_atual_us = ESC_MIN_US;
        motoresSaidaUs[0] = ESC_MIN_US;
        motoresSaidaUs[1] = ESC_MIN_US;
        motoresSaidaUs[2] = ESC_MIN_US;
        motoresSaidaUs[3] = ESC_MIN_US;
        return true;
    }
    return false;
}
void AUX_STM::motores_escrever()
{
    unsigned long agora = millis();
    unsigned long intervalo = agora - tempoPID;
    if (intervalo < 10)
        return;
    tempoPID = agora;

    float dt = intervalo * 0.001f;
    if (dt <= 0.0f || dt > 0.1f)
        dt = 0.01f;

    //converte o throttle (0 a 1) para microsegundos do ESC
    float throttle_alvo = ESC_MIN_US + throttle_ref * (ESC_MAX_US - ESC_MIN_US);
    float max_delta = TAXA_VARIACAO_THROTTLE_US_S * dt;
    if (throttle_alvo > throttle_atual_us + max_delta)
        throttle_atual_us += max_delta; //limita a taxa de variação para suavizar picos
    else if (throttle_alvo < throttle_atual_us - max_delta)
        throttle_atual_us -= max_delta;

    pid.SetBaseThrottle(throttle_atual_us);
    yaw_alvo += yaw_ref * dt; //integra yaw
    while (yaw_alvo > 180.0f) yaw_alvo -= 360.0f; //mantém o yaw dentro de -180 a 180, senão cresceria infinitamente
    while (yaw_alvo < -180.0f) yaw_alvo += 360.0f;
    float yaw_setpoint = yaw_alvo;
    float erro_yaw = yaw_setpoint - mpu.angulo_z;
    if (erro_yaw > 180.0f) yaw_setpoint -= 360.0f; //normaliza o erro de yaw para o caminho mais curto
    else if (erro_yaw < -180.0f) yaw_setpoint += 360.0f;
    pid.Setpoint(roll_ref, pitch_ref, yaw_setpoint, 0.0);
    pid.Input(mpu.angulo_x, mpu.angulo_y, mpu.angulo_z, kf.getAlt());
    pid.RunPID(true, true, true, false);

    motoresSaidaUs[0] = constrain((int)pid.GetM1(), ESC_MIN_US, ESC_MAX_US);
    motoresSaidaUs[1] = constrain((int)pid.GetM2(), ESC_MIN_US, ESC_MAX_US);
    motoresSaidaUs[2] = constrain((int)pid.GetM3(), ESC_MIN_US, ESC_MAX_US);
    motoresSaidaUs[3] = constrain((int)pid.GetM4(), ESC_MIN_US, ESC_MAX_US);
    esc.ESCRodar(motoresSaidaUs[0], motoresSaidaUs[1],
                 motoresSaidaUs[2], motoresSaidaUs[3]);
}

void AUX_STM::comm_enviar()
{
    //envio telemetria a cada 20ms
    if (millis() - tempoEnvio < 20)
        return;
    tempoEnvio = millis();
    bool sensores_ok = pode_ficar_pronto() && mag_voo_ok();

    //altitude do GPS (para comparação): 0 se o GPS estiver desligado ou sem fix
    float gps_alt = 0.0f;

    #if USAR_GPS
        if (gps.checar())
            gps_alt = (float)gps.obter_alt();
    #endif

    //valores que vão ser enviados na telemetria
    float mag_status    = 5.0f;  // 5 = magnetômetro desabilitado no firmware
    float mag_progresso = 0.0f; //responsável pela "barra" de progresso da calibração

    #if USAR_MAG
        mag_status    = magnetometro.getStatus();
        mag_progresso = magnetometro.getProgresso();
    #endif

    cstm.UART_enviar(
        mpu.angulo_x, mpu.angulo_y, mpu.angulo_z,                               
        kf.getAlt(), kf.getVel(), bmp.temperatura,              
        (float)motoresSaidaUs[0], (float)motoresSaidaUs[1],    
        (float)motoresSaidaUs[2], (float)motoresSaidaUs[3],    
        0.0f, gps_alt, sensores_ok, mag_status, mag_progresso                 
        );
}

//avaliação do serial, usado somente se STM32_DEBUG_UART estiver definido
long AUX_STM::debug_escalar(float valor, float escala) //transforma valor float em um inteiro para envio via serial, evitando problemas de precisão e overflow
{//é preciso pois o platformIO não suporta float no printf, então enviamos como inteiro e escalamos no lado do receptor
    if (!isfinite(valor))
        return 99999L;
    float escalado = valor * escala;
    if (escalado >= 99999.0f)
        return 99999L;
    if (escalado <= -99999.0f)
        return -99999L;
    return (long)(escalado >= 0.0f ? escalado + 0.5f : escalado - 0.5f);
}

void AUX_STM::debug_serial() //envio serial
{
#if defined(STM32_DEBUG_UART)
    static uint8_t etapaDebug = 0;
    if (millis() - tempoDebug < 500)
        return;
    tempoDebug = millis(); //tempo do debug

    static uint32_t tx_anterior = 0; //enviados via UART
    static uint32_t bytes_rx_anteriores = 0;  //bytes recebidos via UART
    static uint32_t comandos_anteriores = 0; //comandos enviados (throttle, roll, pitch, yaw)
    static uint32_t crc_anterior = 0; //checagem para dados corrompidos
    uint32_t tx_atual = cstm.UART_FramesEnviados();
    uint32_t bytes_rx_atual = cstm.UART_BytesRecebidos();
    uint32_t comandos_atual = cstm.UART_FramesRecebidos();
    uint32_t crc_atual = cstm.UART_CRCFailures();

    unsigned long agora = millis();
    bool comando_fresco = comando_valido && agora - tempoComando <= TEMPO_MAX_COMANDO; //tempo entre comandos válido
    bool mpu_fresco = mpu_pronto && agora - tempo_mpu_valido <= 50; // tempo entre chamadas mpu
    bool bmp_fresco = bmp_pronto && bmp.idadeAmostraMs() <= 250; //chamadas bmp
    bool mag_fresco = mag_voo_ok(); //mag
    bool mag_amostra_fresca = mag_amostra_valida; //valores válidos
    bool mag_calibracao_ativa = mag_calibrando(); //mag chamada calibração
    char mensagem[224]; //tamanho do array a ser enviado via UART
    int tamanhoMensagem; //colocar todos os dados em valores inteiros
    if (etapaDebug == 0)
    {
        tamanhoMensagem = snprintf(mensagem, sizeof(mensagem),
                                   "[STM32 UART RX] bytes/s=%lu frames/s=%lu CRC/s=%lu age=%lums cmd10=%ld,%ld,%ld t1000=%ld f=%ld\n",
                                   (unsigned long)(bytes_rx_atual - bytes_rx_anteriores), //número de bytes recebidos via UART
                                   (unsigned long)(comandos_atual - comandos_anteriores), //comandos (frames)
                                   (unsigned long)(crc_atual - crc_anterior), //erros 
                                   cstm.UART_TempoUltimoPacote() ? agora - cstm.UART_TempoUltimoPacote() : 0xFFFFFFFFUL, //tempo entre envios UART
                                   debug_escalar(cstm.UART_receber(0), 10.0f), //converte todos esses valores float p int 
                                   debug_escalar(cstm.UART_receber(1), 10.0f),
                                   debug_escalar(cstm.UART_receber(2), 10.0f),
                                   debug_escalar(cstm.UART_receber(3), 1000.0f),
                                   debug_escalar(cstm.UART_receber(4), 1.0f));
    }
    else
    {
        tamanhoMensagem = snprintf(mensagem, sizeof(mensagem),
                                   "[STM32 PWM] state=%d active=%d hz=%lu mode=%d ccr_us=%lu,%lu,%lu,%lu cmd_us=%d,%d,%d,%d gates[c%d,m%d,b%d,g%d,n%d,r%d,s%d,k%d] tx/s=%lu loop=%lu sens=%lu\n",
                                   (int)estado, estado == VOANDO,
                                   (unsigned long)esc.getFrequencyHz(), esc.pwmChannelsConfigured(),
                                   (unsigned long)esc.getPulseWidthUs(1),
                                   (unsigned long)esc.getPulseWidthUs(2),
                                   (unsigned long)esc.getPulseWidthUs(3),
                                   (unsigned long)esc.getPulseWidthUs(4),
                                   motoresSaidaUs[0], motoresSaidaUs[1],
                                   motoresSaidaUs[2], motoresSaidaUs[3],
                                   comando_fresco, mpu_fresco, bmp_fresco, mag_fresco,
                                   neutro_armar_confirmado, pode_ficar_pronto(),
                                   mag_amostra_fresca, mag_calibracao_ativa,
                                   (unsigned long)(tx_atual - tx_anterior),
                                   tempoMaximoLoopUs, tempoMaximoSensoresUs); //retorna valores que permitem identificar o erro
    }
    bool escreveu = Serial && tamanhoMensagem > 0 &&
                    tamanhoMensagem < (int)sizeof(mensagem) &&
                    Serial.availableForWrite() >= tamanhoMensagem; //função q retorna 1 se tiver escrito a mensagem toda
    if (escreveu)
        Serial.write((const uint8_t*)mensagem, tamanhoMensagem); //coloca no monitor serial para ser visto, via powershell

    if (etapaDebug == 0)
    {
        bytes_rx_anteriores = bytes_rx_atual; //assim, quando fizermos a diferença ela deve se manter constante
        comandos_anteriores = comandos_atual;
        crc_anterior = crc_atual;
    }
    else
        tx_anterior = tx_atual;

    etapaDebug = (etapaDebug + 1) % 2; //debug sempre alterna entre 0 e 1, repetidamente a cada iteração
    if (escreveu) { tempoMaximoLoopUs = 0; tempoMaximoSensoresUs = 0;}
#endif
}

//tenta religar sensores que falharam
void AUX_STM::tentar_reconectar()
{
    if (millis() - tempoRetry < 2000)
        return;
    tempoRetry = millis();

    //tentar reconectar mpu
    if (!mpu_pronto)
    {
        mpu_pronto = mpu.inicializar();
        if (mpu_pronto)
            mpu_pronto = mpu.calibrarMPU();
    }

    //tentar reconectar bmp
    if (!bmp_pronto)
    {
        bmp_pronto = bmp_init();
        if (bmp_pronto && bmp.pressao > 0.0f)
        {
            if (pressao_referencia_pa <= 0.0f) pressao_referencia_pa = bmp.pressao;
            float alt_inicial = altitude_barometrica_relativa(bmp.pressao);
            if (isfinite(alt_inicial)) kf.setAlt(alt_inicial);
        }
    }

    #if USAR_MAG
        if (!mag_pronto)
            mag_pronto = magnetometro.mag_init();
    #endif
        //todos os exigidos conectaram
        if (estado == DESLIGADO && pode_ficar_pronto())
            estado = PRONTO;
    }

void AUX_STM::iniciarSTM()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    #if defined(STM32_DEBUG_UART)
        Serial.begin(115200);
        unsigned long inicioSerial = millis();
        while (!Serial && millis() - inicioSerial < 1500) delay(10); //serial não for ligado ou se passar mais de 1500ms
    #endif

    //inicialização de todos os filtros e módulos
    indicarEtapaSetup(1);
    esc_pronto = esc.begin() && esc.pwmChannelsConfigured();
    indicarEtapaSetup(2);
    if (esc_pronto)
    {
        esc.armarESC();
        #if defined(STM32_DEBUG_UART)
            esc.imprimirMapa(Serial);
        #endif
    }
    indicarEtapaSetup(3);
    if (esc_pronto)
        esc.ESCRodar(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
    else
    {
        esc.parar();
        #if defined(STM32_DEBUG_UART)
            Serial.println("ESC PWM: falha na configuracao; voo bloqueado");
            esc.imprimirMapa(Serial);
        #endif
    }
    indicarEtapaSetup(4);
    cstm.UART_init();
    indicarEtapaSetup(5);
    i2c_init();
    sensores_init();
    indicarEtapaSetup(9);
    pid_init();
    indicarEtapaSetup(10);
    tempoAnterior = micros();
    tempoRetry = millis();
}

void AUX_STM::rodarSTM()
{
    unsigned long inicioLoopUs = micros();

    tentar_reconectar();
    comm_receber();

    unsigned long inicioSensoresUs = micros();
    sensores_ler();
    unsigned long duracaoSensoresUs = micros() - inicioSensoresUs;
    if (duracaoSensoresUs > tempoMaximoSensoresUs)
        tempoMaximoSensoresUs = duracaoSensoresUs;

    if (estado == PRONTO && !pode_ficar_pronto())
        estado = DESLIGADO;

    unsigned long agora = micros();
    float dt = (agora - tempoAnterior) * 0.000001; //microsegundos -> segundos
    tempoAnterior = agora;
    if (dt <= 0.0f || dt > 0.1f) //evita um dt enorme na primeira volta (após calibrações) explodir as integrações
        dt = 0.01f;

    kalman_atualizar(dt);

    if (!verificar_queda())
        motores_escrever();

    comm_enviar();
    unsigned long duracaoLoopUs = micros() - inicioLoopUs;
    if (duracaoLoopUs > tempoMaximoLoopUs)
        tempoMaximoLoopUs = duracaoLoopUs;
    debug_serial();
    digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
}

