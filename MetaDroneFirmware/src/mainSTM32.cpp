#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <ESC.h>
#include <BMP.h>
#include <MPU.h>
#include <PID.h>
#include <KALMAN.h>
#include <commSTM.h>
#include <DIST_SENSOR.h>
#include <ina219.h>
#include <gps_neo6m.h>
#include <Hmc5883l.h>

#ifndef USAR_MAG
#define USAR_MAG 1
#endif

#ifndef USAR_INA219
#define USAR_INA219 0
#endif

#ifndef USAR_GPS
#define USAR_GPS 0
#endif

#ifndef USAR_DISTSENSOR
#define USAR_DISTSENSOR 0
#endif

//construtores das bibliotecas
MPU6500 mpu;
PID pid;
commSTM cstm;
ESC_STM32 esc;
DistSensor distsensor;
BMP bmp(0x77);
KALMAN_LINEAR kf(0.1f, 0.5f);
INA219_Sensor ina(0x40);
Uart SerialGPS(PA_12, PA_11);
GPS_NEO6M gps(SerialGPS);
MAG_SENSOR magnetometro;

//desenvolveremos estados para verificar ações do drone
enum EstadoVoo { DESLIGADO, PRONTO, VOANDO };
EstadoVoo estado = DESLIGADO; //inicialmente desligado

//comandos iniciais
float roll_ref = 0.0f, pitch_ref = 0.0f, yaw_ref = 0.0f, throttle_ref = 0.0f;
float yaw_alvo = 0.0f;

bool mpu_pronto = false;
bool bmp_pronto = false;
bool ina_pronto = false;
bool dist_pronto = false;
bool comando_valido = false;
bool mag_pronto = false;
bool mag_amostra_valida = false;
bool calibracao_mag_anterior = false;
bool temporizando_neutro = false;
bool neutro_armar_confirmado = false;
float pressao_referencia_pa = 0.0f;
float pressao_filtrada_pa = 0.0f;
bool pressao_filtrada_pronta = false;
static constexpr float FILTRO_PRESSAO_ALPHA = 0.25f;
static constexpr float FILTRO_REFERENCIA_SOLO_ALPHA = 0.02f;

static float altitude_barometrica_relativa(float pressao_pa)
{
    if (!isfinite(pressao_pa) || pressao_pa <= 0.0f ||
        !isfinite(pressao_referencia_pa) || pressao_referencia_pa <= 0.0f)
        return NAN;
    return 44330.0f * (1.0f - powf(pressao_pa / pressao_referencia_pa, 0.1903f));
}

//gerenciamento de tempo:
unsigned long tempoAnterior = 0;
unsigned long tempo_neutro = 0;
unsigned long tempoBMP = 0;
unsigned long tempoRange = 0;
unsigned long tempoINA = 0;
unsigned long tempo_mag_us = 0;
unsigned long tempo_mag_valido = 0;
unsigned long tempo_mpu_valido = 0;
unsigned long tempoPID = 0;
unsigned long tempoEnvio = 0;
unsigned long tempoDebug = 0;
unsigned long tempoComando = 0;
unsigned long tempoRetry = 0;
unsigned long tempoMaximoLoopUs = 0;
unsigned long tempoMaximoSensoresUs = 0;

//variáveis globais do sistema
uint16_t distancia_mm = 0;
static const unsigned long TEMPO_MAX_COMANDO = 250;
static const float TENSAO_MIN = 9.6f;
static const float TAXA_VARIACAO_THROTTLE_US_S = 1000.0f;
float throttle_atual_us = ESC_MIN_US; //throttle atual em função de microsegundos.
int motoresSaidaUs[4] = {ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US};

//módulo opcional inicialização: magnetometro
static bool mag_requisito_ok()
{
#if USAR_MAG
    return mag_pronto;
#else
    return true;
#endif
}

static bool mag_voo_ok()
{
#if USAR_MAG
    return mag_pronto && millis() - tempo_mag_valido <= 250;
#else
    return true;
#endif
}

static bool mag_calibrando()
{
#if USAR_MAG
    return magnetometro.getCalibrando();
#else
    return false;
#endif
}

//módulo opcional inicialização: sensor de corrente INA219
static bool ina_requisito_ok()
{
#if USAR_INA219
    return ina_pronto;
#else
    return true;
#endif
}

static float tensao_atual()
{
#if USAR_INA219
    return ina.getTensao();
#else
    return -1.0f; //na interface isso é N/A
#endif
}
static bool tensao_armar()
{
#if USAR_INA219
    float tensao = tensao_atual();
    return isfinite(tensao) && tensao >= TENSAO_MIN;
#else
    return true;
#endif
}

//checa modulos essenciais
static bool pode_ficar_pronto()
{
    return mpu_pronto && bmp_pronto && ina_requisito_ok() && mag_requisito_ok();
}


//i2c começo
void i2c_init()
{
    Wire.setSDA(PB7);
    Wire.setSCL(PB6);
    Wire.begin();
    Wire.setClock(400000);
#if defined(STM32_DEBUG_UART)
    Serial.println("I2C pins: SDA=PB7, SCL=PB6, clock=400kHz");
#endif
}

//bmp incializador padrão
bool bmp_init()
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
            return true; //o bmp para a inicialização do kalman necessita de um valor inicial
        delay(20);
    }
    return false;
}

void indicarEtapaSetup(uint8_t etapa);

void sensores_init()
{
    //essenciais
    indicarEtapaSetup(6);
    mpu_pronto = mpu.inicializar() && mpu.calibrarMPU(); // MPU6500 usa SPI, não I2C
#if defined(STM32_DEBUG_UART)
    Serial.print("MPU6500 SPI init/calibracao: "); Serial.println(mpu_pronto ? "OK" : "FALHA");
#endif
    indicarEtapaSetup(7);
    bmp_pronto = bmp_init();
#if defined(STM32_DEBUG_UART)
    Serial.print("BMP180 init/amostra em 0x"); Serial.print(bmp.endereco(), HEX);
    Serial.print(": "); Serial.println(bmp_pronto ? "OK" : "FALHA");
#endif

//modulos opcionais, caso você tire esses quatro, ainda deve funcionar normal
#if USAR_DISTSENSOR
    indicarEtapaSetup(8);
    dist_pronto = distsensor.VL53L0X_init();
#if defined(STM32_DEBUG_UART)
    Serial.print("VL53L0X init (I2C 0x29): "); Serial.println(dist_pronto ? "OK" : "FALHA");
#endif
#endif
#if USAR_GPS
    gps.init();
#endif
#if USAR_INA219
    ina_pronto = ina.init() && ina.ler();
#if defined(STM32_DEBUG_UART)
    Serial.print("INA219 init (I2C 0x40): "); Serial.println(ina_pronto ? "OK" : "FALHA");
#endif
#else
#if defined(STM32_DEBUG_UART)
    Serial.println("INA219 init: DESABILITADO");
#endif
#endif
#if USAR_MAG
    mag_pronto = magnetometro.mag_init();
#if defined(STM32_DEBUG_UART)
    Serial.print("HMC5883L init (I2C 0x1E): "); Serial.println(mag_pronto ? "OK" : "FALHA");
#endif
#else
#if defined(STM32_DEBUG_UART)
    Serial.println("HMC5883L init: DESABILITADO");
#endif
#endif

    //checa se pode inicalizar o kalman
    if (bmp_pronto)
    {
        // Captura a pressão no solo como referência para altitude relativa ao ponto de partida.
        if (pressao_referencia_pa <= 0.0f) pressao_referencia_pa = bmp.pressao;
        kf.reset(0.0f, 0.0f);
    }
    estado = pode_ficar_pronto() ? PRONTO : DESLIGADO;
}

//configuração base PID, kp, ki e kd 
void pid_init()
{
    pid.Config(5.0, 0.8, 0.03,
               5.0, 0.8, 0.03,
               3.0, 0.5, 0.01,
               0.0, 0.0, 0.0);
    pid.SetPeriod(10);
    pid.SetBaseThrottle(ESC_MIN_US);
    pid.Setpoint(0.0, 0.0, 0.0, 0.0);
}

void sensores_ler()
{
    unsigned long agora_ms = millis();
    bool mag_amostra_nova = false;

#if USAR_MAG
    static uint8_t falhas_mag = 0;
    unsigned long agora_us = micros();
    if (agora_us - tempo_mag_us >= 20000) //gerenciamento padrão de tempo da leitura do magnetometro
    {
        tempo_mag_us = agora_us; //intervalo assim mantém a repetição a cada 20k de microsegundos
        mag_amostra_valida = magnetometro.mag_ler(); //se a leitura retorna true
        mag_amostra_nova = mag_amostra_valida; //temos uma amostra nova
        if (mag_amostra_valida)
        {
            falhas_mag = 0;
            tempo_mag_valido = millis(); //definimos o intervalo de tempo em milsegundos válido do mag
        }
        else if (++falhas_mag >= 5 && millis() - tempo_mag_valido > 250) //pode ser escrito ++variavel, funciona igual
            mag_pronto = false; //pode falhar até no máximo 5 vezes, se nã for valido 5 vezes
    }
#endif

    //mpu roda sempre que possível via SPI
    if (mpu_pronto)
    {
        if (mpu.lerMPU())
        {
            tempo_mpu_valido = millis();
            mpu.MPUcalculos(magnetometro.getX(), magnetometro.getY(), magnetometro.getZ(),
                            USAR_MAG && mag_amostra_nova && magnetometro.getCalibrado());
        }
        else if (millis() - tempo_mpu_valido > 50)
            mpu_pronto = false;
    }
    //bmp roda a cada 10ms via i2c
    if (bmp_pronto && (agora_ms - tempoBMP >= 10))
    {
        tempoBMP = agora_ms;
        if (!bmp.lerBMP() && bmp.idadeAmostraMs() > 250)
            bmp_pronto = false;
    }

#if USAR_DISTSENSOR
    //roda a cada 50ms via i2c
    if (dist_pronto && agora_ms - tempoRange >= 50)
    {
        tempoRange = agora_ms;
        distancia_mm = distsensor.VL53L0X_read();
    }
#endif

#if USAR_INA219
    //roda a cada 100ms via i2c
    if (ina_pronto && agora_ms - tempoINA >= 100)
    {
        tempoINA = agora_ms;
        if (!ina.ler())
            ina_pronto = false;
    }
#endif

#if USAR_GPS
    //gps roda sempre que possível via UART
    gps.atualizar();
#endif
}

void kalman_atualizar(float dt)
{
    if (estado != VOANDO)
    {
        kf.setAlt(0.0f);
        kf.setVel(0.0f);
    }

    //se nenhum sensor estiver funcionando não altera nenhuma variável
    if (!mpu_pronto || !bmp_pronto || !isfinite(bmp.pressao) || bmp.pressao <= 0.0f)
        return; //se não houver valores válidos do bmp o kalman não seria capaz de mudar nada

    kf.CHUTE(mpu.filtro_z, dt); //coloque o intervalo ao qual o kalman rodará

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
            pressao_filtrada_pa += FILTRO_PRESSAO_ALPHA * (bmp.pressao - pressao_filtrada_pa);

        if (estado != VOANDO)
        {
            if (!isfinite(pressao_referencia_pa) || pressao_referencia_pa <= 0.0f)
                pressao_referencia_pa = pressao_filtrada_pa;
            else
                pressao_referencia_pa += FILTRO_REFERENCIA_SOLO_ALPHA *
                                         (pressao_filtrada_pa - pressao_referencia_pa);
        }

        float baro_alt = altitude_barometrica_relativa(pressao_filtrada_pa);
        if (isfinite(baro_alt))
            kf.atualizarKALMAN(baro_alt);
        tempoBaroAnterior = tempoAmostra;
    }

    if (estado != VOANDO)
    {
        kf.setAlt(0.0f);
        kf.setVel(0.0f);
    }
}

//começo da comunicação UART, stm32 p esp32 e vice-versa
void comm_receber()
{
    if (!cstm.UART_receber())
        return;

    float roll = cstm.UART_receber(0);
    float pitch = cstm.UART_receber(1);
    float yaw = cstm.UART_receber(2);
    float throttle = cstm.UART_receber(3);
    bool controle_fresco = cstm.UART_receber(4) == 1.0f;
    float calibrar_mag = cstm.UART_receber(5);

    if (cstm.UART_receber(4) == 2.0f)
    {
        if (isfinite(roll) && isfinite(pitch) && isfinite(yaw) &&
            roll >= 0.0f && roll <= 10.0f &&
            pitch >= 0.0f && pitch <= 1.0f &&
            yaw >= 0.0f && yaw <= 2.0f &&
            estado != VOANDO && throttle_ref <= 0.02f)
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

    //isfinite checa se o valor é "real", fabs retorna valor absoluto, checa antes de mandar os comandos
    if (!controle_fresco || !isfinite(roll) || !isfinite(pitch) || !isfinite(yaw) || !isfinite(throttle) ||
        fabsf(roll) > 45.0f || fabsf(pitch) > 45.0f || fabsf(yaw) > 180.0f ||
        throttle < 0.0f || throttle > 1.0f)
    {
        comando_valido = false;
        throttle_ref = 0.0f;
        temporizando_neutro = false;
        neutro_armar_confirmado = false; //não pode armar controles retornam movimentos bruscos
        return;
    }
    //se passar na checagem
    roll_ref = roll;
    pitch_ref = pitch;
    yaw_ref = yaw;
    throttle_ref = throttle;
    tempoComando = millis();
    comando_valido = true;

    //é preciso durante a calibração checar se não há nenhum movimento nos controles, caso contrário ocorrerá um ERRO na calibração e se manterá por todo o funcionamento
    bool controles_neutros = fabsf(roll_ref) <= 2.0f && fabsf(pitch_ref) <= 2.0f && fabsf(yaw_ref) <= 2.0f;
    if (estado == PRONTO && !mag_calibrando() && throttle_ref <= 0.02f && controles_neutros)
    {
        if (!temporizando_neutro) //0 inicialmente
        {
            tempo_neutro = millis(); //começa o gerenciamento de tempo
            temporizando_neutro = true;
        }
        //timeout para começar armar, não mexeu nos controles
        if (millis() - tempo_neutro >= 500)
            neutro_armar_confirmado = true;
    }
    else if (estado == PRONTO && !mag_calibrando() && throttle_ref > 0.02f)
    {
        temporizando_neutro = false;
        //essenciais + módulos ligados (cada um responde por si)
        if (neutro_armar_confirmado && pode_ficar_pronto() &&
            (!USAR_MAG || mag_amostra_valida) && tensao_armar())
        {
            estado = VOANDO;
            pressao_referencia_pa = pressao_filtrada_pronta ? pressao_filtrada_pa : bmp.pressao;
            kf.reset(0.0f, 0.0f);
            yaw_alvo = mpu.angulo_z;
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

bool verificar_queda()
{
    bool link_caiu = !comando_valido || millis() - tempoComando > TEMPO_MAX_COMANDO; //não está mais recebendo comandos ao setpoint
    bool bateria_baixa = false;
    
#if USAR_INA219
bateria_baixa = !ina_pronto || (tensao_atual() > 0.5f && tensao_atual() < TENSAO_MIN); //checa novamente se o ina está ok e sua tensão medida
#endif

    bool sensores_falharam = !mpu_pronto || millis() - tempo_mpu_valido > 50 ||
                             !bmp_pronto || bmp.idadeAmostraMs() > 250; //falha de qualquer sensor essencial
    bool magnetometro_falhou = !mag_voo_ok();

    if (estado == VOANDO && (link_caiu || bateria_baixa || sensores_falharam || magnetometro_falhou || throttle_ref <= 0.02f))
    {
        estado = PRONTO;
        throttle_ref = 0.0f;
        temporizando_neutro = false;
        neutro_armar_confirmado = false;
        pid.Reset();
    }

    if (link_caiu || bateria_baixa || sensores_falharam || magnetometro_falhou ||
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

void motores_escrever()
{
    unsigned long agora = millis();
    unsigned long intervalo = agora - tempoPID;
    if (intervalo < 10)
        return;
    tempoPID = agora;

    float dt = intervalo * 0.001f;
    if (dt <= 0.0f || dt > 0.1f)
        dt = 0.01f; //manter o intervalo em milisegundos

    //converte o valor do throttle para o valor de escrita dos ESCs, chega em porcentagem, throttle_ref = porcentagem
    float throttle_alvo = ESC_MIN_US + throttle_ref * (ESC_MAX_US - ESC_MIN_US);
    float max_delta = TAXA_VARIACAO_THROTTLE_US_S * dt;
    if (throttle_alvo > throttle_atual_us + max_delta)
        throttle_atual_us += max_delta; //regula a variação nos picos, se do nada o throttle estava em 1050 e vai para o máximo, isto "suaviza" este pico
    else if (throttle_alvo < throttle_atual_us - max_delta)
        throttle_atual_us -= max_delta;

    pid.SetBaseThrottle(throttle_atual_us); //escreve dentro do intervalo
    yaw_alvo += yaw_ref * dt; //integra yaw
    while (yaw_alvo > 180.0f) yaw_alvo -= 360.0f; ///isso "trava" o yaw, basicamente deixa ele sempre dentro do intervalo de 0 a 360°, ele cresceria infinitamente caso contrário devido a integração em função do tempo
    while (yaw_alvo < -180.0f) yaw_alvo += 360.0f;
    float yaw_setpoint = yaw_alvo; //yaw que desejamos
    float erro_yaw = yaw_setpoint - mpu.angulo_z; //erro yaw = yaw onde queremos - yaw real
    if (erro_yaw > 180.0f) yaw_setpoint -= 360.0f; //normaliza o erro yaw de 0 a 360°
    else if (erro_yaw < -180.0f) yaw_setpoint += 360.0f;
    pid.Setpoint(roll_ref, pitch_ref, yaw_setpoint, 0.0);
    pid.Input(mpu.angulo_x, mpu.angulo_y, mpu.angulo_z, kf.getAlt());
    pid.RunPID(true, true, true, false);

    motoresSaidaUs[0] = constrain((int)pid.GetM1(), ESC_MIN_US, ESC_MAX_US);
    motoresSaidaUs[1] = constrain((int)pid.GetM2(), ESC_MIN_US, ESC_MAX_US);
    motoresSaidaUs[2] = constrain((int)pid.GetM3(), ESC_MIN_US, ESC_MAX_US);
    motoresSaidaUs[3] = constrain((int)pid.GetM4(), ESC_MIN_US, ESC_MAX_US);
    esc.ESCRodar(motoresSaidaUs[0], motoresSaidaUs[1],
                 motoresSaidaUs[2], motoresSaidaUs[3]); //mandar os motores funcionaremos com a partir dos setpoints desejados
}

void comm_enviar()
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
        gps_alt = (float)gps.obter_alt(); //caso você retorne obter_alt puro, quando houver um erro retorna NaN
#endif

    //envio x,y,z, alt, vel, temp, m1, m2, m3, m4, tensao, altitude gps, sensores_ok, estado_mag, progresso_mag
    cstm.UART_enviar(mpu.angulo_x, mpu.angulo_y, mpu.angulo_z,
                     kf.getAlt(), kf.getVel(), bmp.temperatura,
                     (float)motoresSaidaUs[0], (float)motoresSaidaUs[1],
                     (float)motoresSaidaUs[2], (float)motoresSaidaUs[3],
                     tensao_atual(), gps_alt,
                     sensores_ok,
#if USAR_MAG
                     magnetometro.getStatus(), magnetometro.getProgresso());
#else
                     5.0f, 0.0f); // status 5 = magnetometro desabilitado no firmware
#endif
}

//avaliação do serial, compilado somente se STM32_DEBUG_UART for verdadeiro,
static long debug_escalar(float valor, float escala)
{
    if (!isfinite(valor))
        return 99999L;
    float escalado = valor * escala;
    if (escalado >= 99999.0f)
        return 99999L;
    if (escalado <= -99999.0f)
        return -99999L;
    return (long)(escalado >= 0.0f ? escalado + 0.5f : escalado - 0.5f);
}

void debug_serial()
{
#if defined(STM32_DEBUG_UART)
    static uint8_t etapaDebug = 0;
    if (millis() - tempoDebug < 500)
        return;
    tempoDebug = millis();

    static uint32_t tx_anterior = 0;
    static uint32_t bytes_rx_anteriores = 0;
    static uint32_t comandos_anteriores = 0;
    static uint32_t crc_anterior = 0;
    uint32_t tx_atual = cstm.UART_FramesEnviados();
    uint32_t bytes_rx_atual = cstm.UART_BytesRecebidos();
    uint32_t comandos_atual = cstm.UART_FramesRecebidos();
    uint32_t crc_atual = cstm.UART_CRCFailures();

    unsigned long agora = millis();
    bool comando_fresco = comando_valido && agora - tempoComando <= TEMPO_MAX_COMANDO;
    bool mpu_fresco = mpu_pronto && agora - tempo_mpu_valido <= 50;
    bool bmp_fresco = bmp_pronto && bmp.idadeAmostraMs() <= 250;
    bool mag_fresco = mag_voo_ok();
    bool mag_amostra_fresca = mag_amostra_valida;
    bool mag_calibracao_ativa = mag_calibrando();
    char mensagem[224];
    int tamanhoMensagem;
    if (etapaDebug == 0)
    {
        tamanhoMensagem = snprintf(mensagem, sizeof(mensagem),
                                   "[STM32 UART RX] bytes/s=%lu frames/s=%lu CRC/s=%lu age=%lums cmd10=%ld,%ld,%ld t1000=%ld f=%ld\n",
                                   (unsigned long)(bytes_rx_atual - bytes_rx_anteriores),
                                   (unsigned long)(comandos_atual - comandos_anteriores),
                                   (unsigned long)(crc_atual - crc_anterior),
                                   cstm.UART_TempoUltimoPacote() ? agora - cstm.UART_TempoUltimoPacote() : 0xFFFFFFFFUL,
                                   debug_escalar(cstm.UART_receber(0), 10.0f),
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
                                   tempoMaximoLoopUs, tempoMaximoSensoresUs);
    }
    bool escreveu = Serial && tamanhoMensagem > 0 &&
                    tamanhoMensagem < (int)sizeof(mensagem) &&
                    Serial.availableForWrite() >= tamanhoMensagem;
    if (escreveu)
        Serial.write((const uint8_t*)mensagem, tamanhoMensagem);

    if (etapaDebug == 0)
    {
        bytes_rx_anteriores = bytes_rx_atual;
        comandos_anteriores = comandos_atual;
        crc_anterior = crc_atual;
    }
    else
        tx_anterior = tx_atual;
    etapaDebug = (etapaDebug + 1) % 2;
    if (escreveu)
    {
        tempoMaximoLoopUs = 0;
        tempoMaximoSensoresUs = 0;
    }
#endif
}

void indicarEtapaSetup(uint8_t etapa)
{
#if defined(STM32_DEBUG_UART)
    if (Serial && Serial.availableForWrite() >= 32)
    {
        Serial.print("STM setup stage=");
        Serial.println(etapa);
    }
#endif
    digitalWrite(LED_BUILTIN, (etapa % 2) ? LOW : HIGH); //debug visual do sensor  
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
#if defined(STM32_DEBUG_UART)
    Serial.begin(115200);
    unsigned long inicioSerial = millis();
    while (!Serial && millis() - inicioSerial < 1500) delay(10);
#endif
    //inicialização de todos os filtros e módulos
    indicarEtapaSetup(1); 
    esc.begin();
    indicarEtapaSetup(2);
    esc.armarESC();
    indicarEtapaSetup(3);
    esc.ESCRodar(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
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

void loop()
{
    unsigned long inicioLoopUs = micros();
    if (millis() - tempoRetry >= 2000) //foi recomendado tentar mais uma vez em caso de falha, evita ter que reinicializar sistema manualmente várias vezes
    {
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

#if USAR_INA219
        if (!ina_pronto)
            ina_pronto = ina.init();
#endif

#if USAR_MAG
        if (!mag_pronto)
            mag_pronto = magnetometro.mag_init();
#endif
        //todos os exigidos conectaram
        if (estado == DESLIGADO && pode_ficar_pronto())
            estado = PRONTO;
    }
    comm_receber();
    unsigned long inicioSensoresUs = micros();
    sensores_ler();
    unsigned long duracaoSensoresUs = micros() - inicioSensoresUs;
    if (duracaoSensoresUs > tempoMaximoSensoresUs)
        tempoMaximoSensoresUs = duracaoSensoresUs; //tempo para incialização dos sensores

    if (estado == PRONTO && !pode_ficar_pronto())
        estado = DESLIGADO;

    unsigned long agora = micros();
    float dt = (agora - tempoAnterior) * 0.000001; //microsegundos
    tempoAnterior = agora;
    if (dt <= 0.0f || dt > 0.1f) //é necessário isto pois quando o código roda inicialmente, se somar os tempos de calibração, o tempo inicial vai ser imediatamente muito alto, explodindo os valores que estão sendo integrados
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
