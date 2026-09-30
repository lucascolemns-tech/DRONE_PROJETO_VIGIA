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

/*
Para o teste amanhã é preciso modificar esta parte do código, ela 
basicamente torna o magnetometro não obrigatório pro drone funcionar
*/
#ifndef USAR_MAG
#define USAR_MAG 0
#endif

#ifndef USAR_INA219
#define USAR_INA219 1
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
bool comando_valido = false;
bool mag_pronto = false;
bool mag_amostra_valida = false;
bool calibracao_mag_anterior = false;
bool temporizando_neutro = false;
bool neutro_armar_confirmado = false;

//gerenciamento de tempo:
    //geral
unsigned long tempoAnterior = 0;
unsigned long tempo_neutro = 0;
    //sensores
unsigned long tempoBMP = 0;
unsigned long tempoRange = 0;
unsigned long tempoINA = 0;
unsigned long tempo_mag_us = 0;
unsigned long tempo_mag_valido = 0;
    //comunicação
unsigned long tempoPID = 0;
unsigned long tempoEnvio = 0;
unsigned long tempoDebug = 0;
unsigned long tempoComando = 0;
unsigned long tempoRetry = 0;


float throttle_atual_us = ESC_MIN_US; //throttle atual em função de microsegundos.
uint16_t distancia_mm = 0;

static const unsigned long TEMPO_MAX_COMANDO = 250;
static const float TENSAO_MIN = 9.6f;

/*
como o magnetometro ainda não está disponível proponho está forma de rodarmos o PID sem 
sua utilização
*/
static bool mag_requisito_ok()
{
/*Observação, a estrutura #if serve como um if que rodará antes
do código ser compilado, caso verdadeiro compila esta parte,
caso contrário nem sequer roda o código, salva espaço.
*/
#if USAR_MAG
    return mag_pronto && magnetometro.getCalibrado();
#else
    return true;
#endif
}

static bool mag_voo_ok()
{
#if USAR_MAG
    return mag_pronto && magnetometro.getCalibrado() && !magnetometro.getCalibrando() &&
           mag_amostra_valida && millis() - tempo_mag_valido <= 250;
#else
    return true;
#endif
}

static bool ina_requisito_ok()
{
#if USAR_INA219
    return ina_pronto;
#else
    return true;
#endif
}

void i2c_init()
{
    Wire.setSDA(PB7);
    Wire.setSCL(PB6);
    Wire.begin();
    Wire.setClock(400000);
}

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
            return true;
        delay(20);
    }
    return false; 
}

void sensores_init()
{

    mpu_pronto = mpu.inicializar() && mpu.calibrarMPU();
    bmp_pronto = bmp_init();
  
    distsensor.VL53L0X_init();
    gps.init();

    //checar necessidade de usar o INA219
    #if USAR_INA219
    ina_pronto = ina.init() && ina.ler();
    #else
    ina_pronto = false;
    #endif
    //checar se é necessario compilar o magnetometro
    #if USAR_MAG
        mag_pronto = magnetometro.mag_init();
    #endif

    if (bmp_pronto)
    {   //formula para calculo do parâmetro inicial de altura, e inicializa kalman
        float alt_inicial = 44330.0f * (1.0f - powf(bmp.pressao / 101325.0f, 0.1903f)); 
        kf.setAlt(alt_inicial);
        kf.setVel(0.0f);
    }
    estado = (mpu_pronto && bmp_pronto && ina_requisito_ok() && mag_requisito_ok()) ? PRONTO : DESLIGADO; //é necessário esses módulos para iniciar o drone
}

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
    static uint8_t falhas_mag = 0;
    bool mag_amostra_nova = false;
    unsigned long agora_us = micros(); 
    unsigned long agora_ms = millis();
    if (agora_us - tempo_mag_us >= 20000) //gerenciamento padrão de tempo da leitura do magnetometro
    {
        tempo_mag_us = agora_us;
        mag_amostra_valida = magnetometro.mag_ler();
        mag_amostra_nova = mag_amostra_valida;
        if (mag_amostra_valida)
        {
            falhas_mag = 0;
            tempo_mag_valido = millis();
        }
        else if (USAR_MAG && ++falhas_mag >= 5)
            mag_pronto = false;
    }
    //mpu roda sempre que possível via SPI
    if (mpu_pronto)
    {
        if (mpu.lerMPU())
            mpu.MPUcalculos(magnetometro.getX(), magnetometro.getY(), magnetometro.getZ(),
                            USAR_MAG && mag_amostra_nova && magnetometro.getCalibrado());
        else
            mpu_pronto = false;
    }
    //bmp roda a cada 10ms via i2c
    if (bmp_pronto && (agora_ms - tempoBMP >= 10))
    {
        tempoBMP = agora_ms;
        if (!bmp.lerBMP())
            bmp_pronto = false;
    }
    //roda a cada 50ms via i2c
    if (agora_ms - tempoRange >= 50)
    {
        tempoRange = agora_ms;
        distancia_mm = distsensor.VL53L0X_read();
    }
    //roda a cada 100ms via i2c, se for utilizado
    #if USAR_INA219
        if (ina_pronto && agora_ms - tempoINA >= 100)
        {
            tempoINA = agora_ms;
            if (!ina.ler())
                ina_pronto = false;
        }
    #endif
    //gps roda sempre que possível via UART
    gps.atualizar();
}

void kalman_atualizar(float dt)
{
    //se nenhum sensor estiver funcionando não altera nenhuma variável
    if (!mpu_pronto || !bmp_pronto || !isfinite(bmp.pressao) || bmp.pressao <= 0.0f)
        return; //se não houver valores válidos do bmp o kalman não seria capaz de mudar nada

    kf.CHUTE(mpu.filtro_z, dt); //coloque o intervalo ao qual o kalman rodará

    //gerenciar tempo do Kalman
    static unsigned long tempoBaroAnterior = 0;
    if (tempoBMP != 0 && tempoBMP != tempoBaroAnterior)
    {
        float baro_alt = 44330.0f * (1.0f - powf(bmp.pressao / 101325.0f, 0.1903f)); //formula cálculo altitude
        if (isfinite(baro_alt))
        kf.atualizarKALMAN(baro_alt);
        tempoBaroAnterior = tempoBMP;
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

    //pedido para poder calibrar magnetometro
    if (!isfinite(calibrar_mag) || (calibrar_mag != 0.0f && calibrar_mag != 1.0f))
    {
        comando_valido = false;
        return;
    }

    bool pedido_calibracao = calibrar_mag == 1.0f;
    if (pedido_calibracao && !calibracao_mag_anterior)
    {
        bool controle_neutro_valido = controle_fresco && isfinite(roll) && isfinite(pitch) && isfinite(yaw) &&
                                      isfinite(throttle) && fabsf(roll) <= 2.0f && fabsf(pitch) <= 2.0f &&
                                      fabsf(yaw) <= 2.0f && throttle >= 0.0f && throttle <= 0.02f;
        if (estado != VOANDO && controle_neutro_valido)
        {
            if (!magnetometro.iniciarCalibracao())
                magnetometro.rejeitarCalibracao();
        }
        else
            magnetometro.rejeitarCalibracao();
    }
    calibracao_mag_anterior = pedido_calibracao;

    //isfinite checa se o valor é "real", fabs retorna valor absoluto, checa antes de mandar os comandos
    if (!controle_fresco || !isfinite(roll) || !isfinite(pitch) || !isfinite(yaw) || !isfinite(throttle) ||
        fabsf(roll) > 45.0f || fabsf(pitch) > 45.0f || fabsf(yaw) > 180.0f ||
        throttle < 0.0f || throttle > 1.0f)
    {
        comando_valido = false;
        throttle_ref = 0.0f;
        temporizando_neutro = false;
        neutro_armar_confirmado = false;
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
    if (estado == PRONTO && !magnetometro.getCalibrando() && throttle_ref <= 0.02f && controles_neutros)
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
    else if (estado == PRONTO && !magnetometro.getCalibrando() && throttle_ref > 0.02f)
    {
        temporizando_neutro = false;
        //checar se será necessário utilizar o módulo de corente neste ponto
        #if USAR_INA219
                float tensao = ina.getTensao(); //checar se os motores estão funcionandos, pois eles precisam estar desligados durante calibração
        #endif
                if (neutro_armar_confirmado && mag_requisito_ok() &&
                    (!USAR_MAG || mag_amostra_valida) &&
        #if USAR_INA219
                    isfinite(tensao) && tensao >= 9.6f &&
        #endif
            mpu_pronto && bmp_pronto && ina_requisito_ok()) //conjunto de regras para caso não haja o magnetometro o código ainda rodar
        {
            estado = VOANDO;
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
    bateria_baixa = !ina_pronto || (ina.getTensao() > 0.5f && ina.getTensao() < TENSAO_MIN); //checa novamente se o ina está ok e sua tensão medida
#endif
    bool sensores_falharam = !mpu_pronto || !bmp_pronto; //falha de ambos sensores
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
    float max_delta = 300.0f * dt;
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

    esc.ESCRodar((int)pid.GetM1(), (int)pid.GetM2(), //mandar os motores funcionaremos com a partir dos setpoints desejados
                 (int)pid.GetM3(), (int)pid.GetM4());
}

void comm_enviar()
{
    //envio telemetria a cada 20ms
    if (millis() - tempoEnvio < 20)
        return;
    tempoEnvio = millis();
    bool sensores_ok = mpu_pronto && bmp_pronto && ina_requisito_ok() &&
                       mag_requisito_ok() && mag_voo_ok();

    //envio x,y,z, alt, vel, temp, m1, m2, m3, m4, tensao, gps, altitude gps (para comparação), estado_mag, 
    cstm.UART_enviar(mpu.angulo_x, mpu.angulo_y, mpu.angulo_z,
                     kf.getAlt(), kf.getVel(), bmp.temperatura,
                     (float)pid.GetM1(), (float)pid.GetM2(),
                     (float)pid.GetM3(), (float)pid.GetM4(),
                     USAR_INA219 ? ina.getTensao() : -1.0f, gps.checar() ? (float)gps.obter_alt() : 0.0f,
                     sensores_ok,
                     magnetometro.getStatus(), magnetometro.getProgresso()); //caso você retorne obter_alt puro, quando houver um erro retorna NaN
}

//avaliação do serial, compilado somente se STM32_DEBUG_UART for verdadeiro, 
void debug_serial()
{
#if defined(STM32_DEBUG_UART)
    if (millis() - tempoDebug < 100)
        return;
    tempoDebug = millis();

    Serial.print(mpu.angulo_x); Serial.print(",");
    Serial.print(mpu.angulo_y); Serial.print(",");
    Serial.print(mpu.angulo_z); Serial.print(",");
    Serial.print(kf.getAlt()); Serial.print(",");
    Serial.print(kf.getVel()); Serial.print(",");
    Serial.print(USAR_INA219 ? ina.getTensao() : -1.0f); Serial.print(",");
    Serial.print(distancia_mm); Serial.print(",");
    Serial.println((int)estado);
#endif
}

void setup()
{
#if defined(STM32_DEBUG_UART)
    Serial.begin(115200);
#endif
    //inicialização de todos os filtros e módulos
    esc.begin();
    esc.armarESC();
    esc.ESCRodar(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
    cstm.UART_init();
    i2c_init();
    sensores_init();
    pid_init();
    tempoAnterior = micros();
    tempoRetry = millis();
}

void loop()
{
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
                float alt_inicial = 44330.0f * (1.0f - powf(bmp.pressao / 101325.0f, 0.1903f));
                if (isfinite(alt_inicial))
                    kf.setAlt(alt_inicial);
            }
        }
        //tentar reconectar ina
    #if USAR_INA219
            if (!ina_pronto)
                ina_pronto = ina.init();
    #endif
        //tentar reconectar mag se estiver sendo utilizado
        #if USAR_MAG
                if (!mag_pronto)
                    mag_pronto = magnetometro.mag_init();
        #endif
        //todos conectaram, logo está tudo 
        if (mpu_pronto && bmp_pronto && ina_requisito_ok() && mag_requisito_ok() && estado == DESLIGADO)
            estado = PRONTO;
    }
    comm_receber();
    sensores_ler();

    if ((!mpu_pronto || !bmp_pronto || !ina_requisito_ok() || !mag_requisito_ok()) && estado == PRONTO)
        estado = DESLIGADO; //PS: se "USAR_MAG = 0" mag_requisito é sempre 0 também, caso isto cause confusão

    unsigned long agora = micros();
    float dt = (agora - tempoAnterior) * 0.000001; //microsegundos
    tempoAnterior = agora;
    if (dt <= 0.0f || dt > 0.1f) //é necessário isto pois quando o código roda inicialmente, se somar os tempos de calibração, o tempo inicial vai ser imediatamente muito alto, explodindo os valores que estão sendo integrados
        dt = 0.01f;

    kalman_atualizar(dt);

    if (!verificar_queda())
        motores_escrever();

    comm_enviar();
    debug_serial();
}
