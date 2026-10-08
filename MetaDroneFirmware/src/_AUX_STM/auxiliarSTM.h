#ifndef AUX_STM_H
#define AUX_STM_H

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
#include <gps_neo6m.h>
#include <Hmc5883l.h>

// Módulos opcionais (podem ser sobrescritos via build_flags no platformio.ini)
#ifndef USAR_MAG
#define USAR_MAG 1
#endif

#ifndef USAR_GPS
#define USAR_GPS 0
#endif

#ifndef USAR_DISTSENSOR
#define USAR_DISTSENSOR 0
#endif

class AUX_STM
{
public:
    //desenvolvemos estados para verificar ações do drone
    enum EstadoVoo { DESLIGADO, PRONTO, VOANDO };

    AUX_STM();

    //parâmetros a serem chamados no código principal
    void iniciarSTM(); 
    void rodarSTM();   

    //auxiliares que rodam internamente nesta classe 
    float altitude_barometrica_relativa(float pressao_pa);
    bool mag_requisito_ok();
    bool mag_voo_ok();
    bool mag_calibrando();
    bool pode_ficar_pronto();
    static long debug_escalar(float valor, float escala); // não usa estado, pode ser static

    void i2c_init();
    bool bmp_init();
    void sensores_init();
    void pid_init();
    void sensores_ler();
    void kalman_atualizar(float dt);
    void comm_receber();
    bool verificar_queda();
    void motores_escrever();
    void comm_enviar();
    void debug_serial();

    EstadoVoo estado = DESLIGADO; //inicialmente desligado

    //gerenciamento de tempo:
    unsigned long tempoAnterior = 0;
    unsigned long tempo_neutro = 0;
    unsigned long tempoBMP = 0;
    unsigned long tempoRange = 0;
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

    //comandos iniciais
    float roll_ref = 0.0f, pitch_ref = 0.0f, yaw_ref = 0.0f, throttle_ref = 0.0f;
    float yaw_alvo = 0.0f;

    static constexpr float FILTRO_PRESSAO_ALPHA = 0.25f;
    static constexpr float FILTRO_REFERENCIA_SOLO_ALPHA = 0.02f;

private:
    void indicarEtapaSetup(uint8_t etapa);
    void tentar_reconectar(); // bloco de retry que ficava dentro do loop()

    //construtores das bibliotecas 
    MPU6500 mpu;
    PID pid;
    commSTM cstm;
    ESC_STM32 esc;
    DistSensor distsensor;
    BMP bmp;
    KALMAN_LINEAR kf;
    Uart SerialGPS;
    GPS_NEO6M gps;
    MAG_SENSOR magnetometro;

    //flags de estado dos sensores 
    bool mpu_pronto = false;
    bool bmp_pronto = false;
    bool esc_pronto = false;
    bool dist_pronto = false;
    bool comando_valido = false;
    bool mag_pronto = false;
    bool mag_amostra_valida = false;
    bool calibracao_mag_anterior = false;
    bool temporizando_neutro = false;
    bool neutro_armar_confirmado = false;
    bool pressao_filtrada_pronta = false;

  
    //variáveis globais do sistema
    static constexpr unsigned long TEMPO_MAX_COMANDO = 250;
    static constexpr float TAXA_VARIACAO_THROTTLE_US_S = 1000.0f;
    float pressao_referencia_pa = 0.0f;
    float pressao_filtrada_pa = 0.0f;
    uint16_t distancia_mm = 0;
    float throttle_atual_us = ESC_MIN_US; //throttle atual em função de microsegundos.
    int motoresSaidaUs[4] = {ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US};
};

#endif
