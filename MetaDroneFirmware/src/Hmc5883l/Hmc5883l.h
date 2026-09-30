#ifndef MAG_SENSOR_H
#define MAG_SENSOR_H

#include <Arduino.h>
#include <Adafruit_HMC5883_U.h>

class MAG_SENSOR
{
public:
    enum Status : uint8_t { NAO_CALIBRADO = 0, CALIBRANDO = 1, CALIBRADO = 2, CALIBRACAO_FALHOU = 3, SENSOR_AUSENTE = 4 }; //estados para facilitar 

    MAG_SENSOR();
    bool mag_init();
    bool mag_ler();
    bool iniciarCalibracao();
    void rejeitarCalibracao();

    bool getPronto()     const { return sensor_pronto; }
    bool getCalibrado()  const { return calibracao_valida; }
    bool getCalibrando() const { return calibracao_status == CALIBRANDO; }
    uint8_t getStatus()  const { return calibracao_status; }
    float getProgresso() const;
    float getX() const { return mag_mx; }
    float getY() const { return mag_my; }
    float getZ() const { return mag_mz; }

private:
    bool carregarCalibracao();
    bool salvarCalibracao();
    bool finalizarCalibracao();
    void coletarCalibracao(float x, float y, float z);

    Adafruit_HMC5883_Unified mag;
    float mag_mx = 0.0f;
    float mag_my = 0.0f;
    float mag_mz = 0.0f;
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float offset_z = 0.0f;
    float escala_x = 1.0f;
    float escala_y = 1.0f;
    float escala_z = 1.0f;
    float min_x = 0.0f;
    float min_y = 0.0f;
    float min_z = 0.0f;
    float max_x = 0.0f;
    float max_y = 0.0f;
    float max_z = 0.0f;
    unsigned long inicio_calibracao = 0;
    uint16_t amostras_calibracao = 0;
    uint8_t calibracao_status = SENSOR_AUSENTE;
    bool sensor_pronto = false;
    bool calibracao_valida = false;

    static constexpr uint32_t IDENTIFICADOR = 0x4D414731; //você escolhe, por curiosidade, em ASCII 0x4D é M, 0x41 é A, 0x47 é G e 0x31 é 1: MAG1
    static constexpr uint16_t CALIBRACAO_VERSAO = 1;
    static constexpr unsigned long TEMPO_CALIBRACAO_MS = 25000;
};

#endif
