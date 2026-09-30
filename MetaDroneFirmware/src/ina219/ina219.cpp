#include "ina219.h"
#include <math.h>

INA219_Sensor::INA219_Sensor(uint8_t addr) : ina(addr), _addr(addr) {}

bool INA219_Sensor::init()
{
    leitura_valida = false;
    tensao = 0.0f; corrente = 0.0f; potencia = 0.0f;
    if (!ina.begin())
        return false;
    ina.setCalibration_32V_2A(); //calibração pra faixa especificada
    return true;
}

bool INA219_Sensor::ler()
{
    tensao = ina.getBusVoltage_V();
    corrente = ina.getCurrent_mA();
    potencia = ina.getPower_mW();
    leitura_valida = isfinite(tensao) && isfinite(corrente) && isfinite(potencia) &&
                     tensao > 0.5f && tensao <= 32.0f; //checa valores válidos
    return leitura_valida;
}