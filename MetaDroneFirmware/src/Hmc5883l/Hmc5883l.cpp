#include "Hmc5883l.h"
#include <EEPROM.h>
#include <math.h>
#include <stddef.h>

struct DadosCalibracao
{
    uint32_t identificador;
    uint16_t versao;
    float offset_x;
    float offset_y;
    float offset_z;
    float escala_x;
    float escala_y;
    float escala_z;
    uint32_t validacaoSomatorio;
};

//documentação do algoritmo FNV-1a disponível em: https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
uint32_t Algoritmo_validacaoEEPROMM(const uint8_t* dados, size_t tamanho)
{
    //hash é um conjunto de dados que podem ser descritos de uma ÚNICA forma em uma única variável
    uint32_t hash = 2166136261UL; //valor básico do algoritmo, UL significa para tratar o valor como unsigned long, evita overflow
    for (size_t i = 0; i < tamanho; i++)
    {
        hash ^= dados[i]; //realiza uma XOR com cada bit do array de dados, entre muitas aspas "comprime" o arquivo
        hash *= 16777619UL; //constante pré-definida pela funcionalidade do algoritmo
    }
    return hash;
}

MAG_SENSOR::MAG_SENSOR() : mag(1) {} //qualquer valor funciona, é um construtor que a biblioteca "Adafruit_HMC5883_U" requer, permite o uso de vários magnetometros com números diferentes

bool MAG_SENSOR::mag_init()
{
    sensor_pronto = mag.begin();
    if (!sensor_pronto)
    {
        calibracao_status = SENSOR_AUSENTE;
        return false;
    }
    mag.setMagGain(HMC5883_MAGGAIN_1_3); //ganho ideal, é passível de ajuste, mas este é o recomendado para leitura em relação ao eixo terrestre
    calibracao_valida = carregarCalibracao();
    calibracao_status = calibracao_valida ? CALIBRADO : NAO_CALIBRADO;
    return true;
}

bool MAG_SENSOR::mag_ler()
{
    if (!sensor_pronto)
        return false;

    sensors_event_t evento; //parâmetro para obter taxa de evento (leituras EIXOS) do magnetometro
    if (!mag.getEvent(&evento) || !isfinite(evento.magnetic.x) || !isfinite(evento.magnetic.y) || !isfinite(evento.magnetic.z))
        return false; //nenhum dos eixos retornou valores válidos

    float x_dadoBruto = evento.magnetic.x;
    float y_dadoBruto = evento.magnetic.y;
    float z_dadoBruto = evento.magnetic.z;
    float intensidade = sqrtf(x_dadoBruto * x_dadoBruto + y_dadoBruto * y_dadoBruto + z_dadoBruto * z_dadoBruto); //pitágoras 3 dimensões
    if (!isfinite(intensidade) || intensidade < 5.0f || intensidade > 1000.0f)
        return false; //joga no lixo valores muito altos

    if (getCalibrando()) //por causa do enum é, comparamos estados que queremos, 0 é não calibrado.
        coletarCalibracao(x_dadoBruto, y_dadoBruto, z_dadoBruto);

    mag_mx = (x_dadoBruto - offset_x) * escala_x;
    mag_my = (y_dadoBruto - offset_y) * escala_y;
    mag_mz = (z_dadoBruto - offset_z) * escala_z;

    float intensidade_corrigida = sqrtf(mag_mx * mag_mx + mag_my * mag_my + mag_mz * mag_mz); //valor a partir dos offsets
    if (getCalibrado() && (!isfinite(intensidade_corrigida) || intensidade_corrigida < 10.0f || intensidade_corrigida > 150.0f))
        return false;

    if (getCalibrando() && millis() - inicio_calibracao >= TEMPO_CALIBRACAO_MS)
        finalizarCalibracao();

    return true;
}

bool MAG_SENSOR::iniciarCalibracao()
{
    if (!sensor_pronto || getCalibrando())
        return false;

    min_x = min_y = min_z = INFINITY;
    max_x = max_y = max_z = -INFINITY;
    amostras_calibracao = 0;
    inicio_calibracao = millis();
    calibracao_status = CALIBRANDO;
    return true;
}

void MAG_SENSOR::rejeitarCalibracao()
{
    calibracao_status = sensor_pronto ? CALIBRACAO_FALHOU : SENSOR_AUSENTE;
}

float MAG_SENSOR::getProgresso() const
{
    if (calibracao_status == CALIBRADO)
        return 100.0f;
    if (calibracao_status != CALIBRANDO)
        return 0.0f;

    float progresso = (millis() - inicio_calibracao) * 100.0f / TEMPO_CALIBRACAO_MS;
    return progresso > 100.0f ? 100.0f : progresso;
}

void MAG_SENSOR::coletarCalibracao(float x, float y, float z)
{
    if (x < min_x) min_x = x;
    if (x > max_x) max_x = x;
    if (y < min_y) min_y = y;
    if (y > max_y) max_y = y;
    if (z < min_z) min_z = z;
    if (z > max_z) max_z = z;
    if (amostras_calibracao < UINT16_MAX)
        amostras_calibracao++;
}

bool MAG_SENSOR::finalizarCalibracao()
{
    float raio_x = (max_x - min_x) * 0.5f;
    float raio_y = (max_y - min_y) * 0.5f;
    float raio_z = (max_z - min_z) * 0.5f;
    //rejeitamos a calibração se qualquer um dos dados for distorcido
    if (amostras_calibracao < 300 || !isfinite(raio_x) || !isfinite(raio_y) || !isfinite(raio_z) || raio_x < 6.0f || raio_y < 6.0f || raio_z < 6.0f || raio_x > 500.0f || raio_y > 500.0f || raio_z > 500.0f) 
    {
        calibracao_status = CALIBRACAO_FALHOU;
        return false;
    }

    float raio_medio = (raio_x + raio_y + raio_z) / 3.0f;
    float nova_escala_x = raio_medio / raio_x;
    float nova_escala_y = raio_medio / raio_y;
    float nova_escala_z = raio_medio / raio_z;
    if (!isfinite(nova_escala_x) || !isfinite(nova_escala_y) || !isfinite(nova_escala_z) ||
        nova_escala_x < 0.2f || nova_escala_x > 5.0f ||
        nova_escala_y < 0.2f || nova_escala_y > 5.0f ||
        nova_escala_z < 0.2f || nova_escala_z > 5.0f) //segunda checagem se os valores são válidos
    { calibracao_status = CALIBRACAO_FALHOU; return false; }

    //desenvolvemos a tendência de cada eixo
    offset_x = (max_x + min_x) * 0.5f;
    offset_y = (max_y + min_y) * 0.5f;
    offset_z = (max_z + min_z) * 0.5f;
    escala_x = nova_escala_x;
    escala_y = nova_escala_y;
    escala_z = nova_escala_z;
    if (!salvarCalibracao()) { calibracao_status = CALIBRACAO_FALHOU; return false; }

    calibracao_valida = true;
    calibracao_status = CALIBRADO;
    return true;
}

bool MAG_SENSOR::salvarCalibracao()
{
    DadosCalibracao dados = {};

    dados.identificador = IDENTIFICADOR;
    dados.versao = CALIBRACAO_VERSAO;
    dados.offset_x = offset_x;
    dados.offset_y = offset_y;
    dados.offset_z = offset_z;
    dados.escala_x = escala_x;
    dados.escala_y = escala_y;
    dados.escala_z = escala_z;

    //copia os bytes da struct para um buffer, precisa ser lido em ponteiro, normalmente é float
    uint8_t buffer[sizeof(DadosCalibracao)];
    memcpy(buffer, &dados, sizeof(DadosCalibracao));

    //calcula o checksum sobre o buffer, do byte 4 até antes do checksum
    dados.validacaoSomatorio = Algoritmo_validacaoEEPROMM( 
        buffer + sizeof(dados.identificador),
        offsetof(DadosCalibracao, validacaoSomatorio) - sizeof(dados.identificador));

    if (EEPROM.length() < sizeof(dados))
        return false;
    
        //colocar na memória não volátil do STM32 para evitar ter que calibrar toda vez o magnetometro
    EEPROM.put(0, dados);
    DadosCalibracao verificado = {};
    EEPROM.get(0, verificado);

    return verificado.identificador == dados.identificador &&
           verificado.versao == dados.versao &&
           verificado.validacaoSomatorio == dados.validacaoSomatorio;
}

bool MAG_SENSOR::carregarCalibracao()
{
    if (EEPROM.length() < sizeof(DadosCalibracao))
        return false;

    DadosCalibracao dados = {};
    EEPROM.get(0, dados);
    uint32_t checksum = Algoritmo_validacaoEEPROMM(
        reinterpret_cast<const uint8_t*>(&dados) + sizeof(dados.identificador),
        offsetof(DadosCalibracao, validacaoSomatorio) - sizeof(dados.identificador));

    if (dados.identificador != IDENTIFICADOR || dados.versao != CALIBRACAO_VERSAO ||
        dados.validacaoSomatorio != checksum ||
        !isfinite(dados.offset_x) || !isfinite(dados.offset_y) || !isfinite(dados.offset_z) ||
        !isfinite(dados.escala_x) || !isfinite(dados.escala_y) || !isfinite(dados.escala_z) ||
        dados.escala_x < 0.2f || dados.escala_x > 5.0f ||
        dados.escala_y < 0.2f || dados.escala_y > 5.0f ||
        dados.escala_z < 0.2f || dados.escala_z > 5.0f)
        return false;

    offset_x = dados.offset_x;
    offset_y = dados.offset_y;
    offset_z = dados.offset_z;
    escala_x = dados.escala_x;
    escala_y = dados.escala_y;
    escala_z = dados.escala_z;
    return true;
}
