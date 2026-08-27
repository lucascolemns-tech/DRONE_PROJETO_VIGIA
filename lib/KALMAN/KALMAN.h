//modularização do código
#ifndef  KALMAN_H
#define KALMAN_H

#include <math.h>

class KALMAN_LINEAR
{
public:
  KALMAN_LINEAR(float q, float r)
  {
    Q_acc = q;
    R_alt = r;
    x[0] = 0.0; x[1] = 0.0; //x[0] armazena altitude e x[1] velocidade
    chute_P[0][0] = 10.0;  chute_P[0][1] = 0.0; //taxa initial das estimativas
    chute_P[1][0] = 0.0; chute_P[1][1] = 10.0;
  }

  void CHUTE(float acc_z_inicial, float dt)
  {
    /*
    calculo por movimento uniforme variado s1 = s0 + (v0*t) + (a* 1/2 *  t* t)
    e equação horaria da velocidade v1 = v0 + a*t

    pode ser escrita de forma matricial da maneira:
    F = | 1  dt |
        | 0  1  |
    */
    float termo1 = acc_z_inicial * 0.5 * dt * dt;
    float termo2 = acc_z_inicial * dt;
    x[0] = x[0] + x[1]*dt + termo1;
    x[1] = x[1] + termo2;

    //matriz que propaga a taxa de erro a cada passagem de instante de tempo
    float q11 = (pow(dt, 4))/ 4.0 * Q_acc;
    float q12 = (pow(dt, 3))/ 2.0 * Q_acc;
    float q22 = (pow(dt, 2)) * Q_acc;

    float F_P_FT_00 = chute_P[0][0] + 2 * chute_P[0][1] * dt + chute_P[1][1] * dt * dt; //multiplicação das matrizes F, p E F(em função de T), diz a respeito da altura
    float F_P_FT_01 = chute_P[0][1] + chute_P[1][1] * dt; //correção da altura e velocidade                     
    float F_P_FT_11 = chute_P[1][1]; //velocidade

    chute_P[0][0] = F_P_FT_00 + q11; //chute altura
    chute_P[0][1] = F_P_FT_01 + q12; //termo da diagonal +, são simétricos
    chute_P[1][0] = F_P_FT_01 + q12;
    chute_P[1][1] = F_P_FT_11 + q22; //chute erro velocidade    
  }

  void atualizarKALMAN(float baro_altitude)
  {
    float erro = baro_altitude - x[0];  //erro
    float inc_total = chute_P[0][0] + R_alt; //incerteza total
    float k[2]; 
    k[0] = chute_P[0][0] / inc_total; //se chute_P for alto a divisão vai atuar de forma bruta pra corrigir a diferença enorme 
    k[1] = chute_P[1][0] / inc_total; //divide a parte da matriz, esta parte tem a correlação entre velocidade e altura 

    x[0] = x[0] + k[0] * erro;
    x[1] = x[1] + k[1] * erro;

    float p00 = chute_P[0][0], p01 = chute_P[0][1];
    float p10 = chute_P[1][0], p11 = chute_P[1][1];

    //Teorema de Bayes, calculo da covariância, basicamente a cada vez que você faz uma nova medida sua incerteza tende a dimunuir  
    chute_P[0][0] = (1 - k[0]) * p00;            // Nova incerteza da altura
    chute_P[0][1] = (1 - k[0]) * p01;            // Nova correlação
    chute_P[1][0] = -k[1] * p00 + p10;           // Nova correlação (simétrica)
    chute_P[1][1] = -k[1] * p01 + p11;           // Nova incerteza da velocidade
}
  float getAlt() const { return x[0]; }
  float getVel() const { return x[1]; }
  void setAlt(float h) { x[0] = h; }
  void setVel(float v) { x[1] = v; }

private:
  float Q_acc;
  float R_alt;
  float chute_P[2][2]; //chute
  float x[2]; //armazena velocidade (m/s) e altiude (m)
};

#endif
