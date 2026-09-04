//documentação: https://github.com/r-downing/AutoPID

//bibliotecas
#include <Arduino.h>
#include <AutoPID.h>

#ifndef PID_H
#define PID_H

//constantes
#define g 9.80665
#define Ki 0.0

class PID
{
public:

    //dados gerais
    void RunPID(bool X, bool Y, bool Z, bool H);
    PID();
    
    double GetM1();
    double GetM2();
    double GetM3();
    double GetM4();

    //configs
    void Input(double Xi, double Yi, double Zi, double Hi);
    void Setpoint(double Xs, double Ys, double Zs, double Hs);
    void Config(double Xp, double Xd, double Yp, double Yd,
                double Zp, double Zd, double Hp, double Hd);

    void SetPeriod(unsigned long Period);
    void SetBaseThrottle(double Throttle);

private:

    AutoPID XAnglePD;
    AutoPID YAnglePD;
    AutoPID ZAnglePD;
    AutoPID HeightPD;

    double Xinput = 0.0, Xsetpoint = 0.0, Xoutput = 0.0;
    double Yinput = 0.0, Ysetpoint = 0.0, Youtput = 0.0;
    double Zinput = 0.0, Zsetpoint = 0.0, Zoutput = 0.0;
    double Hinput = 0.0, Hsetpoint = 0.0, Houtput = 0.0;

    double XKp = 0, XKd = 0;
    double YKp = 0, YKd = 0;
    double ZKp = 0, ZKd = 0;
    double HKp = 0, HKd = 0;

    //Unidade em ms
    unsigned long PIDPeriod = 10;

    double BaseThrottle = 0;

    //porcentagem de cada motor
    double M1 = 0, M2 = 0, M3 = 0, M4 = 0;
};

#endif
