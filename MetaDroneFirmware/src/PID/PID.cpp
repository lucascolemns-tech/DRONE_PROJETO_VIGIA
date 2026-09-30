#include "PID.h"

/*
        Frente
     <-         ->
     M1    X    M2
       \   |   /
        \  |  /
    <------Z------> Y
        /  |  \
       /   |   \
     M3         M4
     ->         <-
M1 e M4 no sentido horário, M2 e M3 no sentido anti-horário
*/

PID::PID()
:   XAnglePD(&Xinput, &Xsetpoint, &Xoutput, -10, 10, 0.0, 0.0, 0.0),
    YAnglePD(&Yinput, &Ysetpoint, &Youtput, -10, 10, 0.0, 0.0, 0.0),
    ZAnglePD(&Zinput, &Zsetpoint, &Zoutput, -10, 10, 0.0, 0.0, 0.0),
    HeightPD(&Hinput, &Hsetpoint, &Houtput, -15, 15, 0.0, 0.0, 0.0)
{}

void PID::Input(double Xi, double Yi, double Zi, double Hi)
{
    Xinput = Xi;
    Yinput = Yi;
    Zinput = Zi;
    Hinput = Hi;
}

void PID::Reset()
{
    XAnglePD.reset();
    YAnglePD.reset();
    ZAnglePD.reset();
    HeightPD.reset();
}

void PID::Setpoint(double Xs, double Ys, double Zs, double Hs)
{
    Xsetpoint = Xs;
    Ysetpoint = Ys;
    Zsetpoint = Zs;
    Hsetpoint = Hs;
}

void PID::Config(double Xp, double Xd, double Xki,
                 double Yp, double Yd, double Yki,
                 double Zp, double Zd, double Zki,
                 double Hp, double Hd, double Hki)
{
    XKp = Xp;  XKd = Xd;  XKi = Xki;
    XAnglePD.setGains(XKp, XKi, XKd);

    YKp = Yp;  YKd = Yd;  YKi = Yki;
    YAnglePD.setGains(YKp, YKi, YKd);

    ZKp = Zp;  ZKd = Zd;  ZKi = Zki;
    ZAnglePD.setGains(ZKp, ZKi, ZKd);

    HKp = Hp;  HKd = Hd;  HKi = Hki;
    HeightPD.setGains(HKp, HKi, HKd);
}

void PID::SetBaseThrottle(double Throttle) { BaseThrottle = Throttle; }

void PID::SetPeriod(unsigned long Period)
{
    PIDPeriod = Period;
    XAnglePD.setTimeStep(PIDPeriod);
    YAnglePD.setTimeStep(PIDPeriod);
    ZAnglePD.setTimeStep(PIDPeriod);
    HeightPD.setTimeStep(PIDPeriod);
}

void PID::RunPID(bool X, bool Y, bool Z, bool H)
{
    if (X) XAnglePD.run();
    if (Y) YAnglePD.run();
    if (Z) ZAnglePD.run();
    if (H) HeightPD.run();
}

double PID::GetM1()
{
    M1 = BaseThrottle + Xoutput - Youtput + Zoutput + Houtput;
    if (M1 < 1000) M1 = 1000;
    if (M1 > 2000) M1 = 2000;
    return M1;
}

double PID::GetM2()
{
    M2 = BaseThrottle - Xoutput - Youtput - Zoutput + Houtput;
    if (M2 < 1000) M2 = 1000;
    if (M2 > 2000) M2 = 2000;
    return M2;
}

double PID::GetM3()
{
    M3 = BaseThrottle + Xoutput + Youtput - Zoutput + Houtput;
    if (M3 < 1000) M3 = 1000;
    if (M3 > 2000) M3 = 2000;
    return M3;
}

double PID::GetM4()
{
    M4 = BaseThrottle - Xoutput + Youtput + Zoutput + Houtput;
    if (M4 < 1000) M4 = 1000;
    if (M4 > 2000) M4 = 2000;
    return M4;
}