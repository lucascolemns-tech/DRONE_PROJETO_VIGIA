#include <AutoPID.h>

#ifndef PID_H
#define PID_H

class PID
{
public:
    void RunPID(bool X, bool Y, bool Z, bool H);
    PID();

    double GetM1();
    double GetM2();
    double GetM3();
    double GetM4();

    void Reset();
    void Input(double Xi, double Yi, double Zi, double Hi);
    void Setpoint(double Xs, double Ys, double Zs, double Hs);
    void Config(double Xp, double Xd, double Xki,
                double Yp, double Yd, double Yki,
                double Zp, double Zd, double Zki,
                double Hp, double Hd, double Hki);

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

    double XKp = 0.0, XKi = 0.0, XKd = 0.0;
    double YKp = 0.0, YKi = 0.0, YKd = 0.0;
    double ZKp = 0.0, ZKi = 0.0, ZKd = 0.0;
    double HKp = 0.0, HKi = 0.0, HKd = 0.0;

    unsigned long PIDPeriod = 10;

    double BaseThrottle = 0;

    double M1 = 0, M2 = 0, M3 = 0, M4 = 0;
};

#endif