#include "PID.h"

/*
Ola, lembrem-se sempre que estamos utilizando a configuração "X" dos motores
assim, observando esse diagrama:
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
Teemos que M1 e M4 precisam estar no sentido horário, e M2 e M3 no sentido anti-horário
*/

//Sempre que a classe PID for chamada essa função roda dentro dela
PID::PID()
:   XAnglePD(&Xinput, &Xsetpoint, &Xoutput, -10, 10, 0.0, Ki, 0.0),
    YAnglePD(&Yinput, &Ysetpoint, &Youtput, -10, 10, 0.0, Ki, 0.0),
    ZAnglePD(&Zinput, &Zsetpoint, &Zoutput, -10, 10, 0.0, Ki, 0.0),
    HeightPD(&Hinput, &Hsetpoint, &Houtput, -15, 15, 0.0, Ki, 0.0)
{}

void PID::Input(double Xi, double Yi, double Zi, double Hi) 
{
  Xinput = Xi;
  Yinput = Yi;
  Zinput = Zi;

  //Assumindo que o Hi seja a altura
  Hinput = Hi;
}

void PID::Setpoint(double Xs, double Ys, double Zs, double Hs) 
{
  //Assumindo que o input já esteja na unidade de medida correta
  Xsetpoint = Xs;
  Ysetpoint = Ys;
  Zsetpoint = Zs;

  //Lembrando que a undidade do H é diferente
  Hsetpoint = Hs;
}

//"p" é proporcional e "d" é derivativo
void PID::Config(double Xp, double Xd, double Yp, double Yd, double Zp, double Zd, double Hp, double Hd) 
{

  XKp = Xp;
  XKd = Xd;

  XAnglePD.setGains(XKp, Ki, XKd);
  
  YKp = Yp;
  YKd = Yd;

  YAnglePD.setGains(YKp, Ki, YKd);
  
  ZKp = Zp;
  ZKd = Zd;

  ZAnglePD.setGains(ZKp, Ki, ZKd);

  HKp = Hp;
  HKd = Hd;

  HeightPD.setGains(HKp, Ki, HKd);

}

void PID::SetBaseThrottle(double Throttle) 
{
  BaseThrottle = Throttle;
}

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

//O metódo de soma é devido o sentido de rotação e posição de cada motor
double PID::GetM1() 
{
  M1 = BaseThrottle + Xoutput - Youtput + Zoutput + Houtput;

  return M1;
}

double PID::GetM2() 
{
  M2 = BaseThrottle - Xoutput - Youtput - Zoutput + Houtput;

  return M2;
}

double PID::GetM3() 
{
  M3 = BaseThrottle + Xoutput + Youtput - Zoutput + Houtput;

  return M3;
}

double PID::GetM4() 
{
  M4 = BaseThrottle - Xoutput + Youtput + Zoutput + Houtput;

  return M4;
}
