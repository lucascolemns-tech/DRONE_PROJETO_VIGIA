#ifndef META_DRONE_ESC_H
#define META_DRONE_ESC_H

#include <Arduino.h>
#include <esc_range.h>

#if defined(ARDUINO_ARCH_ESP32)

#include <ESP32Servo.h>

#define PINO_M1 32
#define PINO_M2 13
#define PINO_M3 33
#define PINO_M4 15

#define ESC_FREQ_HZ 50
#define ESC_ARM_WAIT_MS 8000

class ESC_ESP32
{
public:
  bool armarESC()
  {
    const int pinos[4] = {PINO_M1, PINO_M2, PINO_M3, PINO_M4};

    for (uint8_t i = 0; i < 4; ++i)
    {
      motores_pwm[i].attach(pinos[i], ESC_MIN_US, ESC_MAX_US);
      if (!motores_pwm[i].attached())
      {
        for (uint8_t j = 0; j < 4; ++j)
          motores_pwm[j].detach();
        armamentoIniciado = false;
        return false;
      }
      motores_pwm[i].setPeriodHertz(ESC_FREQ_HZ);
      motores_pwm[i].writeMicroseconds(ESC_MIN_US);
    }

    inicioArmamentoMs = millis();
    armamentoIniciado = true;
    return true;
  }

  bool prontoParaControle() const
  {
    return armamentoIniciado && millis() - inicioArmamentoMs >= ESC_ARM_WAIT_MS;
  }

  void parar()
  {
    for (uint8_t i = 0; i < 4; ++i)
      if (motores_pwm[i].attached())
        motores_pwm[i].writeMicroseconds(ESC_MIN_US);
  }

  void ESCRodar(int m1, int m2, int m3, int m4)
  {
    if (!prontoParaControle())
    {
      parar();
      return;
    }

    motores_pwm[0].writeMicroseconds(constrain(m1, ESC_MIN_US, ESC_MAX_US));
    motores_pwm[1].writeMicroseconds(constrain(m2, ESC_MIN_US, ESC_MAX_US));
    motores_pwm[2].writeMicroseconds(constrain(m3, ESC_MIN_US, ESC_MAX_US));
    motores_pwm[3].writeMicroseconds(constrain(m4, ESC_MIN_US, ESC_MAX_US));
  }

private:
  Servo motores_pwm[4];
  bool armamentoIniciado = false;
  unsigned long inicioArmamentoMs = 0;
};

#else

#include <HardwareTimer.h>
#include <PeripheralPins.h>
// Se o compilador reclamar de PinMap_PWM, descomente:
// #include "PeripheralPins.h"
#define PINO_M1 PA0
#define PINO_M2 PA1
#define PINO_M3 PA2
#define PINO_M4 PA3

#define ESC_FREQ_HZ 50      // PWM padrão de servo (período de 20 ms)

// Tempos (ajuste ao manual do seu ESC)
#define ESC_ARM_WAIT_MS     3000  // segura MIN até o ESC terminar os bips de início
#define ESC_CAL_MAX_WAIT_MS 5000  // segura MAX após ligar, até os bips do ponto máximo
#define ESC_CAL_MIN_WAIT_MS 5000  // segura MIN até os bips do ponto mínimo

/*
 * Diferenças em relação à versão anterior:
 *  - Cada pino descobre o PRÓPRIO timer e canal (não assume TIM2 para todos).
 *    Se um pino não tem PWM, só aquele canal falha e begin() diz qual.
 *  - Um HardwareTimer por timer real, criado dentro de begin() (nunca no construtor).
 *  - ESCRodar() só passa throttle depois do arming; antes disso segura MIN.
 *  - armar_iniciar()/armar_atualizar(): arming sem delay(); armarESC() continua existindo.
 *  - imprimirMapa(): mostra pino -> timer -> canal e o pulso lido de volta.
 */
class ESC_STM32
{
public:
  static const uint8_t N = 4;

  ESC_STM32() {}
  ESC_STM32(const ESC_STM32 &) = delete;
  ESC_STM32 &operator=(const ESC_STM32 &) = delete;
  ~ESC_STM32()
  {
    for (uint8_t i = 0; i < _nTimers; i++) delete _timers[i];
  }

  // Configura os 4 canais e inicia o PWM em MIN (1000 us).
  // Retorna true se TODOS os canais foram configurados.
  bool begin()
  {
    if (_iniciado) return _falhos == 0;

    const uint32_t pinos[N] = {PINO_M1, PINO_M2, PINO_M3, PINO_M4};

    for (uint8_t i = 0; i < N; i++)
    {
      _ok[i] = false;
      PinName pn = digitalPinToPinName(pinos[i]);

      TIM_TypeDef *inst = (TIM_TypeDef *)pinmap_peripheral(pn, PinMap_TIM);
      if (!inst) { _falhos |= (1 << i); continue; }   // pino sem PWM

      uint32_t ch = STM_PIN_CHANNEL(pinmap_function(pn, PinMap_TIM));

      // dois motores no mesmo timer E mesmo canal = conflito
      bool duplicado = false;
      for (uint8_t j = 0; j < i; j++)
        if (_ok[j] && _inst[j] == inst && _canal[j] == ch) duplicado = true;
      if (duplicado) { _falhos |= (1 << i); continue; }

      HardwareTimer *t = obterTimer(inst);
      if (!t) { _falhos |= (1 << i); continue; }

      t->setMode(ch, TIMER_OUTPUT_COMPARE_PWM1, pn);
      _timer[i] = t;
      _inst[i]  = inst;
      _canal[i] = ch;
      _ok[i]    = true;
    }

    // frequência de cada timer usado, antes dos valores de compare
    for (uint8_t k = 0; k < _nTimers; k++)
      _timers[k]->setOverflow(ESC_FREQ_HZ, HERTZ_FORMAT);

    // todos os canais começam em MIN
    for (uint8_t i = 0; i < N; i++)
      if (_ok[i])
        _timer[i]->setCaptureCompare(_canal[i], ESC_MIN_US, MICROSEC_COMPARE_FORMAT);

    for (uint8_t k = 0; k < _nTimers; k++)
      _timers[k]->resume();

    _iniciado = true;
    return _falhos == 0;
  }

  // Bitmask dos canais que falharam: bit0 = M1 ... bit3 = M4
  uint8_t canaisFalhos() const { return _falhos; }

  // true se begin() foi feito e os 4 canais estão em modo PWM
  bool pwmChannelsConfigured()
  {
    if (!_iniciado || _falhos != 0) return false;
    for (uint8_t i = 0; i < N; i++)
      if (_timer[i]->getMode(_canal[i]) != TIMER_OUTPUT_COMPARE_PWM1) return false;
    return true;
  }

  // ---------------- saída dos motores ----------------

  // Define os 4 motores em us. Antes de armar, segura MIN.
  void ESCRodar(int m1, int m2, int m3, int m4)
  {
    if (!_iniciado) return;
    if (!_armado) { parar(); return; }
    escrever(0, m1);
    escrever(1, m2);
    escrever(2, m3);
    escrever(3, m4);
  }

  // Um único motor (0 a 3): útil para testar um canal isolado
  void ESCRodarUm(uint8_t motor, int us)
  {
    if (!_iniciado || motor >= N) return;
    if (!_armado) { parar(); return; }
    escrever(motor, us);
  }

  void setAll(int us) { ESCRodar(us, us, us, us); }

  // Todos em MIN, independente do estado de arming
  void parar()
  {
    for (uint8_t i = 0; i < N; i++) escrever(i, ESC_MIN_US);
  }

  // Corta os motores e exige novo arming antes de aceitar throttle
  void desarmar()
  {
    parar();
    _armado = false;
    _armando = false;
  }

  bool armado() const { return _armado; }

  // ---------------- ARMING ----------------
  // O ESC precisa estar ENERGIZADO enquanto o sinal já está em MIN.

  // Versão sem delay(): chame armar_iniciar() uma vez e armar_atualizar() no loop
  void armar_iniciar()
  {
    parar();
    _armado = false;
    _armando = true;
    _tArm = millis();
  }

  // true quando terminou de armar
  bool armar_atualizar()
  {
    if (_armado) return true;
    if (!_armando) return false;
    parar();  // reforça MIN enquanto espera
    if (millis() - _tArm >= ESC_ARM_WAIT_MS)
    {
      _armado = true;
      _armando = false;
    }
    return _armado;
  }

  // Versão bloqueante (para o setup)
  void armarESC()
  {
    armar_iniciar();
    while (!armar_atualizar()) delay(1);
  }

  // ---------------- CALIBRAÇÃO ----------------
  // Ensina ao ESC a faixa MIN/MAX. Só quando necessário.
  // !!! TIRE AS HÉLICES ANTES !!!
  //  1. Sai MAX com a bateria do ESC DESCONECTADA.
  //  2. Você conecta a bateria e envia qualquer caractere por `io`.
  //  3. Segura MAX até os bips de confirmação do máximo.
  //  4. Vai para MIN e segura até os bips do mínimo.
  //  5. Termina em MIN e já armado.
  void calibrarESC(Stream &io = Serial)
  {
    if (!_iniciado) return;

    _armado = false;
    _armando = false;

    escreverTodos(ESC_MAX_US);
    io.println(F("CALIBRACAO: helices fora, bateria do ESC DESCONECTADA."));
    io.println(F("Enviando MAX. Conecte a bateria e envie qualquer caractere."));

    while (io.available()) io.read();
    while (!io.available()) delay(10);
    while (io.available()) io.read();

    io.println(F("Segurando MAX, aguarde os bips..."));
    delay(ESC_CAL_MAX_WAIT_MS);

    io.println(F("Enviando MIN..."));
    escreverTodos(ESC_MIN_US);
    delay(ESC_CAL_MIN_WAIT_MS);

    _armado = true;
    io.println(F("Calibracao concluida. ESC em MIN (armado)."));
  }

  // ---------------- DIAGNÓSTICO ----------------

  // channel = 1 a 4 (motor)
  uint32_t getPulseWidthUs(uint32_t channel)
  {
    if (!_iniciado || channel < 1 || channel > N) return 0;
    uint8_t i = channel - 1;
    if (!_ok[i]) return 0;
    return _timer[i]->getCaptureCompare(_canal[i], MICROSEC_COMPARE_FORMAT);
  }

  uint32_t getFrequencyHz()
  {
    if (!_iniciado || _nTimers == 0) return 0;
    return _timers[0]->getOverflow(HERTZ_FORMAT);
  }

  // Mostra, para cada motor: timer, canal e pulso lido de volta
  void imprimirMapa(Stream &io = Serial)
  {
    io.println(F("--- Mapa ESC ---"));
    for (uint8_t i = 0; i < N; i++)
    {
      io.print(F("M")); io.print(i + 1); io.print(F(": "));
      if (!_iniciado) { io.println(F("begin() nao chamado")); continue; }
      if (!_ok[i]) { io.println(F("FALHOU (sem PWM neste pino ou conflito de canal)")); continue; }
      io.print(nomeTimer(_inst[i]));
      io.print(F(" CH")); io.print(_canal[i]);
      io.print(F("  pulso=")); io.print(getPulseWidthUs(i + 1)); io.println(F("us"));
    }
    io.print(F("Freq=")); io.print(getFrequencyHz()); io.println(F("Hz"));
    io.print(F("Armado=")); io.println(_armado ? F("sim") : F("nao"));
  }

private:
  // Escreve um motor (0 a 3), limitado a MIN..MAX. Ignora canal que falhou.
  void escrever(uint8_t i, int us)
  {
    if (!_ok[i]) return;
    us = constrain(us, ESC_MIN_US, ESC_MAX_US);
    _timer[i]->setCaptureCompare(_canal[i], us, MICROSEC_COMPARE_FORMAT);
  }

  void escreverTodos(int us)
  {
    for (uint8_t i = 0; i < N; i++) escrever(i, us);
  }

  // Um HardwareTimer por timer real (motores no mesmo timer compartilham o objeto)
  HardwareTimer *obterTimer(TIM_TypeDef *inst)
  {
    for (uint8_t k = 0; k < _nTimers; k++)
      if (_instTimers[k] == inst) return _timers[k];
    if (_nTimers >= N) return nullptr;
    _timers[_nTimers] = new HardwareTimer(inst);
    _instTimers[_nTimers] = inst;
    return _timers[_nTimers++];
  }

  static const char *nomeTimer(TIM_TypeDef *i)
  {
#ifdef TIM1
    if (i == TIM1) return "TIM1";
#endif
#ifdef TIM2
    if (i == TIM2) return "TIM2";
#endif
#ifdef TIM3
    if (i == TIM3) return "TIM3";
#endif
#ifdef TIM4
    if (i == TIM4) return "TIM4";
#endif
#ifdef TIM5
    if (i == TIM5) return "TIM5";
#endif
#ifdef TIM8
    if (i == TIM8) return "TIM8";
#endif
    return "TIM?";
  }

  // por motor
  HardwareTimer *_timer[N] = {nullptr, nullptr, nullptr, nullptr};
  TIM_TypeDef   *_inst[N]  = {nullptr, nullptr, nullptr, nullptr};
  uint32_t       _canal[N] = {0, 0, 0, 0};
  bool           _ok[N]    = {false, false, false, false};

  // por timer real
  HardwareTimer *_timers[N]     = {nullptr, nullptr, nullptr, nullptr};
  TIM_TypeDef   *_instTimers[N] = {nullptr, nullptr, nullptr, nullptr};
  uint8_t        _nTimers = 0;

  uint8_t       _falhos = 0;
  bool          _iniciado = false;
  bool          _armado = false;
  bool          _armando = false;
  unsigned long _tArm = 0;
};

#endif // ARDUINO_ARCH_ESP32

#endif // META_DRONE_ESC_H
