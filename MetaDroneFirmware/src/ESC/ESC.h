#ifndef ESC_STM32_H
#define ESC_STM32_H

#include <Arduino.h>
#include <HardwareTimer.h>

#define PINO_M1 PA0
#define PINO_M2 PA1
#define PINO_M3 PA2
#define PINO_M4 PA3

//definimos a frequência mínima da biblioteca pro stm32
#define ESC_FREQ_HZ 50

#define ESC_MIN_US 1050
#define ESC_MAX_US 2000

class ESC_STM32
{
public:
  ESC_STM32() : _timer(new HardwareTimer(TIM2)) {} //construtor
  ~ESC_STM32() { delete _timer; } //destrutor, evita memory leak nos STM, pois é como se você alocasse uma memory insana pelo construtor e nunca a liberasse, tornando essa peça aqui importante em um futuro debug
  
  void begin()
  {
    //inicialização da biblioteca hardware timer
    _timer->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PINO_M1);
    _timer->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PINO_M2);
    _timer->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PINO_M3);
    _timer->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PINO_M4);

    _timer->setOverflow(ESC_FREQ_HZ, HERTZ_FORMAT);

    _timer->setCaptureCompare(1, ESC_MIN_US, MICROSEC_COMPARE_FORMAT);
    _timer->setCaptureCompare(2, ESC_MIN_US, MICROSEC_COMPARE_FORMAT);
    _timer->setCaptureCompare(3, ESC_MIN_US, MICROSEC_COMPARE_FORMAT);
    _timer->setCaptureCompare(4, ESC_MIN_US, MICROSEC_COMPARE_FORMAT);

    _timer->resume();
  }

  void ESCRodar(int m1, int m2, int m3, int m4)
  {
    m1 = constrain(m1, ESC_MIN_US, ESC_MAX_US); // não deixa os valores fora da faixa mínimo-maximo
    m2 = constrain(m2, ESC_MIN_US, ESC_MAX_US);
    m3 = constrain(m3, ESC_MIN_US, ESC_MAX_US);
    m4 = constrain(m4, ESC_MIN_US, ESC_MAX_US);

    //modo hardware timer
    _timer->setCaptureCompare(1, m1, MICROSEC_COMPARE_FORMAT);
    _timer->setCaptureCompare(2, m2, MICROSEC_COMPARE_FORMAT);
    _timer->setCaptureCompare(3, m3, MICROSEC_COMPARE_FORMAT);
    _timer->setCaptureCompare(4, m4, MICROSEC_COMPARE_FORMAT);
  }

  uint32_t getPulseWidthUs(uint32_t channel)
  {
    if (channel < 1 || channel > 4)
      return 0;
    return _timer->getCaptureCompare(channel, MICROSEC_COMPARE_FORMAT);
  }

  uint32_t getFrequencyHz()
  {
    return _timer->getOverflow(HERTZ_FORMAT);
  }

  bool pwmChannelsConfigured()
  {
    return _timer->getMode(1) == TIMER_OUTPUT_COMPARE_PWM1 &&
           _timer->getMode(2) == TIMER_OUTPUT_COMPARE_PWM1 &&
           _timer->getMode(3) == TIMER_OUTPUT_COMPARE_PWM1 &&
           _timer->getMode(4) == TIMER_OUTPUT_COMPARE_PWM1;
  }

  void armarESC()
  {
    ESCRodar(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
    delay(8000); 
  }

private:
  HardwareTimer *_timer; //objeto
};

#endif
