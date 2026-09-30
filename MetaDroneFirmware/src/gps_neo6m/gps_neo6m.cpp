#include "gps_neo6m.h"

//construtor
GPS_NEO6M::GPS_NEO6M(Uart& serial, uint32_t baud)
    : _serial(serial), _baud(baud) {} //underline indica exclusivo e privado da classe, esse é o contrutor lembrando, setamos qual porta usamos e baud rate

void GPS_NEO6M::init()
{
    _serial.begin(_baud);
}

void GPS_NEO6M::atualizar()
{
    while (_serial.available())
        gps.encode(_serial.read()); 
}
