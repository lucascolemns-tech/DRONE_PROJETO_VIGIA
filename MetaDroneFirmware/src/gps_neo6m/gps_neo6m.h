#ifndef GPS_NEO6M_H
#define GPS_NEO6M_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

class GPS_NEO6M
{
public:
    GPS_NEO6M(Uart& serial, uint32_t baud = 9600);
    void init();
    void atualizar();
    bool checar() const { return gps.location.isValid() && gps.location.age() < 2000 &&
                                 gps.altitude.isValid() && gps.altitude.age() < 2000; }
    double obter_lat() { return gps.location.lat(); }
    double obter_long() { return gps.location.lng(); }
    double obter_alt() { return gps.altitude.meters(); }
    int obter_sat() { return gps.satellites.value(); }
    uint32_t obter_idade() { return gps.location.age(); }

private:
    Uart& _serial;
    uint32_t _baud;
    TinyGPSPlus gps;
};

#endif
