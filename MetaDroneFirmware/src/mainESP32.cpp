#include "_AUX_ESP/auxiliarESP.h"

AUX_ESP firmwareESP;

void setup()
{
    firmwareESP.iniciar();
}

void loop()
{
    firmwareESP.rodar();
}
