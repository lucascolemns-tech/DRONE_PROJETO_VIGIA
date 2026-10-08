#include "_AUX_STM/auxiliarSTM.h"

AUX_STM firmwareSTM;

void setup()
{
    firmwareSTM.iniciarSTM();
}

void loop()
{
    firmwareSTM.rodarSTM();
}
