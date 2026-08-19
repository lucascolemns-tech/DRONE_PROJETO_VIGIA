#include "BE252Q.h"

uint8_t BE252Q::lerRegistradorI2C(uint8_t endereco)
{
    Wire.beginTransmission(endereco);
    Wire.write(BE_252Q_ID);

    Wire.endTransmission(false);

    Wire.requestFrom(endereco, 1);

    if (Wire.available())
        return Wire.read();

    return 0xFF;
}

void BE252Q::escreverRegistradorI2C(uint8_t endereco, uint8_t valor)
{
    Wire.beginTransmission(endereco);

    Wire.write(valor);

    Wire.endTransmission(false);
}

void BE252Q::inicilizar_USART()
{
    UBRR0H = ((int8t_t)(UBRR_VAL >> 8)) 
    UBRR0L = ((int8t_t) UBRR_VAL ) 

    //comandos para habilitação do transmissor e receptor, seguindo formula documentada pela https://siliconwit.com/
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);

    //definição do modo de operaçaõ, seguindo formula documentada pela https://siliconwit.com/
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void BE252Q::escreverUSART(uint8_t valor)
{   
    while (!(UCSR0A & (1 << UDRE0))) {
     // Espera por registrador    
    }  

    UDR0 = valor;
}

uint8_t BE252Q::lerUSART()
{
    while (!(UCSR0A & (1 << RXC0))) {
    // Espera por dados no receptor    
    }   

    return UDR0; 
}

void BE252Q::UART_BE252Q()
{


}
