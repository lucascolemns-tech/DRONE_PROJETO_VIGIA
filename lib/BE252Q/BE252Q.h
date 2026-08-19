//datasheet:
#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h> //Desenvolver nossa própria comunicação serial à 115200
    /*
    É adicionados os seguintes #define a partir dessa biblioteca: 

        UDR0	    Transmit/Receive data buffer
        UCSR0A	    Status: RXC0, TXC0, UDRE0, frame error, parity error
        UCSR0B	    Enable: RXEN0, TXEN0, RXCIE0, TXCIE0, UDRIE0
        UCSR0C	    Frame format: UCSZ (data bits), UPM (parity), USBS (stop bits)
        UBRR0H      (byte mais significativo)  Baud rate register (12-bit)
        UBBR0L	    (byte menos significativo) Baud rate register (12-bit)
    */

#define BE252Q_H
#ifdef BE252Q_H

//I2C 
#define BE_252Q_ID 0x0D
#define BE_252Q_ADRESS 0x0C 

//UART 115200 para Atmega328p
#define F_CPU 16000000UL 
#define BAUD 115200UL
#define UBRR_VAL ((F_CPU / (8UL * BAUD)) - 1) // U2X0 mode
    /*
    formula para calcular a velocidade de comunicação
    disponível em https://siliconwit.com/education/embedded-programming-atmega328p/uart-serial-communication/
    */

class BE252Q
{
public:
    
    //I2C
    uint8_t lerRegistradorI2C(uint8_t endereco);
    void escreverRegistradorI2C(uint8_t endereco, uint8_t valor);

    //USART
    void UART_BE252Q();
    void inicilizar_USART();
    uint8_t lerUSART();
    void escreverUSART(uint8_t valor);
    
    private:
};

#endif
