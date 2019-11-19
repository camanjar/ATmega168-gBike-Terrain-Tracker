/*
//Author: Cesar Manjarrez, Dr. Robert Heinrichs, Elliot Williams 
Assignment: 5
Description: The following program uses the I@C bus to interface with an intelligent I2C sensor, 
accelerometer ADXL345. The serial communication aspect of the last assignment is used
in this program as well. 
 */


// ------- Preamble ------- //
#define __AVR_ATmega168__ 
#define BAUD 9600
#define ADXL345_ADDRESS_W 0b10100110
#define ADXL345_ADDRESS_R 0b10100111
#include "pinDefines.h"
#include <avr/io.h>      //Macros, ports, registers, etc.
#include <util/delay.h>  //Library for seconds   
#include <util/setbaud.h>
#include <avr/interrupt.h> //Library for interrupts

//---------- GLOBAL FUNCTIONS & VARIABLES ----------//

//Code for init_uart is authered by Dr. Robert Heinrichs from SER456 Slides 14
//-- vvImplementation of foward declarations below main vv --//

void init_uart(); //Initialize UART
uint8_t read_uart(void); // Retrieves serial information 
void write_uart(uint8_t data); //Assigns char to UDR0 (USART Data Register)
void check_input(uint8_t n); //Breadboard behavior (RGB LED/ 7 Display)
void i2c_accelerometer();

uint16_t x, y, z; //Measurements taken from accelerometer

//i2c Functions (from Canvas)
void initI2C(void);
void i2cWaitForComplete(void);
void i2cStart(void); 
void i2cStop(void);
uint8_t i2cReadAck(void);
uint8_t i2cReadNoAck(void);
void i2cSend(uint8_t data);

//USART functions (from Canvas)
void transmitByte(uint8_t data); 
uint8_t receiveByte(void);
void printString(const char myString[]); 
void printByte(uint8_t byte);

void read_accel_data(); //Read information from sensor
void write_accel_data(uint8_t x,uint8_t y,uint8_t z); //Write information to serial output

int main(void) {

    init_uart(); //Initialize USART
    initI2C(); //Initialize i2c
    i2c_accelerometer(); //i2c setup with accelerometer

    while(1) {
      read_accel_data(); //Read sensor information
      write_accel_data(x, y, z); //write x, y, z values accordingly
      _delay_ms(1000); //One second delay
    }

    return 0;
}

void i2c_accelerometer() {
    i2cStart(); //Init serial communication 
    i2cSend(ADXL345_ADDRESS_W); //Begin communication with sensor
    i2cSend(0x2D); //POWER_CTL 
    i2cSend(1 << 3); //D3 set to high (measure)
    i2cStop(); 
}
//Code for init_uart is authered by Dr. Robert Heinrichs from SER456 Slides 14
void init_uart() {
    //------ UART INIT ------//
    UBRR0H = UBRRH_VALUE; //UBRRH_VALUE
    UBRR0L = UBRRL_VALUE; //UBRRL_VALUE

    #if USE_2X 
        UCSR0A |= (1 << U2X0); 
    #else 
        UCSR0A &= ~(1 << U2X0);
    #endif

    //Enable USART transmitter/reciever 
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);

    //Set packet size (8 data bits)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    //Set stop bit amount (1 stop bit)
    UCSR0C &= ~(1 << USBS0);
}

//Read values from accelerometer (I have no idea...)
void read_accel_data() {
  i2cStart();
  i2cSend(ADXL345_ADDRESS_W); //Begin communication with sensor
  i2cSend(0x32);
  i2cStart();
  i2cSend(ADXL345_ADDRESS_R);
  x = i2cReadAck();
  y = i2cReadAck();
  z = i2cReadNoAck();
  i2cStop();
}

//The following formats the serial output to a reader-friendly format
void write_accel_data(uint8_t x,uint8_t y,uint8_t z) { 
  printString("x = ");
  printByte(x);
  printString(", y = ");
  printByte(y);
  printString(", z = ");
  printByte(z);
  printString("\n\r");
}


//I2C FUNCTION 
void initI2C(void) {
                                     /* set pullups for SDA, SCL lines */
  I2C_SDA_PORT |= ((1 << I2C_SDA) | (1 << I2C_SCL));
  TWBR = 32;   /* set bit rate (p.242): 8MHz / (16+2*TWBR*1) ~= 100kHz */
  TWCR |= (1 << TWEN);                                       /* enable */
}

void i2cWaitForComplete(void) {
  loop_until_bit_is_set(TWCR, TWINT);
}

void i2cStart(void) {
  TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWSTA));
  i2cWaitForComplete();
}

void i2cStop(void) {
  TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWSTO));
}

uint8_t i2cReadAck(void) {
  TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWEA));
  i2cWaitForComplete();
  return (TWDR);
}

uint8_t i2cReadNoAck(void) {
  TWCR = (_BV(TWINT) | _BV(TWEN));
  i2cWaitForComplete();
  return (TWDR);
}

void i2cSend(uint8_t data) {
  TWDR = data;
  TWCR = (_BV(TWINT) | _BV(TWEN));                  /* init and enable */
  i2cWaitForComplete();
}

//UART
void transmitByte(uint8_t data) {
                                     /* Wait for empty transmit buffer */
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = data;                                            /* send data */
}

uint8_t receiveByte(void) {
  loop_until_bit_is_set(UCSR0A, RXC0);       /* Wait for incoming data */
  return UDR0;                                /* return register value */
}


                       /* Here are a bunch of useful printing commands */

void printString(const char myString[]) {
  uint8_t i = 0;
  while (myString[i]) {
    transmitByte(myString[i]);
    i++;
  }
}

void printByte(uint8_t byte) {
              /* Converts a byte to a string of decimal text, sends it */
  transmitByte('0' + (byte / 100));                        /* Hundreds */
  transmitByte('0' + ((byte / 10) % 10));                      /* Tens */
  transmitByte('0' + (byte % 10));                             /* Ones */
}