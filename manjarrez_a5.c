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
//#include <i2c/i2c.h>
#include "pinDefines.h"
#include <avr/io.h>      //Macros, ports, registers, etc.
#include <util/delay.h>  //Library for seconds   
#include <util/setbaud.h>
#include <avr/interrupt.h> //Library for interrupts

//---------- GLOBAL FUNCTIONS & VARIABLES ----------//

uint8_t serial_info; //Char being read & written
uint8_t temp;

//Code for init_uart is authered by Dr. Robert Heinrichs from SER456 Slides 14
//-- vvImplementation of foward declarations below main vv --//
void init_uart(); //Initialize UART
uint8_t read_uart(void); // Retrieves serial information 
void write_uart(uint8_t data); //Assigns char to UDR0 (USART Data Register)
void check_input(uint8_t n); //Breadboard behavior (RGB LED/ 7 Display)
void i2c_accelerometer();

//i2c functions 
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

//UART FUNCTIONS
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

void readString(char myString[], uint8_t maxLength) {
  char response;
  uint8_t i;
  i = 0;
  while (i < (maxLength - 1)) {                   /* prevent over-runs */
    response = receiveByte();
    transmitByte(response);                                    /* echo */
    if (response == '\r') {                     /* enter marks the end */
      break;
    }
    else {
      myString[i] = response;                       /* add in a letter */
      i++;
    }
  }
  myString[i] = 0;                          /* terminal NULL character */
}

void printByte(uint8_t byte) {
              /* Converts a byte to a string of decimal text, sends it */
  transmitByte('0' + (byte / 100));                        /* Hundreds */
  transmitByte('0' + ((byte / 10) % 10));                      /* Tens */
  transmitByte('0' + (byte % 10));                             /* Ones */
}

void printWord(uint16_t word) {
  transmitByte('0' + (word / 10000));                 /* Ten-thousands */
  transmitByte('0' + ((word / 1000) % 10));               /* Thousands */
  transmitByte('0' + ((word / 100) % 10));                 /* Hundreds */
  transmitByte('0' + ((word / 10) % 10));                      /* Tens */
  transmitByte('0' + (word % 10));                             /* Ones */
}

void printBinaryByte(uint8_t byte) {
                       /* Prints out a byte as a series of 1's and 0's */
  uint8_t bit;
  for (bit = 7; bit < 255; bit--) {
    if (bit_is_set(byte, bit))
      transmitByte('1');
    else
      transmitByte('0');
  }
}

char nibbleToHexCharacter(uint8_t nibble) {
                                   /* Converts 4 bits into hexadecimal */
  if (nibble < 10) {
    return ('0' + nibble);
  }
  else {
    return ('A' + nibble - 10);
  }
}

void printHexByte(uint8_t byte) {
                        /* Prints a byte as its hexadecimal equivalent */
  uint8_t nibble;
  nibble = (byte & 0b11110000) >> 4;
  transmitByte(nibbleToHexCharacter(nibble));
  nibble = byte & 0b00001111;
  transmitByte(nibbleToHexCharacter(nibble));
}

uint8_t getNumber(void) {
  // Gets a numerical 0-255 from the serial port.
  // Converts from string to number.
  char hundreds = '0';
  char tens = '0';
  char ones = '0';
  char thisChar = '0';
  do {                                                   /* shift over */
    hundreds = tens;
    tens = ones;
    ones = thisChar;
    thisChar = receiveByte();                   /* get a new character */
    transmitByte(thisChar);                                    /* echo */
  } while (thisChar != '\r');                     /* until type return */
  return (100 * (hundreds - '0') + 10 * (tens - '0') + ones - '0');
}

int main(void) {

    init_uart(); //Initialize USART
    initI2C();
    

    while(1) {
        
        i2c_accelerometer();
        printByte(serial_info);
        printByte(temp);
        _delay_ms(1000);
        //printString("\n");
        
        //serial_info= read_uart(); //Recieve serial information
        //write_uart(serial_info); //Transmit serial information
        //check_input(serial_info); //Provide behavior to breadboard (RGB-LED, 7-Display )

    }

    return 0;
}

void i2c_accelerometer() {
    i2cStart();
    i2cSend(ADXL345_ADDRESS_W);
    i2cSend(0x2D);
    i2cSend(1 << 3);
    i2cSend(ADXL345_ADDRESS_R);
    serial_info = i2cReadAck;
    temp = i2cReadNoAck;
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


