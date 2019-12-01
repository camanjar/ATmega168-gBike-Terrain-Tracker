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
#include "lcd.h"

//-----Button Functions-----//
void button_init(); //Initialize button behavior
void interrupts_init(); //Initilize interrupts
ISR(INT0_vect); //Interrupt service routine
uint8_t debounce(void); //Debouncing function for button

//-----Biking Stats-----//
void riding_bike();
void gbike_greeting();

static int button_start = 1;
int flag = 1; //Flag for checking states
int has_started=0; //Flag for checking states
int time = 0;

int main(void) {

   //------Initializations-----//

   LCD_Init();
   button_init();
   interrupts_init();

   LCD_Clear();
   LCD_String("Hi from G-Bike!");
   _delay_ms(1000);
   LCD_String_xy(1,0, "Biking companion");
   _delay_ms(3000);

   LCD_Clear();
   LCD_String("Press button to");
   LCD_String_xy(1,0, "start ride! :)");
   while(button_start);

    return 0; 
}

//---------------BUTTON FUNCTIONALITY---------------//

void button_init() {
    DDRD &= ~(1<<PD2); //Double checks if we are in input mode (book)
    PORTD |= (1<<PD2); //Enbles pull-up resistor
}

//The following code has been applied from SER456 Slides 18
void interrupts_init() {
    EIMSK |= (1<< INT0); //Enabling interrupt INT0
    EICRA |= (1<<ISC00); //Configuring INT0
    sei(); //Activate all enabled interrupts
}

// Interupt Service Routine - 
// Runs code that will return the last character input to the computer
// as required for full assignment credit
ISR(INT0_vect) { //Runs when there is a button change
    if(debounce()) {
       PORTB ^= (1<<PB1);

       if(has_started) { //For continuous use
          gbike_greeting();
       }

       if(flag) { //Checks different states (first, used, etc.)

          flag = 0;
          has_started = 0;
          riding_bike(); //Main communication prompt

       } else { 

          LCD_Clear();
          LCD_String("Thanks for");
          LCD_String_xy(1,0, "riding! :)");
          flag = 1;
          has_started = 1;
       }
    }
}

//SOURCE: MAKE: AVR Programming by Elliot Williams
uint8_t debounce(void) {
    if (bit_is_clear(PIND, PD2)) { //When button is pressed
        _delay_us(1000); //delay 1000 miliseconds to ignore noise
        if (bit_is_clear(PIND, PD2)) { //return "true" if button is still noise
            return 1;
        }
    }
    return 0; //break out if button is ready
}

void riding_bike() { //Main communication prompt for user
   button_start = 0;
   LCD_Clear();
   LCD_String("Ride starting in");
   LCD_String_xy(1,8, "3");
   _delay_ms(1000);
   LCD_String_xy(1,8, "2");
   _delay_ms(1000);
   LCD_String_xy(1,8, "1");
   _delay_ms(1000);
   LCD_Clear();
   LCD_String("Start!");
   _delay_ms(1000);
   gbike_main_display();
}

void gbike_greeting() { //Greets user and prompts them for button press
   LCD_Clear();
   LCD_String("Hi from G-Bike!");
   _delay_ms(1000);
   LCD_String_xy(1,0, "Biking companion");
   _delay_ms(3000);

   LCD_Clear();
   LCD_String("Press button to");
   LCD_String_xy(1,0, "start ride! :)");
   while(button_start);
}

void gbike_main_display() {
   LCD_Clear();
   LCD_String("Terrain: ");
   LCD_String_xy(1,0, "Time: ");

   while (time < 10) {
      time++;
      char* temp = (char*) time;
      LCD_String_xy(0,9, "Rocky");
      LCD_String_xy(1,6, "60s");
      _delay_ms(1000);
   }

   LCD_Clear();
   LCD_String("Average terrain: ");
   LCD_String_xy(1,6, "Rocky");
   
}