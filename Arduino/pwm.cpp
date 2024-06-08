#include <Arduino.h>

#include "pwm.h"



void setupPWM()
{
  // Set the PWMs pin as output
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(45, OUTPUT);

  // Configure timers and PWM mode
  setupPwmTimer1();
  setupPwmTimer5();
}


void setupPwmTimer1()
{
  // PWM frequency = Clock frequency / (Prescaler * (1 + Top))
  // For Arduino Mega Clock frequency is 16 Mhz

  // Set up Timer 1
  // TCCR1A and TCCR1B: These are Timer/Counter Control Registers for Timer 1
  TCCR1A = 0;          // Clear control register A
  TCCR1B = 0;          // Clear control register B
  TCNT1 = 0;           // Initialize counter value to 0
  
  // Set to WGM Mode 14 (Fast PWM)
  // Waveform Generation Mode
  // Fast PWM: The counter counts from 0 up to a maximum value specified in the ICR1 register
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  
  // Set prescaler to 1 (no prescaling)
  // The prescaler divide the frequency of the main clock
  // before it is used by the timer module.
  TCCR1B |= (1 << CS10);  // Set prescaler to 1: CS12 = 0, CS11 = 0, CS10 = 1
  
  // Set ICR1 for Top value
  // Input Capture Register 1
  // In fast PWM mode (WGM Mode 14), the timer counts from zero up to 
  // the value in ICR1, then resets to zero and starts over.
  // It is a 16-bits register (0 to 65535)
  ICR1 = 31999;
  
  // Enable PWM on Pin 11 and 12
  /*
  Setting COM1A1 and COM1B1 to 1 configures the output compare pins to clear on 
  compare match when the timer is counting up, and set on compare match when the timer 
  is counting down in Fast PWM mode. This results in non-inverting PWM output on these pins. 
  */
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
  DDRB |= (1 << PB5) | (1 << PB6); // PB5 is Pin 11, PB6 is Pin 12
}


void setupPwmTimer5()
{
  /*
  Setup the timer 5 for 500Hz PWM the same way
  as the setupPwmTimer1() function
  */

  TCCR5A = 0; // Clear control register A
  TCCR5B = 0; // Clear control register B
  TCNT5 = 0;  // Initialize counter value to 0

  // Set to WGM Mode 14 (Fast PWM)
  TCCR5A |= (1 << WGM51);
  TCCR5B |= (1 << WGM52) | (1 << WGM53);

  // Set prescaler to 1 (no prescaling)
  TCCR5B |= (1 << CS50);

  // Set ICR5 for Top value
  ICR5 = 31999;

  // Enable PWM on Pin 44 and 45
  TCCR5A |= (1 << COM5C1) | (1 << COM5B1); // Configure as non-inverting
  DDRL |= (1 << PL3) | (1 << PL4); // PL3 is Pin 45, PL4 is Pin 44
}

void setPwmPin11(unsigned int Ton)
{
  // Set duty cycle for pin 11
  OCR1A = Ton;
}

void setPwmPin12(unsigned int Ton)
{
  // Set duty cycle for pin 12
  OCR1B = Ton;
}

void setPwmPin44(unsigned int Ton)
{
  // Set duty cycle for pin 44
  OCR5C = Ton;
}

void setPwmPin45(unsigned int Ton)
{
  // Set duty cycle for pin 45
  OCR5B = Ton;
}






