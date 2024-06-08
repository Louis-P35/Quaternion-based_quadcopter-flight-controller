#include <Arduino.h>

#include "readPWM.h"

// Store the start and duration time of the high signal for the 4 channel
volatile unsigned long g_pulseStart[4] = {0, 0, 0, 0};
volatile unsigned long g_pulseDuration[4] = {0, 0, 0, 0};

// The 4 PMW's channels
static uint8_t g_channelPins[4] = {0, 0, 0, 0};

void setup_PWM_reader(uint8_t channel0, uint8_t channel1, uint8_t channel2, uint8_t channel3)
{
  /* Configure interrupt for radio receiver reading */
  pinMode(channel0, INPUT);
  pinMode(channel1, INPUT);
  pinMode(channel2, INPUT);
  pinMode(channel3, INPUT);

  /* Remember the 4 channels */
  g_channelPins[0] = channel0;
  g_channelPins[1] = channel1;
  g_channelPins[2] = channel2;
  g_channelPins[3] = channel3;

  /* Attach interrupts */
  attachInterrupt(digitalPinToInterrupt(channel0), readPWM0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel1), readPWM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel2), readPWM2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel3), readPWM3, CHANGE);
}

/* functions to read PWM signal, called at any state change (high to low or low to high) */
void handle_PWM_read(uint8_t pin, int channel)
{
  if (digitalRead(pin) == HIGH)
  {
    g_pulseStart[channel] = micros();
  }
  else
  {
    g_pulseDuration[channel] = micros() - g_pulseStart[channel];
  }
}

/* ISR functions. Must be static instances */
void readPWM0()
{
  handle_PWM_read(g_channelPins[0], 0);
}

void readPWM1()
{
  handle_PWM_read(g_channelPins[1], 1);
}

void readPWM2()
{
  handle_PWM_read(g_channelPins[2], 2);
}

void readPWM3()
{
  handle_PWM_read(g_channelPins[3], 3);
}


/* Interrupt safe getter */
unsigned long getRadioChannel(int channel)
{
  /* Temporarily disable interrupts to ensure consistent readings */
  // TODO: Only disable PWM's interrupts ?
  noInterrupts();
  unsigned long durationTmp = g_pulseDuration[channel];
  interrupts(); // Re-enable interrupts

  return durationTmp;
}


