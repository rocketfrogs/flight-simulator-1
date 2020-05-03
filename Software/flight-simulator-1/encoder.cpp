#include <Arduino.h>
#include "encoder.h"

/////////////////////////////////////
// Private functions and variables //
/////////////////////////////////////
namespace {
  void ICACHE_RAM_ATTR pinA_rising_interrupt();
  void ICACHE_RAM_ATTR pinA_falling_interrupt();
  void ICACHE_RAM_ATTR pinB_rising_interrupt();
  void ICACHE_RAM_ATTR pinB_falling_interrupt();

  // We cannot call digitalRead() because the function is in flash and flash cannot be accessed from ISRs.
  volatile int pinA;
  volatile int pinB;
  volatile int position;

  // A position encoder is counting position using two sensors (two bits) between 0 and 3 using Gray code:
  // position | A | B
  //    0     | 0 | 0
  //    1     | 0 | 1
  //    2     | 1 | 1
  //    3     | 1 | 0

  void ICACHE_RAM_ATTR pinA_rising_interrupt()
  {
    pinA = 1;
    if (pinB) {
      position++;
    } else {
      position--;
    }
  }

  void ICACHE_RAM_ATTR pinA_falling_interrupt()
  {
    pinA = 0;
    if (pinB) {
      position--;
    } else {
      position++;
    }
  }

  void ICACHE_RAM_ATTR pinB_rising_interrupt()
  {
    pinB = 1;
    if (pinA) {
      position--;
    } else {
      position++;
    }
  }

  void ICACHE_RAM_ATTR pinB_falling_interrupt()
  {
    pinB = 0;
    if (pinA) {
      position++;
    } else {
      position--;
    }
  }
}

//////////////////////
// Public functions //
//////////////////////

void EncoderBegin(int pinA, int pinB)
{
  pinMode(pinA, INPUT);
  pinA = digitalRead(pinA);

  pinMode(pinB, INPUT);
  pinB = digitalRead(pinB);

  position = 0;

  attachInterrupt(digitalPinToInterrupt(pinA), pinA_rising_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(pinA), pinA_falling_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinB), pinB_rising_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(pinB), pinB_falling_interrupt, FALLING);
}

int EncoderGetMovement()
{
  noInterrupts();
  int ret = position;
  interrupts();
  return ret;
}
