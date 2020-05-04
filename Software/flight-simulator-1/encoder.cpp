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
  volatile int position;
  int pinA;
  int pinB;

  // A position encoder is counting position using two sensors (two bits) between 0 and 3 using Gray code:
  // position | A | B
  //    0     | 0 | 0
  //    1     | 0 | 1
  //    2     | 1 | 1
  //    3     | 1 | 0

  void ICACHE_RAM_ATTR pinA_interrupt()
  {
    int a = digitalRead(pinA);
    int b = digitalRead(pinB);

    // a b position
    // 0 0 ++
    // 0 1 --
    // 1 0 --
    // 1 1 ++

    if (a == b) {
      position++;
    } else {
      position--;
    }
  }

  void ICACHE_RAM_ATTR pinB_interrupt()
  {
    int a = digitalRead(pinA);
    int b = digitalRead(pinB);

    // a b position
    // 0 0 --
    // 0 1 ++
    // 1 0 ++
    // 1 1 --
    
    if (a != b) {
      position++;
    } else {
      position--;
    }
  }
}

//////////////////////
// Public functions //
//////////////////////

void EncoderBegin(int _pinA, int _pinB)
{
  pinA = _pinA;
  pinB = _pinB;
  pinMode(_pinA, INPUT);
  pinMode(_pinB, INPUT);
  position = 0;

  attachInterrupt(digitalPinToInterrupt(_pinA), pinA_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinB), pinB_interrupt, CHANGE);
}

int EncoderGetMovement()
{
  noInterrupts();
  int ret = position;
  position = 0;
  interrupts();
  return ret;
}
