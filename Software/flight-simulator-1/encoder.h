#pragma once
#include <Arduino.h>

static ICACHE_RAM_ATTR void encoder_interrupt();

class EncoderClass
{
  public:
    void begin(int pinA, int pinB, void (*up_callback)(), void (*down_callback)(), void (*error_callback)(), bool use_interrupts = true) {
      this->pinA = pinA;
      this->pinB = pinB;
      this->up_callback = up_callback;
      this->down_callback = down_callback;
      this->error_callback = error_callback;

      pinMode(pinA, INPUT);
      pinMode(pinB, INPUT);
      this->position = read_position();

      if (use_interrupts) {
        attachInterrupt(digitalPinToInterrupt(pinA), encoder_interrupt, CHANGE);
        attachInterrupt(digitalPinToInterrupt(pinB), encoder_interrupt, CHANGE);
      }
    }

    void interrupt() {
      int new_position = read_position();
      if (new_position == position) {
        // no change, ignore
        return;
      } else if (new_position == ((position + 1) & 0b11)) {
        up_callback();
      } else if (new_position == ((position - 1) & 0b11)) {
        down_callback();
      } else {
        // Encoder failure!
        error_callback();
      }
      position = new_position;
    }

  private:
    int pinA;
    int pinB;
    void (*up_callback)(void);
    void (*down_callback)(void);
    void (*error_callback)(void);

    // Encoder position (0 to 3)
    volatile int position;

    int read_position() {
      // A position encoder is counting position using two sensors (two bits) between 0 and 3 using Gray code:
      // position | A | B
      //    0     | 0 | 0
      //    1     | 0 | 1
      //    2     | 1 | 1
      //    3     | 1 | 0
      //
      // We use the following table to convert the binary value {b1,b0}={A,B} back to a position
      static const unsigned char PINS_TO_POSITION[4] = {0, 1, 3, 2};

      int a = digitalRead(pinA);
      int b = digitalRead(pinB);
      return PINS_TO_POSITION[a << 1 | b];
    }
};

EncoderClass Encoder;

static ICACHE_RAM_ATTR void encoder_interrupt() {
  Encoder.interrupt();
}
