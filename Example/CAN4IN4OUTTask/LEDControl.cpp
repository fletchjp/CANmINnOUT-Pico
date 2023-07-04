
#include "LEDControl.h"

//
/// class for individual LED with non-blocking control
//

LEDControl::LEDControl() {

  _state = LOW;
  _flash = false;
  _lastTime = 0UL;
}

//  set the pin for this LED

void LEDControl::setPin(byte pin, bool active) {

  _pin = pin;
  pinMode(_pin, OUTPUT);
  _active = active;
}

// turn LED state on

void LEDControl::on(void) {

  if (_active) {
    _state = HIGH;
  } else {
    _state = LOW;
  }
  _flash = false;
}

// turn LED state off

void LEDControl::off(void) {

  if (_active) {
    _state = LOW;
  } else {
    _state = HIGH;
  }
  _flash = false;
}

// blink LED

void LEDControl::flash(unsigned int period) {

  _flash = true;
  _period = period;
}

// actually operate the LED dependent upon its current state
// must be called frequently from loop() if the LED is set to blink or pulse

void LEDControl::run() {

  if (_flash) {

    // blinking
    if ((millis() - _lastTime) >= _period) {
      _state = !_state;
      _lastTime = millis();
    }
  }

  digitalWrite(_pin, _state);
}
