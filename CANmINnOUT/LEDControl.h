
#ifndef LEDControl_h
#define LEDControl_h

#include <Arduino.h>      // for definition of byte datatype
#include <Streaming.h>

//
/// class to encapsulate a non-blocking LED
//

class LEDControl {

  public:
    LEDControl();
    void virtual setPin(byte pin, bool active = 1);
    void on();
    void off();
    void flash(unsigned int period);
    virtual void run();

  private:
    byte _pin;
    bool _active;
    bool _state;
    bool _flash;
    unsigned int _period;
    unsigned long _lastTime;
};

#endif
