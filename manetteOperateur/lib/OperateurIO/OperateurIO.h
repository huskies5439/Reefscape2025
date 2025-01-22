#ifndef OperateurIO_h
#define OperateurIO_h

#include "Arduino.h"
#include <Joystick.h>
#include <Adafruit_NeoPixel.h>


class OperateurIO
{
  public:
    OperateurIO(int firstPin, int lastPin, Joystick_ *joystick, Adafruit_NeoPixel *strip);
    void loopBoutonEtLED();
  private:
    int _firstPin;
    int _lastPin;
    int _dernierBouton;
    int _dernierBoutonPasse;
    Joystick_ *_joystick;
    Adafruit_NeoPixel *_strip;
};

#endif