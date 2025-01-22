#ifndef OperateurIO_h
#define OperateurIO_h

#include "Arduino.h"
#include <Joystick.h>
#include <Adafruit_NeoPixel.h>
#include <CD74HC4067.h>


class OperateurIO
{
  public:
    OperateurIO(int firstPin, int lastPin, Joystick_ *joystick, Adafruit_NeoPixel *strip, CD74HC4067 *mux);
    void loopBoutonEtLED();
  private:
    int _firstChannel;
    int _lastChannel;
    int _dernierBouton;
    int _dernierBoutonPasse;
    Joystick_ *_joystick;
    Adafruit_NeoPixel *_strip;
    CD74HC4067 *_mux;
};

#endif