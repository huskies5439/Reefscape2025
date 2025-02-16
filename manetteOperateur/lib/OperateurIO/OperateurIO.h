#ifndef OperateurIO_h
#define OperateurIO_h

#include "Arduino.h"
#include <Joystick.h>
#include <Adafruit_NeoPixel.h>
#include <CD74HC4067.h>

#include <../../include/DefinitionPins.h>


class OperateurIO
{
  public:
    OperateurIO(int firstPin, int lastPin, Joystick_ *joystick, Adafruit_NeoPixel *strip, CD74HC4067 *mux);
    void lireBouton();
    bool isChangementBouton();
    int getDernierBouton();
    void actionsBouton(int idxBouton);
    void reset();
  private:
    int _firstChannel;
    int _lastChannel;
    int _dernierBouton;
    int _dernierBoutonPasse;
    bool _changementBouton;
    Joystick_ *_joystick;
    Adafruit_NeoPixel *_strip;
    CD74HC4067 *_mux;
};

#endif