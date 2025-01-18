#include "Arduino.h"
#include "OperateurIO.h"

OperateurIO::OperateurIO(int firstPin, int lastPin, Joystick_ joystick, Adafruit_NeoPixel strip) {
  _firstPin = firstPin;
  _lastPin = lastPin;
  _joystick = joystick;
  _strip = strip;
  _dernierBouton = 0;
  _dernierBoutonPasse = 0;
}

void OperateurIO::loopBoutonEtLED() {
  //trouver dernier bouton
  for (int i = _firstPin; i <= _lastPin; i++) {
    if (!digitalRead(i)) {
      _dernierBouton = i;
    }
  }
  
  
  //determiner si le bouton vient de changer
  if (_dernierBouton != _dernierBoutonPasse) {
    //On itère sur tous les boutons pour les allumer/éteindre dans la driver station
    //et les LED correspondantes
    for (int i = _firstPin; i <= _lastPin; i++){
      
      if (i == _dernierBouton) {//Allume le dernier bouton
        _joystick.setButton(i, 1);
        _strip.setPixelColor(i-2, 0, 255, 0);

      } else {//Éteint tous les autres
        _joystick.setButton(i, 0);
        _strip.setPixelColor(i-2, 0, 0, 0);

      }
    }
    _strip.show();
  }

  //reinitialise dernier bouton
  _dernierBoutonPasse = _dernierBouton;
  delay(20);
}