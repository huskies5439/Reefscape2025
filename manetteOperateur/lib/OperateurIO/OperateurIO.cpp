#include "Arduino.h"
#include "OperateurIO.h"

/* Voici le site qui a permit de faire fonctionner le joystick
https://stackoverflow.com/questions/76930785/c-arduino-pass-one-objects-instance-as-a-pointer-to-another-class

Mais
1) On peut écrire Joystick_* joystick (stackoverflow) OU Joystick_ *joystick (doc arduino) pour indiquer un pointeur et ça ne change rien ! Pourquoi ?
2) D'où vient la syntaxe ": _joystick(joystick)" dans le constructeur ?? J'ai fait la même chose pour la manette du Qbot ? Où est la doc pour ça ?
3) pourquoi joystick->setButton ? C'est un opérateur lambda ? On passe la fonction d'une classe à une autre ?
*/

OperateurIO::OperateurIO(int firstPin, int lastPin, Joystick_ *joystick, Adafruit_NeoPixel strip) : _joystick(joystick) {
  _firstPin = firstPin;
  _lastPin = lastPin;
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
        _joystick -> setButton(i, 1);
        _strip.setPixelColor(i-2, 0, 255, 0);

      } else {//Éteint tous les autres
        _joystick -> setButton(i, 0);
        _strip.setPixelColor(i-2, 0, 0, 0);

      }
    }
    _strip.show();
  }

  //reinitialise dernier bouton
  _dernierBoutonPasse = _dernierBouton;
  delay(20);
}