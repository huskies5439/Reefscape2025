#include "OperateurIO.h"


/* Voici le site qui a permit de faire fonctionner le joystick
https://stackoverflow.com/questions/76930785/c-arduino-pass-one-objects-instance-as-a-pointer-to-another-class
*/

//TODO : Mieux commenter la classe

//Le * peut être soit collé sur le type, soit sur le nom de la variable.
OperateurIO::OperateurIO(int firstChannel, int lastChannel, Joystick_ *joystick, Adafruit_NeoPixel *strip, CD74HC4067 *mux) {
  _firstChannel = firstChannel;
  _lastChannel = lastChannel;
  
  _joystick = joystick; //pas mettre le * car le nom de la variable du pointeur est seulement joystick, pas *joystick.
  _strip = strip;
  _mux = mux;

  _dernierBouton = -1;
  _dernierBoutonPasse = -1;
}


void OperateurIO::loopBoutonEtLED() {
  //trouver dernier bouton
  for (int i = _firstChannel; i <= _lastChannel; i++) {
    _mux->channel(i);
    if (!digitalRead(SIG)) {
      _dernierBouton = i;
    }
  }
  
  
  //determiner si le bouton vient de changer
  if (_dernierBouton != _dernierBoutonPasse) {
    //On itère sur tous les boutons pour les allumer/éteindre dans la driver station
    //et les LED correspondantes
    for (int i = _firstChannel; i <= _lastChannel; i++){
      
      if (i == _dernierBouton) {//Allume le dernier bouton
        _joystick -> setButton(i, 1); //Il faut utiliser l'opérateur lambda -> car on passe par un pointeur 
        _strip -> setPixelColor(i, 0, 255, 0);

      } else {//Éteint tous les autres
        _joystick -> setButton(i, 0);
        _strip -> setPixelColor(i, 0, 0, 0);

      }
    }
    _strip -> show();
  }

  //reinitialise dernier bouton
  _dernierBoutonPasse = _dernierBouton;
}