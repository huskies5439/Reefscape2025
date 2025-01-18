#include "Joystick.h"
#include "Adafruit_NeoPixel.h"
#include "OperateurIO.h"//Mettre le dossier dans un ZIP et inclure la librairie

#define LED_PIN 21
#define LED_COUNT 6

#define L1 2
#define L2 3
#define L3 4
#define L4 5

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   6, 0, false, false, false, // 6 est le nombre de boutons 
                   false, false, false,
                   false, false,
                   false, false, false);

OperateurIO hauteur(L1, L2, joystick, strip);
OperateurIO position(L3, L4, joystick, strip);

void setup() {
  
  strip.begin();
  strip.clear();
  strip.setBrightness(10);

  // intialise les boutons
   for (int i = L1; i <= L4; i++){
    pinMode(i, INPUT_PULLUP);
   }

  joystick.begin();

  Serial.begin(9600);
}

void loop() {
  hauteur.loopButtonsAndLEd();
  position.loopButtonsAndLEd();
}

