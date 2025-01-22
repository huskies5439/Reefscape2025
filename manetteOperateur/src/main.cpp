#include <Arduino.h>

#include <Joystick.h>
#include <Adafruit_NeoPixel.h>

#include <OperateurIO.h>
#include <DefinitionPins.h>


#define NB_BOUTON 4


Adafruit_NeoPixel strip(NB_BOUTON, LED_PIN, NEO_GRB + NEO_KHZ800);

//Sert à communiquer avec la Driver Station
Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   NB_BOUTON + 2 /* +2 car on crée 0 et 1, mais on ne les utilise pas*/, 0, false, false, false,
                   false, false, false,
                   false, false,
                   false, false, false);

//firstPin, lastPin, le joystick et les del
//Il faut pointer vers le joystick car on veut que chaque objet OperateurIO parle au même objet joystick
//Même chose pour la LED strip
OperateurIO hauteur(L1, L2, &joystick, &strip);
OperateurIO position(L3, L4, &joystick, &strip);

void setup()
{

    // intialise les boutons
    for (int i = L1; i <= L4; i++)
    {
        pinMode(i, INPUT_PULLUP);
    }

    //Initialise le ruban de DEL
    strip.begin();
    strip.setBrightness(10);

    //Reset la strip au début du programme
    strip.clear();
    strip.show();
    

    //Initiatilise le joystick
    joystick.begin();

    // Serial.begin(9600);
}

void loop()
{
    //Input : Itère sur tous les boutons, vérifie le dernier appuyé
    //Output : Envoie le dernier bouton appuyé à la driver station et aux LEDs
    hauteur.loopBoutonEtLED();
    position.loopBoutonEtLED();

}
