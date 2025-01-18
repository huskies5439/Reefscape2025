#include <Arduino.h>
#include <Joystick.h>
#include <Adafruit_NeoPixel.h>
#include <OperateurIO.h>

#define LED_PIN 21 //A3
#define LED_COUNT 4

#define L1 2
#define L2 3
#define L3 4
#define L4 5

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//Sert à communiquer avec la Driver Station
Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   4 /* <- c'est le nombre de boutons*/, 0, false, false, false,
                   false, false, false,
                   false, false,
                   false, false, false);

//firstPin, lastPin, le joystick et les del
OperateurIO hauteur(L1, L2, joystick, strip);
OperateurIO position(L3, L4, joystick, strip);

void setup()
{

    // intialise les boutons
    for (int i = L1; i <= L4; i++)
    {
        pinMode(i, INPUT_PULLUP);
    }

    //Initialise le ruban de DEL
    strip.begin();
    strip.clear();
    strip.setBrightness(10);

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
