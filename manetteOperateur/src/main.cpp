#include <OperateurIO.h>

#define NB_BOUTON 16



Adafruit_NeoPixel strip(NB_BOUTON, LED_PIN, NEO_GRB + NEO_KHZ800);

//Sert à communiquer avec la Driver Station
Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   NB_BOUTON, 0, false, false, false,
                   false, false, false,
                   false, false,
                   false, false, false);


CD74HC4067 mux(S0,S1,S2,S3);

//firstPin, lastPin, le joystick et les del
//Il faut pointer vers le joystick car on veut que chaque objet OperateurIO parle au même objet joystick
//Même chose pour la LED strip
OperateurIO hauteur(L1, L4, &joystick, &strip, &mux);
OperateurIO position(A, L, &joystick, &strip, &mux);

void setup()
{

    //Initialise le ruban de DEL
    strip.begin();
    strip.setBrightness(50);

    //Reset la strip au début du programme
    strip.clear();
    strip.show();
    

    //Initiatilise le joystick
    joystick.begin();

    Serial.begin(9600);

    pinMode(SIG,INPUT_PULLUP);
}

void loop()
{
    //Input : Itère sur tous les boutons, vérifie le dernier appuyé
    //Output : Envoie le dernier bouton appuyé à la driver station et aux LEDs
    hauteur.loopBoutonEtLED();
    position.loopBoutonEtLED();
}
