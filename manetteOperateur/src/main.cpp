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

unsigned long previousMillis;
unsigned long resetTime;

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

    previousMillis = millis();
    resetTime = 5 * 60 * 1000L; //5 minutes en milli secondes //Le L veut dire que c'est un constante "long"
}

void loop()
{
    //Loop sur tous les boutons
    hauteur.lireBouton();
    position.lireBouton();

    //Si le bouton vient de changer, on active la DEL et on envoit le bouton à la driver station
    if (hauteur.isChangementBouton()){
        hauteur.actionsBouton(hauteur.getDernierBouton());
    }

    //Idem pour les positions
     if (position.isChangementBouton()){
        position.actionsBouton(position.getDernierBouton());
    }

    if (hauteur.isChangementBouton() || position.isChangementBouton()){
        previousMillis = millis();//Si on appuie sur un bouton, ça empêche un reset
    }

    //Si la manette est inactive durant 5 minutes, on reset tout
    if(millis() - previousMillis >= resetTime){
        hauteur.reset();
        position.reset();
    }
}
