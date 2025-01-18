#include <Joystick.h>
#define L1 2
#define L2 3
#define L3 4
#define L4 5

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   6, 0, false, false, false,
                   false, false, false,
                   false, false,
                   false, false, false);

int dernierBouton = 0;

int dernierBoutonPasse = 0;

bool change = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(L1, INPUT_PULLUP);
  pinMode(L2, INPUT_PULLUP);
  pinMode(L3, INPUT_PULLUP);
  pinMode(L4, INPUT_PULLUP);
  Joystick.begin();

  Serial.begin(9600);
}

void loop() {
  for (int i = L1; i <= L4; i++) {
    if (!digitalRead(i)) {
      dernierBouton = i;
    }
  }
  if (dernierBouton != dernierBoutonPasse) {
    change = true;
  }
  if (change) {
    Serial.println(dernierBouton);
    for (int i = L1; i <= L4; i++){
      if (i == dernierBouton) {
        Joystick.setButton(i, 1);
      } else {
        Joystick.setButton(i, 0);
      }
    }
    change = false;
  }
  dernierBoutonPasse = dernierBouton;
  delay(100);
}
