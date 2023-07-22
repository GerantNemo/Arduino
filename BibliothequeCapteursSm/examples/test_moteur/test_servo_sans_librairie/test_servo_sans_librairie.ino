
#include <Servo.h>
/*
Les pins entres de la classe Command commandent dans l'ordre:
-Moteur principal avant droit
-Moteur principal arriere droit
-Moteur principal arriere gauche
-Moteur principal avant gauche
-Moteur de profondeur avant droit
-Moteur de profondeur arriere droit
-Moteur de profondeur arriere gauche
-Moteur de profondeur avant gauche
*/

/*
Les pins entres de la classe capteurs connectent:
-le capteur d'humidite (pin analogique)
-l'emission du capteur ultrason (pin digital)
-la reception du capteur ultrason (pin digital)
-le capteur de tension (montage de resistances) (pin analogique)
*/
//La pression communique en I2C (broches A4(SDA) et A5(SCL) pour Uno, broches 20(SDA) et 21(SCL))
//Cables pression : GND : noir ; VCC (tension, 3.3 ou 5v) : rouge ; SDA : blanc ; SCL : vert
//L'IMU communique sur le port Serial 1 de l'Arduino Mega

Servo servo;

const int pinMot1 = 2;
// Définition de la broche de l'interrupteur
const int interrupteurPin = 13;

// Variable pour stocker l'état de l'interrupteur
int etatInterrupteur = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400);

  // Configuration de la broche de l'interrupteur en entrée
  pinMode(interrupteurPin, INPUT);

  // Activation de la résistance de pull-up interne
  digitalWrite(interrupteurPin, HIGH);

  servo.attach(pinMot1);
  servo.writeMicroseconds(1500);//Arret du moteur
  delay(7000);

}

void loop() {

  // Lecture de l'état de l'interrupteur
  etatInterrupteur = digitalRead(interrupteurPin);

  // Vérification de l'état de l'interrupteur
  if (etatInterrupteur == LOW) {
    // L'interrupteur est ferme
    servo.writeMicroseconds(1700);//Arret du moteur
    delay(1000);
  } else {
    // L'interrupteur est ouvert : arret d'urgence
    servo.writeMicroseconds(1500);
    delay(1000);
  }
}
