#include <Ultrasonic.h>
#include <command_motors.h>
#include <capteurs_sm.h>

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

command_motors Command(2, 3, 4, 5, 6, 7, 8, 9);
capteurs_sm Capteurs(A0, 30, 31, A1);

// Définition de la broche de l'interrupteur
const int interrupteurPin = 13;

// Variable pour stocker l'état de l'interrupteur
int etatInterrupteur = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400);
  Serial1.begin(9600);
    
  Wire.begin();

  // Configuration de la broche de l'interrupteur en entrée
  pinMode(interrupteurPin, INPUT);

  // Activation de la résistance de pull-up interne
  digitalWrite(interrupteurPin, HIGH);

  Command.initialisation_generale();

}

void loop() {

  // Lecture de l'état de l'interrupteur
  etatInterrupteur = digitalRead(interrupteurPin);

  // Vérification de l'état de l'interrupteur
  if (etatInterrupteur == LOW) {
    // L'interrupteur est ferme
    Command.cmd_Test(1700, 1700, 1700, 1700, 1700, 1700, 1700, 1700);
    Capteurs.get_and_send_data();
  } else {
    // L'interrupteur est ouvert : arret d'urgence
    Command.cmd_U();
  }
}
