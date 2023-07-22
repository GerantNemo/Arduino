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

// Définition de la broche de l'interrupteur
const int interrupteurPin = 13;

// Variable pour stocker l'état de l'interrupteur
int etatInterrupteur = 0;

//Variable pour lancer une unique fois les moteurs (une fois l'interrupteur ouvert, les moteurs ne se rallumeront plus meme en refermant celui-ci)
int Secu = 0;

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

  Command.send_datas();

  // Vérification de l'état de l'interrupteur
  if (etatInterrupteur == LOW && Secu == 0) {
    // L'interrupteur est ferme
    Command.get_and_exec_command();
  //Command.cmd_D(); //Affichage des commandes reçues
  } else {
    // L'interrupteur est ouvert : arret d'urgence
    Command.cmd_U();
    Secu == 1;
  }
}
