// Ce programme contrôle un lidar et deux moteurs pas à pas.
// Il effectue des mesures avec le lidar à différentes positions angulaires et enregistre les données dans un fichier CSV sur une carte SD.

// Configuration du lidar
// Choix entre le mode I2C et UART en fonction du branchement du capteur, on prend I2C.
#include "DFRobot_LIDAR07.h"

//If using IIC mode, please enable macro USE_IIC
#define USE_IIC
#ifdef USE_IIC
DFRobot_LIDAR07_IIC LIDAR07;
#endif

// Configuration des moteurs
// Initialisation des paramètres des moteurs pas à pas et des pins de contrôle
#include <Arduino.h>  // Bibliothèque

double MOTOR_STEPS_Theta = 400;  // 0.9° par pas sans modification
double MOTOR_STEPS_Phi = 200;    // 1.8° par pas sans modification
#define RPM 120                  // Vitesse des moteurs

#include "A4988.h"  // Bibliothèque des drivers moteurs
#define DIR 5       // Choix du sens de rotation
#define STEP 6      // Effectue les rotations
#define MS1 9       // Configuration des 3 pins microsteps pour un meilleur ratio de précision
#define MS2 8
#define MS3 7
#include "BasicStepperDriver.h"  // generic


//BasicStepperDriver stepperTheta(MOTOR_STEPS_Theta, 5, 6);
A4988 stepperTheta(MOTOR_STEPS_teta, DIR, STEP, MS1, MS2, MS3);  // Moteur de 400 pas configuré pour le driver
A4988 stepperPhi(MOTOR_STEPS_Phi, 0, 1, 4, 3, 2);                // Moteur de 200 pas configuré pour le driver
int a = 1;                                                       // Variable de changements de sens pour le moteur petit

//Definition des microsteps compris celon 1,2,4,8,16,32
double Microsteps_Theta = 1;  //multiplicateur du nombres de pas pour notre moteur , si on met plus de 1 alors changer le basic en stepper
double Microsteps_Phi = 2;    //idem

// Fin de la configuration des moteurs

// Configuration de la carte SD
#include <SPI.h>  // Bibliothèque
#include <SD.h>   // Bibliothèque
// Fin de la configuration de la carte SD

File dataFile;  // Démarre la carte SD

//on declare des variable pour stocker dans un tableau pour aller plus vite en evitant d'ouvrir et fermer tout le temp le fichier csv
struct DataPoint {  // declare un constructeur , une sorte de classe sans methode qui a 3 attributs
  double distance;
  double Angle_Theta;
  double Angle_Phi;
};

const int MAX_DATA_POINTS = 400;  // taille du tableau , du nombre de valeur qu'on a en 1 tour

DataPoint dataList[MAX_DATA_POINTS];  // Tableau pour stocker les données ou chaque emplacement de donne est du type de la classe au dessus
int dataCount = 0;                    // Variable pour compter le nombre de points de données stockés

// Déclaration des variables pour écrire dans notre fichier CSV
double Angle_Theta = 0;
double Angle_Phi = 0;
double Var_Theta = 360 / (MOTOR_STEPS_Theta * Microsteps_Theta);
double Var_Phi = 360 / (MOTOR_STEPS_Phi * Microsteps_Phi);

double nb_pas_Phi = MOTOR_STEPS_Phi * Microsteps_Phi / 2;
double nb_pas_Theta = MOTOR_STEPS_Phi * Microsteps_Phi;
//config matrice led
#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;

const int ROWS = 8;
const int COLS = 12;
const int SIZE = ROWS * COLS;

// Définition des matrices 5, 4, 3, 2, 1, 0
uint8_t frame_5[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 }
};
uint8_t frame_4[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 }
};
uint8_t frame_3[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 }
};
uint8_t frame_2[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 }
};
uint8_t frame_1[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 }
};
uint8_t frame_0[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 }
};
uint8_t frame[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};
const uint32_t happy[] = {  // Matrice visage souriant
  0x19819,
  0x80000001,
  0x81f8000
};
const uint32_t heart[] = {  // Matrice coeur
  0x3184a444,
  0x44042081,
  0x100a0040
};

//fin config matrice led

// Initialisation des composants au démarrage du programme
// Allumage des moteurs pas à pas et configuration du lidar
void setup() {
  matrix.begin();  //demarage matrice led


  // Allumage et configuration des microsteps
  stepperTheta.begin(RPM);  // Vitesse moteur 1
  stepperPhi.begin(RPM);    // Vitesse moteur 2
  stepperTheta.enable();    // Allume moteur 1
  stepperPhi.enable();      // Allume moteur 2
  //stepperTheta.setMicrostep(Microsteps_Theta);   // Définit le ratio pour le moteur 1, on a 400 pas et si on modifie, c'est du binaire jusqu'à 1/16
  stepperPhi.setMicrostep(Microsteps_Phi);  // Définit le ratio pour le moteur 2, on a 800 pas et si on modifie, c'est du binaire jusqu'à 1/16

  // Allumage du lidar
  uint32_t version;
  Serial.begin(115200);
#ifdef USE_IIC
  while (!LIDAR07.begin()) {
    Serial.println("The sensor returned data validation error");
    delay(100);
  }
#endif

  version = LIDAR07.getVersion();
  Serial.print("VERSION: ");
  Serial.print((version >> 24) & 0xFF, HEX);
  Serial.print(".");
  Serial.print((version >> 16) & 0xFF, HEX);
  Serial.print(".");
  Serial.print((version >> 8) & 0xFF, HEX);
  Serial.print(".");
  Serial.println((version)&0xFF, HEX);

  //After enabling the filter, it can be stopped by calling LIDAR07.stopFilter() mais ca marche pas car la fonction est privé !!!!!
  while (!LIDAR07.startFilter())
    ;
  while (!LIDAR07.setMeasureMode(LIDAR07.eLidar07Continuous))
    ;
  while (!LIDAR07.setConMeasureFreq(10))
    ;

  //Open measurement (in single measurement mode, it will automatically close after sampling).To stop collection, use stopMeasure() toujours pas fonctionel
  LIDAR07.startMeasure();

  // Initialisation de la carte SD
  if (SD.begin(10)) {
    Serial.println("La carte SD est initialisée.");
  } else {
    Serial.println("Erreur lors de l'initialisation de la carte SD.");
  }
  // Supprimer le fichier data.csv s'il existe déjà
  if (SD.exists("data.csv")) {
    SD.remove("data.csv");
    Serial.println("Fichier data.csv existant supprimé.");
  }
  // Ouverture d'un fichier CSV pour écrire dedans
  dataFile = SD.open("data.csv", FILE_WRITE);

  if (dataFile) {
    dataFile.println("Distance,AnglePetit,AngleGros");
    dataFile.close();
  } else {
    Serial.println("Erreur lors de l'ouverture de data.csv en écriture.");
  }
}

// Boucle principale du programme
// Déplacement des moteurs pas à pas pour effectuer les mesures avec le lidar à différentes positions angulaires

void loop() {
  matrice();                              // decompte pour le démarage , c'est l'indicateur pour l'utilisateur
  for (int j = 0; j < nb_pas_Phi; j++) {  // Boucle pour le gros moteur qui tourne lentement
    stepperPhi.move(1);
    delay(1);
    Angle_Theta = 0;

    for (int i = 0; i < nb_pas_Theta; i++) {  // Boucle petit moteur
      //Get the collected data
      if (LIDAR07.getValue()) {
        insertData(LIDAR07.getDistanceMM(), Angle_Theta, Angle_Phi);
      }
      stepperTheta.move(a);          // Bouge
      delay(10);                     //il permet de rendre plus stable le mouvement on le met comme la frequence de mesure du lidar
      Angle_Theta += Var_Theta * a;  //- Change de sens après un tour complet
    }
    Angle_Phi += Var_Phi;  // Bouge
    a = -a;                // Change le sens de rotation du petit moteur
    writeDataToFile();
    dataCount = 0;
  }



  while (true) {  // boucle infinie qui indique que le code a fini de tourner
    matrix.loadFrame(happy);
    delay(500);
    matrix.loadFrame(heart);
    delay(500);
  }
}


// Fonction pour ajouter une nouvelle entrée à la liste
void insertData(double distance, double Angle_Theta, double Angle_Phi) {
  dataList[dataCount].distance = distance;
  dataList[dataCount].Angle_Theta = Angle_Theta;
  dataList[dataCount].Angle_Phi = Angle_Phi;
  dataCount++;  // Incrémente le nombre de données stockées
}

// Fonction pour écrire toutes les données stockées sur la carte SD
void writeDataToFile() {
  dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {

    for (int i = 0; i < dataCount; i++) {
      dataFile.print(dataList[i].distance);
      dataFile.print(",");
      dataFile.print(dataList[i].Angle_Theta);
      dataFile.print(",");
      dataFile.println(dataList[i].Angle_Phi);
    }

    dataFile.close();
  }
}


void matrice() {  // decomtpe du 5,4,3,2,1
  matrix.renderBitmap(frame_5, 8, 12);
  delay(1000);
  matrix.renderBitmap(frame_4, 8, 12);
  delay(1000);
  matrix.renderBitmap(frame_3, 8, 12);
  delay(1000);
  matrix.renderBitmap(frame_2, 8, 12);
  delay(1000);
  matrix.renderBitmap(frame_1, 8, 12);
  delay(1000);
  matrix.renderBitmap(frame_0, 8, 12);
  delay(1000);
  matrix.renderBitmap(frame, 8, 12);
}
