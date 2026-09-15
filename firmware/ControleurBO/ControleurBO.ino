// Librairies
#include <IRremote.hpp>
#include <SparkFun_TB6612.h>

// Définition des pins
#define AI1 7
#define AI2 8
#define STBY 13
#define BI1 10
#define BI2 11
#define PWMA 6
#define PWMB 9
#define encodeur_1A 2
#define encodeur_1B 4
#define encodeur_2A 3
#define encodeur_2B 5
#define IR_receiver 12

// Définition des variables

// Encodeurs
volatile long compteEncodeur1 = 0;
volatile long compteEncodeur2 = 0;

const int NB_tic_encodeurs = 422; // tics par tour complet

// Roues
const float dia_roue = 40; // en mm
const float empattement = 160; // en mm

// Moteurs
bool sens_rotation_A = true;
bool sens_rotation_B = false;
const float vitesse_modif = 0.5; // modificateur de vitesse

// Labyrinthe
const float dim_cellules = 300; // en mm

// Paramètres d'asservissement
const unsigned long periodeControle = 20; // période en ms 1/F
unsigned long dernierTemps = 0;
const float toleranceAngle = 2.0;   // degrés
const float toleranceDistance = 5.0; // mm

// Définition des objets de classe
Motor moteur1 = Motor(AI1, AI2, PWMA, 1, STBY);
Motor moteur2 = Motor(BI1, BI2, PWMB, 1, STBY);

// Séquences d'interupt pour le compte des encodeurs
void encodeur1_ISR() {
  if (digitalRead(encodeur_1A) == sens_rotation_A) {
    compteEncodeur1++;
  } else {
    compteEncodeur1--;
  }
}

void encodeur2_ISR() {
  if (digitalRead(encodeur_2A) == sens_rotation_B) {
    compteEncodeur2++;
  } else {
    compteEncodeur2--;
  }
}

// Lecture de la consigne et liste d'attente de commandes
struct Commande {
  int direction;   // 0=UP, 1=RIGHT, 2=DOWN, 3=LEFT
  int nbCases;
};

Commande fileCommandes[10];
enum Direction {NORD, SUD, EST, OUEST};

int nbCommandesRecues = 0;
int directionEnAttente = -1; // -1 = pas encore de direction reçue

void afficherFileCommandes() {
  Serial.println("--- File de commandes ---");
  for (int i = 0; i < nbCommandesRecues; i++) {
    Serial.print(i);
    Serial.print(" : direction=");
    Serial.print(fileCommandes[i].direction);
    Serial.print(" nbCases=");
    Serial.println(fileCommandes[i].nbCases);
  }
  Serial.println("-------------------------");
}
int codeVersOrientation(int code) {
  if (code == 0x46) return 0; // Nord
  if (code == 0x43) return 1; // Est
  if (code == 0x15) return 2; // Sud
  if (code == 0x44) return 3; // Ouest
  return -1; // pas une touche direction
}
int codeVersDistance(int code) {
  if (code == 0x16) return 1;
  if (code == 0x19) return 2;
  if (code == 0xD)  return 3;
  if (code == 0xC)  return 4;
  if (code == 0x18) return 5;
  if (code == 0x5E) return 6;
  if (code == 0x8)  return 7;
  if (code == 0x1C) return 8;
  if (code == 0x5A) return 9;
  return -1; // pas un chiffre
}
void receptionCmdIR() {
  if (IrReceiver.decode()) {
    int code = IrReceiver.decodedIRData.command;

    if (code == 0x40) {
      // OK -> termine la séquence, lance le déplacement (pour plus tard, étape 4)
      // rien à faire ici pour l'instant
      afficherFileCommandes();
    }
    else if (code == 0x52) {
      // recommencer la séquence -> vide la file
      nbCommandesRecues = 0;
      directionEnAttente = -1;
    }
    else if (code == 0x42 || code == 0x4A) {
      // arrêt d'urgence -> pour plus tard (étape 2, moteurs)
      moteur1.brake();
      moteur2.brake();
    }
    else if (directionEnAttente == -1) {
      // On attend une direction
      int dir = codeVersOrientation(code);
      if (dir != -1) {
        directionEnAttente = dir;
      }
    }
    else {
      // On attend un chiffre
      int chiffre = codeVersDistance(code);
      if (chiffre != -1 && nbCommandesRecues < 10) {
        fileCommandes[nbCommandesRecues].direction = directionEnAttente;
        fileCommandes[nbCommandesRecues].nbCases = chiffre;
        nbCommandesRecues++;
        directionEnAttente = -1;
      }
    }

    IrReceiver.resume();
  }
}

// Orchestration du mouvement

enum EtatRobot { ATTENTE, ROTATION, TRANSLATION, TERMINE };
EtatRobot etatActuel = ATTENTE;

int indexCommandeActuelle = 0;
float consigneAngle = 0;
float consigneDistance = 0;

// Odométrie
float distRoue1() {
  return PI * dia_roue * compteEncodeur1 / NB_tic_encodeurs;
}
float distRoue2() {
  return PI * dia_roue * compteEncodeur2 / NB_tic_encodeurs;
}
float distParcourue() {
  return 0.5 * (distRoue1() + distRoue2());
}
float orientation() {
  return (distRoue1() - distRoue2()) / empattement * (180.0 / PI); // en degrés
}
void resetOdometrie() {
  noInterrupts();
  compteEncodeur1 = 0;
  compteEncodeur2 = 0;
  interrupts();
}

// Commande moteurs
void appliquerMoteurs(int cmd1, int cmd2) {
  cmd1 = constrain(cmd1, -255, 255);
  cmd2 = constrain(cmd2, -255, 255);
  moteur1.drive((int) cmd1*vitesse_modif);
  moteur2.drive((int) cmd2*vitesse_modif);
}

// Asservissement
void asservirAngle(float dt, float consigne) {
  float erreur = consigne - orientation();

  float Kp = 1;
  float Kd = 0;
  float Ki = 0;
  
  float cmd = Kp*erreur - Kd*erreur/dt + Ki*erreur*dt;
  appliquerMoteurs(cmd,-cmd);
}

void asservirPosition(float dt, float consigne) {
  float Kp = 1;
  float Kd = 0;
  float position_actuelle = distParcourue();
  float erreur = consigne - position_actuelle;
  float cmd = Kp*erreur - Kd*erreur/dt;
  appliquerMoteurs(cmd,cmd);
}

bool consigneAtteinte(float consigne, float mesure, float tolerance) {
  return abs(consigne - mesure) < tolerance;
}

// On se prépare ... initialisation  des composants et de la logique du contrôleur
void setup() {
  pinMode(encodeur_1A,  INPUT_PULLUP); // connexion aux encodeurs
  pinMode(encodeur_1B,  INPUT_PULLUP);
  pinMode(encodeur_2A,  INPUT_PULLUP);
  pinMode(encodeur_2B,  INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encodeur_1A), encodeur1_ISR, RISING); // liaison à l'interupt service routine
  attachInterrupt(digitalPinToInterrupt(encodeur_2A), encodeur2_ISR, RISING);

  Serial.begin(9600); // vous êtes censé savoir c'est quoi

  IrReceiver.begin(IR_receiver); // connexion à la manette
}

// Exécution!!
void loop() {
  receptionCmdIR(); //est-ce qu'on a reçu une nouvelle commande?

  unsigned long tempsActuel = millis(); //part le timer

  if (tempsActuel - dernierTemps >= periodeControle) {
    // est-ce qu'on doit faire un reset de la commande selon notre fréquence?
    float dt = (tempsActuel - dernierTemps);
    dernierTemps = tempsActuel;

    //où est-ce qu'on est dans notre séquence
    switch (etatActuel) {
      case ATTENTE:
      // Waiting for the worms
      break;

      case ROTATION:
      asservirAngle(dt, consigneAngle);
      break;

      case TRANSLATION:
      asservirPosition(dt, consigneDistance);
      break;

      case TERMINE:
      // clear les variables, retourne à ATTENTE
      break;
    }
  }
}