// Librairies
#include <IRremote.hpp>
#include <SparkFun_TB6612.h>

// Définition des pins
#define AI1 10
#define AI2 13
#define STBY 9
#define BI1 8
#define BI2 7
#define PWMA 11
#define PWMB 6
#define encodeur_1A 2
#define encodeur_1B 4
#define encodeur_2A 3
#define encodeur_2B 5
#define IR_receiver 12

// Définition des variables
bool debug = false;

// Encodeurs
volatile long compteEncodeur1 = 0;
volatile long comptencodeur2 = 0;
const int NB_tic_encodeurs = 103; // tics par tour complet

// Roues
const float dia_roue = 70; // en mm
const float empattement = 180; // en mm

// Moteurs
bool sens_rotation_A = true;
bool sens_rotation_B = false;
const float vitesse_modif = 0.5; // modificateur de vitesse

// Labyrinthe
const float dim_cellules = 300; // en mm
float orientationOrigine = 0;

// Horloge
const unsigned long periodeControle = 50; // période en ms 1/F
unsigned long dernierTemps = 0;

// Paramètres d'asservissement
const float toleranceAngle = 5.0;   // degrés
const float toleranceDistance = 50.0; // mm
float consigneAngle = 0;
float consigneDistance = 0;
float erreur_integrale = 0;

// Gains
float Kp_pos = 10;
float Kd_pos = 0;
float Ki_pos = 0;
float Kp_ang = 10;
float Kd_ang = 0;
float Ki_ang = 0;

// Erreur
float ed0 = 0;
float ei0 = 0;

// Définition des objets de classe
Motor moteur1 = Motor(AI1, AI2, PWMA, 1, STBY);
Motor moteur2 = Motor(BI1, BI2, PWMB, -1, STBY);

// Orchestration du mouvement
enum EtatRobot {ATTENTE,ROTATION,TRANSLATION,TERMINE};
EtatRobot etatActuel = ATTENTE;
int indexCommandeActuelle = 0;

// Lecture de la consigne et liste d'attente de commandes
struct Commande {
  int direction;   // 0=UP, 1=RIGHT, 2=DOWN, 3=LEFT
  int nbCases;
};
Commande fileCommandes[10];
enum Direction {NORD, SUD, EST, OUEST};

int nbCommandesRecues = 0;
int directionEnAttente = -1; // -1 = pas encore de direction reçue

// Séquences d'interupt pour le compte des encodeurs
void encodeur1_ISR() {
  if (digitalRead(encodeur_1B) == sens_rotation_A) {
    compteEncodeur1++;
  } else {
    compteEncodeur1--;
  }
}
void encodeur2_ISR() {
  if (digitalRead(encodeur_2B) == sens_rotation_B) {
    compteEncodeur2++;
  } else {
    compteEncodeur2--;
  }
}

// Réception des commandes manette
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
  if (code == 0x43) return -90; // Est
  if (code == 0x15) return 180; // Sud
  if (code == 0x44) return 90; // Ouest
  return -1; // pas une touche direction
}
int codeVersDistance(int code) {
  if (code == 0x16) return 1*dim_cellules;
  if (code == 0x19) return 2*dim_cellules;
  if (code == 0xD)  return 3*dim_cellules;
  if (code == 0xC)  return 4*dim_cellules;
  if (code == 0x18) return 5*dim_cellules;
  if (code == 0x5E) return 6*dim_cellules;
  if (code == 0x8)  return 7*dim_cellules;
  if (code == 0x1C) return 8*dim_cellules;
  if (code == 0x5A) return 9*dim_cellules;
  return -1; // pas un chiffre
}
void receptionCmdIR() {
  if (IrReceiver.decode()) {
    int code = IrReceiver.decodedIRData.command;

    if (code == 0x40) {
      // OK -> termine la séquence, lance le déplacement (pour plus tard, étape 4)
      // rien à faire ici pour l'instant
      // afficherFileCommandes();
      etatActuel = ROTATION;
      Serial.println("Go time");
    }
    else if (code == 0x52) {
      // recommencer la séquence -> vide la file
      nbCommandesRecues = 0;
      directionEnAttente = -1;
    }
    else if (code == 0x42 || code == 0x4A) {
      // arrêt d'urgence -> pour plus tard (étape 2, moteurs)
      etatActuel = TERMINE;
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

// Odométrie
float distRoue1() {
  return PI * dia_roue * compteEncodeur1 / NB_tic_encodeurs;
}
float distRoue2() {
  return PI * dia_roue * compteEncodeur2 / NB_tic_encodeurs;
}
float distance() {
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

  if (debug) {
    Serial.print(" cmd1: ");
    Serial.print(cmd1);
    Serial.print(" cmd2: ");
    Serial.print(cmd2);
  }
}
// Asservissement
void asservirAngle(float dt, float consigne, float etat) {
  float erreur = consigne - etat;
  float cmd = Kp_ang*erreur - Kd_ang*erreur/dt + Ki_ang*erreur*dt;
  appliquerMoteurs(cmd,-cmd);
}
void asservirPosition(float dt, float consigne, float etat) {
  float erreur = consigne - etat;
  float cmd = Kp_pos*erreur - Kd_pos*erreur/dt + Ki_pos*dt;
  appliquerMoteurs(cmd,cmd);
}
bool consigneAtteinte(float consigne, float mesure, float tolerance) {
  return abs(consigne - mesure) < tolerance;
}
float controleurD (float erreur, float dt) {
  float cmd_derivee = (erreur-ed0)/dt;
  ed0 = erreur;
  return cmd_derivee;
}
float controleurI (float erreur, float dt) {
  float cmd_integrale = 0;
}


// On se prépare ... initialisation  des composants 
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
    float dt = (tempsActuel - dernierTemps);
    dernierTemps = tempsActuel;    

    // 4 états du robot
    switch (etatActuel) {
      case ATTENTE: {
        Serial.println(" Waiting for the worms");
        break;
      }
      case ROTATION: {
        consigneAngle = fileCommandes[indexCommandeActuelle].direction - orientationOrigine;
        while (consigneAngle > 180) consigneAngle -= 360;
        while (consigneAngle < -180) consigneAngle += 360;
        float theta = orientation();
        asservirAngle(dt, consigneAngle, theta);         

        if (consigneAtteinte(consigneAngle, theta, toleranceAngle)) {
          moteur1.brake();
          moteur2.brake();
          orientationOrigine = orientation();
          resetOdometrie();
          etatActuel = TRANSLATION;
        }
        if (debug) {
        Serial.print("État : ");
        Serial.print(etatActuel);
        Serial.print(" Compte : ");
        Serial.print(compteEncodeur1);
        Serial.print(" Duchesse : ");
        Serial.print(compteEncodeur2);
        Serial.print(" Orientation : ");
        Serial.print(theta);
        }
        break;
      }
      case TRANSLATION: {
        consigneDistance = fileCommandes[indexCommandeActuelle].nbCases;
        float x = distance();
        asservirPosition(dt, consigneDistance, x);        

        if (consigneAtteinte(consigneDistance, x, toleranceDistance)) {
          moteur1.brake();
          moteur2.brake();
          indexCommandeActuelle++;
          if (indexCommandeActuelle < nbCommandesRecues) {
            etatActuel = ROTATION;
            consigneAngle = fileCommandes[indexCommandeActuelle].direction;
          } 
          else {
            etatActuel = TERMINE;
          }
          resetOdometrie();
        };
        if (debug) {
        Serial.print("État : ");
        Serial.print(etatActuel);
        Serial.print(" Compte : ");
        Serial.print(compteEncodeur1);
        Serial.print(" Duchesse : ");
        Serial.print(compteEncodeur2);
        Serial.print(" Position : ");
        Serial.println(x);
        }
        break;
      }
      case TERMINE: {
        // clear les variables, retourne à ATTENTE
        moteur1.brake();
        moteur2.brake();
        resetOdometrie();
        Serial.println("Victory chant");
        etatActuel = ATTENTE;
        break;
      }
    }
  }
}