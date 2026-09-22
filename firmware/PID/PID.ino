// Librairies
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

// Définition des objets de classe
Motor moteur1 = Motor(AI1, AI2, PWMA, -1, STBY);
Motor moteur2 = Motor(BI1, BI2, PWMB, 1, STBY);

// Encodeurs
volatile long compte_encodeur1 = 0;
volatile long compte_encodeur2 = 0;
bool sens_rotation1 = true;
bool sens_rotation2 = false;
float n_clic = 103;

// Roues & véhicule
float dia_roue = 0.06; // diamètre de roue en m
float empattement = 0.19; // distance entre les roues en m
float Z = 0.2; // un coefficient de multiplication de la vitesse

// PID
// gains
const float kp_p = 800; // gain proportionnel en position
const float kd_p = 200; // gain dérivé en position
const float ki_p = 10; // gain intégral en position
const float kp_a = 2; // gain proportionnel angulaire
const float kd_a = 2; // gain dérivé angulaire
const float ki_a = 0.001; // gain intégral angulaire
// erreur
float ed0 = 0; // pour le calcul de l'erreur dérivée
float ei0 = 0; // pour le calcul de l'erreur intégrale
// consigne
float cons_p = 0.3; // consigne en position en m
float cons_a = -90; // consigne angulaire en deg
// tolerance
float tol_p = 0.005; 
float tol_a = 1;

// Orchestration du mouvement
enum MOVES {ATTENTE, GO, AVANCE, TOURNE, CELEBRE, TEST};
MOVES dance_moves = GO;
int count = 0;
float periode_ctrl = 50; // période de calcul (ms)
unsigned long temps_actuel = 0;
unsigned long dernier_temps = 0;

// Liste des consignes
struct Consignes {
  int orientation;
  int nbCases;
}

Consignes listeConsignes[2];
listeConsignes[0].orientation = 90;
listeConsignes[0].nbCases = 2;
listeConsignes[1].orientation = 90;
listeConsignes[1].nbCases = 2;

// Séquences d'interupt pour le compte des encodeurs
void encodeur1_ISR() {
  if (digitalRead(encodeur_1B) == sens_rotation1) {
    compte_encodeur1++;
  } else {
    compte_encodeur1--;
  }
}
void encodeur2_ISR() {
  if (digitalRead(encodeur_2B) == sens_rotation2) {
    compte_encodeur2++;
  } else {
    compte_encodeur2--;
  }
}

// Odométrie
float position() {
  float distance_mesure = (compte_encodeur1 + compte_encodeur2)*PI*dia_roue/(2*n_clic);
  return distance_mesure;
}
float orientation() {
  float orientation_mesure = (compte_encodeur1 - compte_encodeur2)*180*dia_roue/(empattement*n_clic);
  return orientation_mesure;
}

// Contrôleur PID
int PID(float erreur, float kp, float kd, float ki, float dt) {
  float cmd = erreur*kp - kd*erreur_derivee(erreur, dt) + ki*erreur_integrale(erreur, dt);
  cmd = Z*constrain(cmd, -255, 255);
  return (int) cmd;
}
float erreur_derivee(float erreur, float dt) {
  float ed = (erreur-ed0)/dt;
  ed0 = erreur;
  return ed;
}
float erreur_integrale(float erreur, float dt) {
  ei0 += erreur*dt;
  return ei0;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(encodeur_1A,  INPUT_PULLUP); // connexion aux encodeurs
  pinMode(encodeur_1B,  INPUT_PULLUP);
  pinMode(encodeur_2A,  INPUT_PULLUP);
  pinMode(encodeur_2B,  INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encodeur_1A), encodeur1_ISR, RISING); // liaison à l'interupt service routine
  attachInterrupt(digitalPinToInterrupt(encodeur_2A), encodeur2_ISR, RISING);

  Serial.begin(9600); // vous êtes censé savoir c'est quoi
}


void loop() {
  // put your main code here, to run repeatedly:
  temps_actuel = millis();
  float dt = temps_actuel-dernier_temps;

  if (dt > periode_ctrl) {
    dernier_temps = temps_actuel;
    switch (dance_moves) {
      case ATTENTE: {
        
        break;
      }
      case TEST: {
        Serial.print("Position : ");
        Serial.print(position());
        Serial.print(" Orientation : ");
        Serial.println(orientation()); 
        break;
      }
      case GO: {
        ed0 = cons_a;
        dance_moves = TOURNE;
        dernier_temps = millis();
        break;
      }
      case AVANCE: {
        float erreur = cons_p - position();
        if (abs(erreur) > tol_p) {
          int cmd = PID(erreur, kp_p, kd_p, ki_p, dt);
          moteur1.drive(cmd);
          moteur2.drive(cmd);
        } 
        else {
          moteur1.brake();
          moteur2.brake();
          delay(1000);
          compte_encodeur1 = 0;
          compte_encodeur2 = 0;
          ed0 = cons_a;
          ei0 = 0;
          dance_moves = CELEBRE;
        }
        Serial.print("Consigne : ");
        Serial.print(cons_p);
        Serial.print(" Erreur : ");
        Serial.println(erreur);  
        break;
      }
      case TOURNE: {
          float erreur = cons_a - orientation();
          if (abs(erreur) > tol_a) {
            int cmd = PID(erreur, kp_a, kd_a, ki_a, dt);
            moteur1.drive(cmd);
            moteur2.drive(-cmd);
          } 
          else {
            moteur1.brake();
            moteur2.brake();
            delay(1000);
            compte_encodeur1 = 0;
            compte_encodeur2 = 0;
            ed0 = cons_p;
            ei0 = 0;
            dernier_temps = millis();
            dance_moves = AVANCE;
          }
        Serial.print("Consigne : ");
        Serial.print(cons_a);
        Serial.print(" Erreur : ");
        Serial.println(erreur);  
        break;
      }
      case CELEBRE: {
        Serial.println("yippe");
        dance_moves = ATTENTE;
        break;
      }
    }
  }
}
