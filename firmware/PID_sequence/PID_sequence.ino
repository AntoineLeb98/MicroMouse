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

// Debug mode
bool debug = true;

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
float dia_roue = 60; // diamètre de roue en mm
float empattement = 160; // distance entre les roues en mm
float Z = 0.5; // un coefficient de multiplication de la vitesse
float x0 = 0; // pour le calcul de la vitesse
float a0 = 0; // pour le calcul de la vitesse angulaire

// Labyrinthe
float d_case = 300; // distance d'une cellule

// PID
// gains directionnels
const float kp_p = 4; // gain proportionnel en position
const float kd_p = 6; // gain dérivé en position
const float ki_p = 0.01; // gain intégral en position
// gains angulaires
const float kp_a = 1.5; // gain proportionnel angulaire
const float kd_a = 1; // gain dérivé angulaire
const float ki_a = 0.003; // gain intégral angulaire
// erreur
float ed0 = 0; // pour le calcul de l'erreur dérivée
float ei0 = 0; // pour le calcul de l'erreur intégrale
// consigne
float cons_p = 0; // consigne en position en m
float cons_a = 0; // consigne angulaire en deg
// tolerance
float tol_p = 5; 
float tol_a = 1;
float tol_v = 2;
float tol_w = 2;

// Orchestration du mouvement
enum MOVES {ATTENTE, GO, AVANCE, TOURNE, CELEBRE, TEST};
MOVES dance_moves = GO;
int compte = 0;
float periode_ctrl = 50; // période de calcul (ms)
unsigned long temps_actuel = 0;
unsigned long dernier_temps = 0;

// Liste des consignes
struct Mouvements {
  int orientation;
  int nbCases;
};
const int nombre_mouvements = 9; 
Mouvements liste_consignes[nombre_mouvements] {
  {0, 2},
  {-90, 1},
  {-90, 1},
  {90, 1},
  {90, 4},
  {90, 4},
  {-90, 4},
  {-90, 2},
  {90, 2}
};

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
  float orientation_mesure = (compte_encodeur1 - compte_encodeur2)*180*dia_roue/(empattement*0.5*n_clic);
  return orientation_mesure;
}
float vitesse(float x, float dt) {
  float v = (x-x0)/dt;
  x0 = x;
  return v;
}
float vitesse_angulaire(float a, float dt) {
  float w = (a-a0)/dt;
  a0 = a;
  return w;
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
  pinMode(encodeur_1A,  INPUT_PULLUP);
  pinMode(encodeur_1B,  INPUT_PULLUP);
  pinMode(encodeur_2A,  INPUT_PULLUP);
  pinMode(encodeur_2B,  INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encodeur_1A), encodeur1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encodeur_2A), encodeur2_ISR, RISING);

  Serial.begin(9600);
}

void loop() {
  temps_actuel = millis();
  float dt = temps_actuel-dernier_temps;

  if (dt > periode_ctrl) {
    dernier_temps = temps_actuel;
    switch (dance_moves) {
      case ATTENTE: {
          delay(15000);
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
        cons_a = liste_consignes[compte].orientation;
        ed0 = cons_a;
        dance_moves = TOURNE;
        dernier_temps = millis();
        break;
      }
      case AVANCE: {
        float x = position();
        float erreur = cons_p - x;
        float v = vitesse(x, dt);
        if (abs(erreur) > tol_p || abs(v) > tol_v) {
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
          compte += 1;
          if (compte >= nombre_mouvements) {
            dance_moves = CELEBRE;
          } 
          else {
            cons_a = liste_consignes[compte].orientation;
            ed0 = cons_a;
            ei0 = 0;
            dernier_temps = millis();
            dance_moves = TOURNE;
          }
        }
        break;
      }
      case TOURNE: {
          float theta = orientation();
          float erreur = cons_a - theta;
          float w = vitesse_angulaire(theta, dt);
          if (abs(erreur) > tol_a || abs(w) > tol_w) {
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
            cons_p = liste_consignes[compte].nbCases * d_case;
            ed0 = cons_p;
            ei0 = 0;
            dernier_temps = millis();
            dance_moves = AVANCE;
          }
        break;
      }
      case CELEBRE: {
        Serial.println("yippe");
        dance_moves = ATTENTE;
        break;
      }
    }
  }
  if (debug) {
    Serial.print("Position : ");
    Serial.print(position());
    Serial.print(" Orientation : ");
    Serial.println(orientation());
  }
}