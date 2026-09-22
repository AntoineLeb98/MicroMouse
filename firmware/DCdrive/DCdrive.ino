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

// Définition des objets de classe
Motor moteur1 = Motor(AI1, AI2, PWMA, -1, STBY);
Motor moteur2 = Motor(BI1, BI2, PWMB, 1, STBY);

void setup() {

}

void loop() {
  moteur1.drive(255);
  moteur2.drive(255);
}