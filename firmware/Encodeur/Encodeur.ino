#define encodeur_1A 2
#define encodeur_1B 4
#define encodeur_2A 3
#define encodeur_2B 5

// Encodeurs
volatile long compte_encodeur1 = 0;
volatile long compte_encodeur2 = 0;
bool sens_rotation1 = true;
bool sens_rotation2 = false;
float n_clic = 103;

// Roues & véhicule
float dia_roue = 60; // diamètre de roue en mm
float empattement = 190; // distance entre les roues en mm


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
  Serial.print("Position : ");
  Serial.print(position());
  Serial.print(" Orientation : ");
  Serial.print(orientation());
  Serial.print(" Compte1 : ");
  Serial.print(compte_encodeur1);
  Serial.print(" Compte2 : ");
  Serial.println(compte_encodeur2);
}