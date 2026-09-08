#define Encodeur_A 3
#define Encodeur_B 4

volatile long compteEncodeur = 0;

void encodeurISR() {
  if (digitalRead(Encodeur_B) == LOW){
    compteEncodeur++;
  } else {
    compteEncodeur--;
  }
}

void setup() {
  pinMode(Encodeur_A, INPUT_PULLUP);
  pinMode(Encodeur_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Encodeur_A),encodeurISR,RISING);

  Serial.begin(9600);
}

void loop() {
  Serial.print("Nombre de clics : ");
  Serial.println(compteEncodeur);
  delay(100);
}