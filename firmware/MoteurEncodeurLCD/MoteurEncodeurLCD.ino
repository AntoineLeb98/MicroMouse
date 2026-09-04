#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioclass/hd44780_I2Cexp.h>

#define AIN1 46
#define AIN2 44
#define PWMA 2
#define STBY 48

#define ENCODER_A 3
#define ENCODER_B 4

int pot_pin = A0;
int pot_value;
int drive_value;

volatile long encoderCount = 0;

Motor moteur = Motor(AIN1, AIN2, PWMA, 1, STBY);
hd44780_I2Cexp lcd;

void encoderISR() {
  if (digitalRead(ENCODER_B) == LOW) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  pinMode(pot_pin, INPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  Serial.begin(9600);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Encodeur:");
}

void loop() {
  pot_value = analogRead(pot_pin);
  drive_value = map(pot_value, 0, 1023, 0, 255);
  moteur.drive(drive_value);

  lcd.setCursor(0, 1);
  lcd.print("Count: ");
  lcd.print(encoderCount);
  lcd.print("      ");  // efface les anciens chiffres restants

  delay(100);  // limite la fréquence d'update du LCD
}