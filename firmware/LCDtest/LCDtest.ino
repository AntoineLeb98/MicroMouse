#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioclass/hd44780_I2Cexp.h>

hd44780_I2Cexp lcd;

const int LCD_COLS = 16;
const int LCD_ROWS = 2;

void setup() {
  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  if (status) {
    // erreur d'initialisation - reste en boucle avec le backlight qui clignote
    hd44780::fatalError(status);
  }
  lcd.setCursor(0, 0);
  lcd.print("Micromouse");
  lcd.setCursor(0, 1);
  lcd.print("Encoder: 0");
}

void loop() {
}