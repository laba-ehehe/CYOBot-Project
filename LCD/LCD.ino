#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 4 line display

void setup()
{
  Wire.begin(21, 22); // SDA pin 21, SCL pin 22 (default for ESP32, adjust if needed)

  lcd.init(); // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(":DDD");
  // lcd.setCursor(0,1);
  // lcd.print(":DDD");
  // lcd.setCursor(0,2);
  // lcd.print(":DDD");
  // lcd.setCursor(0,3);
  // lcd.print(":DDD");
  // lcd.setCursor(2,1);
}

void loop()
{
}
