#include <Wire.h>
#include <LiquidCrystal_I2C.h>



/*INIT*/
//LCD
LiquidCrystal_I2C lcd(0x39,16,2);

void setup() {

  
  //LCD Init 
  lcd.begin(16, 2);
  lcd.init();                      
  lcd.init();
  lcd.noBacklight(); //using open collector, therefore polarity is reversed
  
  lcd.print("hello, world!");
  delay(1000);

  pinMode(PIN_PC1, OUTPUT);
}

void loop() {
  // set the cursor to column 0, line 1
  lcd.setCursor(0,0);
  lcd.print(analogRead(PIN_PA1));
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis() / 100);
  
  digitalWrite(PIN_PC1, HIGH);
  digitalWrite(PIN_PC1, LOW);
}
