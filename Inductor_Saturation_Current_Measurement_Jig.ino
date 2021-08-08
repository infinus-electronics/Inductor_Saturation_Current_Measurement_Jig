//defines
#define FET_GATE PIN_PB5


#include <Wire.h>
#include <LiquidCrystal_I2C.h>



/*INIT*/
//LCD
LiquidCrystal_I2C lcd(0x39,16,2);

void setup() {

  //pinMode Init
  pinMode(FET_GATE, OUTPUT);
  analogWrite(FET_GATE, 127);
  
  //LCD Init 
  lcd.begin(16, 2);
  lcd.init();                      
  lcd.init();
  lcd.noBacklight(); //using open collector, therefore polarity is reversed
  
  lcd.print("hello, world!");
  delay(1000);

  
}

void loop() {
  
}
