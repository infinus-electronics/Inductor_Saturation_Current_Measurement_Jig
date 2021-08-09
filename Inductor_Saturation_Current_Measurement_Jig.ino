//defines
//pins
#define FET_GATE PIN_PB2
#define SS PIN_PC3
#define MOSI PIN_PC2
#define SCK PIN_PC0
#define MUX_A PIN_PA3
#define MUX_B PIN_PA2

//constants
#define VREF 5.000f


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>



/*INIT*/
//LCD
LiquidCrystal_I2C lcd(0x39,16,2);

void setup() {

  initMUX();
  
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

  initDAC();
  pinMode(PIN_PA5, OUTPUT); //DEBUG: use this as a temporary voltage reference;
  digitalWrite(PIN_PA5, OUTPUT);

  
}

void loop() {
  
  writeDAC(1023);
  delay(1000);
  writeDAC(2047);
  delay(1000);
  writeDAC(4095);
  delay(1000);

}

void initDAC() {

  pinMode(SCK, OUTPUT); 
  pinMode(MOSI, OUTPUT); 
  pinMode(SS, OUTPUT); 

  digitalWrite(SS, HIGH); //SS high while we are setting up the SPI

  SPI.swap(1); //use alternate SPI pins
  SPI.begin();

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

  digitalWrite(SS, LOW); 

  SPI.transfer(0b0111 << 4); //set DAC output to 0
  SPI.transfer(0);

  digitalWrite(SS, HIGH);

  SPI.endTransaction();

}

void writeDAC(int value){

  int hibyte = (value >> 8) & 0x0f; //we need to write a total of 12+4 bytes, split them 
  int lobyte = (value & 0xff);
  
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  
  digitalWrite(SS, LOW); 

  SPI.transfer(((0b0111) << 4) | hibyte); //set DAC output to 0
  SPI.transfer(lobyte);

  digitalWrite(SS, HIGH);

  SPI.endTransaction();

}

void initMUX() {

  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);

  digitalWrite(MUX_A, LOW); //default to dummy load mode
  digitalWrite(MUX_B, HIGH);

}

void MUXPWM() { //set the mux to pass PWM from micro to gate

  digitalWrite(MUX_B, LOW);

}

void MUXDummyLoad() { //set the mux to pass opamp output to gate

  digitalWrite(MUX_B, HIGH);

}
