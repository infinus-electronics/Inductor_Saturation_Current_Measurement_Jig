//defines
//pins
#define DBG PIN_PB4
#define FET_GATE PIN_PB2
#define SS PIN_PC3
#define MOSI PIN_PC2
#define SCK PIN_PC0
#define MUX_A PIN_PA7
#define MUX_B PIN_PA6
#define CHARGE_PUMP PIN_PA3
#define PWM PIN_PB5

//constants
#define VREF 5.080f
#define CHARGE_PUMP_FREQ 100000
#define MAX_PWM_FREQ 78000 //to maintain 8-bit duty cycle control


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>



//global variables
volatile uint16_t latest_reading = 0; // remember to disable interrupts while reading this so it doesn't get changed by the ISR while you're halfway through reading it.
unsigned int period = 0xFFFF;


/*INIT*/
//LCD
LiquidCrystal_I2C lcd(0x39,16,2);

void setup() {

  initMUX();
  initChargePump();
  initPWM();
  setPWMPeriod(100000, 10000);
  pinMode(DBG, OUTPUT);
  
  //LCD Init 
  initLCD();

  initDAC();
  
  pinMode(PIN_PA5, OUTPUT); //DEBUG: use this as a temporary voltage reference;
  digitalWriteFast(PIN_PA5, HIGH); //ordinary digitalWrite has some weird turn off timer stuff that breaks everything
  
  writeDACmV(999);  

  initADC1();

}

void loop() {
  
  lcd.setCursor(0, 0);
  char result[7];
  float adc = readADC1mV();
  float mAmps = adc * 2.0f;
  dtostrf(mAmps, 6, 1, result);
  lcd.print(result);
  lcd.print(" mA");
  

  delay(100);

}


void initLCD() {

  lcd.begin(16, 2);
  lcd.init();                      
  lcd.init();
  lcd.noBacklight(); //using open collector, therefore polarity is reversed

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

void writeDACmV(float milivolts) {

  float data = (milivolts/VREF/1000.0f) * 4095;
  int DACword = (int)data;

  writeDAC(DACword);

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

void initADC1() {

  init_ADC1(); //set up prescalers

  ADC1.MUXPOS = 0x00; //ADC1 in 0 PA4
  ADC1.CTRLB = ADC_SAMPNUM_ACC64_gc;  //take 64 samples for 3 extra bits, total 13 bits
  ADC1.CTRLA = ADC_ENABLE_bm | ADC_FREERUN_bm; //start in freerun
  ADC1.INTCTRL = 1 << ADC_RESRDY_bp  /* Result Ready Interrupt Enable: enabled */
                | 0 << ADC_WCMP_bp; /* Window Comparator Interrupt Enable: disabled */
  ADC1.COMMAND = ADC_STCONV_bm; //start first conversion!

}

float readADC1mV() {

  cli();
  uint16_t rdg = latest_reading; //grab a copy of the reading without being interrupted to prevent corruption
  sei();

  //TODO: Optimise away this floating point garbage
  float mVoltage = ((float)rdg) / 8191.0f * VREF * 1000.0f; //13-bits max oversampling and decimation

  return mVoltage;

}

ISR(ADC1_RESRDY_vect) {

  uint16_t raw_reading = ADC1.RES;
  latest_reading = raw_reading >> 3; // 13-bits

}

void initChargePump() {

  pinMode(CHARGE_PUMP, OUTPUT);

  TCB1.CCMP = 0x7FFF; //50% duty cycle
  TCB1.CTRLB = TCB_CCMPEN_bm | 0b111 ; //enable waveform output, 8-bit PWM mode
  TCB1.CTRLA = TCB_ENABLE_bm; //start the timer

}

void initPWM() { //Waveform generation for inductor saturation current measurement

  pinMode(PWM, OUTPUT);

  PORTMUX.CTRLC = PORTMUX_TCA02_bm; //set Waveform output 2 to alternate location

  TCA0.SPLIT.CTRLA = 0; //disable TCA0 and set divider to 1
  TCA0.SPLIT.CTRLESET = TCA_SPLIT_CMD_RESET_gc|0x03; //set CMD to RESET, and enable on both pins.
  TCA0.SPLIT.CTRLD = 0; //Split mode now off, CMPn = 0, CNT = 0, PER = 255

  TCA0.SINGLE.CTRLB = (TCA_SINGLE_CMP2EN_bm|TCA_SINGLE_WGMODE_SINGLESLOPE_gc); //Single slope PWM mode, PWM on WO0
  TCA0.SINGLE.PER = 0xFFFF; // Count all the way up to 0xFFFF
  // At 20MHz, this gives ~305Hz PWM
  TCA0.SINGLE.CMP2 = 0;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm; //enable the timer with no prescaler
}


void setPWMFreq(unsigned long freq) {

  freq = constrain(freq, 1, MAX_PWM_FREQ);

  unsigned long tempPeriod = F_CPU / freq;

  byte presc = 0;

  while(tempPeriod > 65535 && presc < 7) {

    presc++;
    tempPeriod = tempPeriod >> (presc > 4 ? 2 : 1);

  }
  period = tempPeriod;

  TCA0.SINGLE.CTRLA = (presc << 1) | TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.PER = period; 
  

}

void setPWMDuty(int duty){

  TCA0.SINGLE.CMP2 = map(duty, 0, 255, 0, period);

}

void setPWMPeriod(long period_uS, long highPeriod_uS){

  period_uS = constrain(period_uS, 0, 1000000);
  highPeriod_uS = constrain(highPeriod_uS, 0, period_uS);

  long targetFreq = 1000000 / period_uS;
  int targetDuty = (255L * highPeriod_uS / period_uS);

  setPWMFreq(targetFreq);
  setPWMDuty(targetDuty);

}

