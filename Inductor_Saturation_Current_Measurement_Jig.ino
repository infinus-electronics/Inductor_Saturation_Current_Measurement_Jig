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
#define DPAD PIN_PA1

//constants
#define VREF 5.080f
#define CHARGE_PUMP_FREQ 100000
#define MAX_PWM_FREQ 78000 //to maintain 8-bit duty cycle control
#define LONGPRESS 2500 //2500ms counts as a long press
#define LONGRETRIGGER 2000 //if the button is held down an additional 2000ms, retrigger the longpress action
#define BLINKPERIOD 500 //digit blink frequency in parameter set mode
const char modeDescription[3][17] = {"PWM", "CC Dummy Load", "Current Readout"};


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>



//global variables
volatile uint16_t latest_reading = 0; // remember to disable interrupts while reading this so it doesn't get changed by the ISR while you're halfway through reading it.
unsigned int period = 0xFFFF;

long lastButtonPress[5] = {0, 0, 0, 0, 0}; //down, left, mid, right, up
bool buttonWasPressed[5] = {false, false, false, false, false}; //are the buttons currently pressed
long lastLongPress[5] = {0, 0, 0, 0, 0}; //the last time a long-press action was triggered

typedef enum {ISat, DL, AMM, final} Mode; //saturation current, dummy load, and ammeter modes, final is there to help in mode cycling, it is not actually used
Mode mode = DL; //default to dummy load mode

//user changeable parameters
bool paramSet = false; //are we currently setting parameters?
int currentCursorPos = 0; //which thing are we setting?
const int maxCursorPos[3] = {7, 3}; //how many available positions are there to be set in each mode, 7 in ISat because 0-4 are freq whereas 5,6,7 are duty
int setLoad = 0; //DAC output word for dummy load, note that the dac resolution is 1mA per LSB
long setFreq = 1000; //target frequency out
int setDuty = 127; //target duty cycle (8 bit)


/*INIT*/
//LCD
LiquidCrystal_I2C lcd(0x39,16,2);

void setup() {

  initMUX();  
  initChargePump();
  initPWM();

  initLCD();
  lcd.setCursor(0, 0);
  lcd.printf("%-16s", modeDescription[mode]);

  initDAC();
  initADC1();

  Serial.begin(9600);
  pinMode(DBG, OUTPUT);
  pinMode(PIN_PA5, OUTPUT); //DEBUG: use this as a temporary voltage reference;
  digitalWriteFast(PIN_PA5, HIGH); //ordinary digitalWrite has some weird turn off timer stuff that breaks everything
  writeDACmV(999);

}

void loop() {

  //take a current reading first
  char result[7];
  float adc = readADC1mV();
  float mAmps = adc * 2.0f;
  dtostrf(mAmps, 6, 1, result);

  //begin button handler here
  int currentButtonState = analogRead(DPAD);

  if(currentButtonState > 46 && currentButtonState <= 135) { //down button
    downButton();
  }
  else if(currentButtonState > 135 && currentButtonState <= 247) { //left button
    leftButton();
  }
  else if (currentButtonState > 247 && currentButtonState <= 497) { //middle button
    midButton();
  }
  else if (currentButtonState > 497 && currentButtonState <= 758) { //right button
    rightButton();
  }
  else if (currentButtonState > 758) { //top button
    upButton();
  }
  else {
    //all buttons released, go ahead and clear everything
    for(int i = 0; i < 4; i++) {
      buttonWasPressed[i] = false;
    }
    
  }

  switch(mode) { //mode state machine

    case ISat:

      lcd.setCursor(0, 1);
      lcd.printf("f=%05dHz  d=%03d", setFreq, setDuty);

      break;

    case DL:

      lcd.setCursor(0, 1);
      lcd.printf("%smA  %04dmA", result, setLoad);      

      break;

    case AMM:
    
      lcd.setCursor(0, 1); //print current ammeter reading on second line
      lcd.printf("        %smA", result);
      
      break;

    default:

      break;

  }

  if( (paramSet == true) && ((millis() / BLINKPERIOD) % 2) ) { //blink digit that is being set

    switch(mode) { //mode state machine

    case ISat:

      lcd.setCursor((currentCursorPos <= 4 ? (6 - currentCursorPos) : (15 - currentCursorPos)), 1);
      lcd.print(" ");

      break;

    case DL:

      lcd.setCursor(13 - currentCursorPos, 1);
      lcd.print(" "); 

      break;

    case AMM:
    
      
      break;

    default:

      break;

  }


  }
  
  

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


//button handlers
void downButton() {

  if (buttonWasPressed[0] == false) { //button was just pressed

    buttonWasPressed[0] = true;
    lastButtonPress[0] = millis(); //track the downstroke
    //do short-press action here
    

  }
  else if ( ((millis() - lastButtonPress[0]) >= LONGPRESS) && ((millis() - lastLongPress[0]) >= LONGRETRIGGER) ) { //button was already pressed, and was held for LONGPRESS amount of miliseconds, and the longpress timeout has expired

    lastLongPress[0] = millis();
    //do long-press action here
    
  }


}

void leftButton() {
  
  if (buttonWasPressed[1] == false) { //button was just pressed

    buttonWasPressed[1] = true;
    lastButtonPress[1] = millis(); //track the downstroke
    //do short-press action here

  }
  else if ( ((millis() - lastButtonPress[1]) >= LONGPRESS) && ((millis() - lastLongPress[1]) >= LONGRETRIGGER) ) { //button was already pressed, and was held for LONGPRESS amount of miliseconds, and the longpress timeout has expired

    lastLongPress[1] = millis();
    //do long-press action here

    switch(mode) { //mode was changed, do all the requisite updating

      case ISat:
        
        //go to ammeter mode
        mode = AMM;

        setPWMDuty(0); //turn off PWM to reduce noise

        lcd.setCursor(0, 0);
        lcd.printf("%-16s", modeDescription[mode]);

        break;

      case DL:

        //go to inductor sat current mode
        mode = ISat;
        lcd.setCursor(0, 0);
        lcd.printf("%-16s", modeDescription[mode]);

        break;
      
      case AMM:

        //go to dummy load mode
        mode = DL;
        lcd.setCursor(0, 0);
        lcd.printf("%-16s", modeDescription[mode]);

        break;
      
      default:

        break;

    }
    
  }

}

void midButton() {

  if (buttonWasPressed[2] == false) { //button was just pressed

    buttonWasPressed[2] = true;
    lastButtonPress[2] = millis(); //track the downstroke
    //do short-press action here

  }
  else if ( ((millis() - lastButtonPress[2]) >= LONGPRESS) && ((millis() - lastLongPress[2]) >= LONGRETRIGGER) ) { //button was already pressed, and was held for LONGPRESS amount of miliseconds, and the longpress timeout has expired

    lastLongPress[2] = millis();
    //do long-press action here
    lcd.setCursor(0, 0);

    //long pressing middle button enters and exits parameter set mode
    if(paramSet == false) { //we are just about to set some parameters

      paramSet = true;
      currentCursorPos = 0; //initialise cursor position
      
      switch(mode) {

        case ISat:

          break;

        case DL:

          break;

        case AMM:

          break;

        default:
          break;

      }

    }

    else {

      paramSet = false;
      

    }
    
  }

}

void rightButton() {

  if (buttonWasPressed[3] == false) { //button was just pressed

    buttonWasPressed[3] = true;
    lastButtonPress[3] = millis(); //track the downstroke
    //do short-press action here

  }
  else if ( ((millis() - lastButtonPress[3]) >= LONGPRESS) && ((millis() - lastLongPress[3]) >= LONGRETRIGGER) ) { //button was already pressed, and was held for LONGPRESS amount of miliseconds, and the longpress timeout has expired

    lastLongPress[3] = millis();
    //do long-press action here
    
    switch(mode) { //mode was changed, do all the requisite updating

      case ISat:
        
        //go to dummy load mode
        mode = DL;
        
        setPWMDuty(0); //turn off PWM to reduce noise

        lcd.setCursor(0, 0);
        lcd.printf("%-16s", modeDescription[mode]);

        break;

      case DL:

        //go to ammeter mode
        mode = AMM;
        lcd.setCursor(0, 0);
        lcd.printf("%-16s", modeDescription[mode]);

        break;
      
      case AMM:

        //go to inductor sat current mode
        mode = ISat;
        lcd.setCursor(0, 0);
        lcd.printf("%-16s", modeDescription[mode]);

        break;
      
      default:

        break;

    }
    
  }

}

void upButton() {

  if (buttonWasPressed[4] == false) { //button was just pressed

    buttonWasPressed[4] = true;
    lastButtonPress[4] = millis(); //track the downstroke
    //do short-press action here

  }
  else if ( ((millis() - lastButtonPress[4]) >= LONGPRESS) && ((millis() - lastLongPress[4]) >= LONGRETRIGGER) ) { //button was already pressed, and was held for LONGPRESS amount of miliseconds, and the longpress timeout has expired

    lastLongPress[4] = millis();
    //do long-press action here
    
  }
}
