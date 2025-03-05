/*
This is an electric smoker controller written by Darren Chapman
copywrite 2025
This program is free to use and modify for personal use.
Commercialization is forbidden unless approved by Darren Chapman
contact dmc_md@hotmail.com with questions/comments
*/

//ADJUST THESE PINS TO FIT HOW YOU BUILT THE CONTROLLER BOX
//IF THE PLUGS DON'T MATCH ON THE BOX WHERE YOU WANT THEM TO BE, CHANGE THESE PINS AROUND
// named constants for the thermometer pins
const int pitTempPin = A0;
const int meat1Pin = A1;
const int meat2Pin = A2;
const int meat3Pin = A3;

//SWITCH THESE TWO PINS IF YOUR ENCODER IS ROTATING THE WRONG WAY
// set up the rotary encoder pins
const int encoderPinA = 3;
const int encoderPinB = 2;

//YOU CAN SET HOW HOT YOUR SMOKER IS ALLOWED TO GO WITH THIS NUMBER
//DEFAULT IS 210 BECAUSE I MADE THIS TO SMOKE SAUSAGE
//BUT IF YOU'RE USING IT FOR OTHER PURPOSES, YOU CAN MAKE THIS NUMBER GO AS HIGH AS YOU'D LIKE
int maxTempMax = 210;


// include these library codes:
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <PID_v1.h>

LiquidCrystal_I2C lcd(0x3F, 20, 4);

// EEPROM storage setup
int kpAddress = 8;
int kiAddress = 12;
int kdAddress = 16;
int setPointAddress = 20;
int keepWarmAddress = 24;
int heatProfileAddress = 28;
int meatSetPointAddress = 32;
int deltaStartTempAddress = 36;
int tempDeltaAddress = 40;
int timer1MinutesAddress = 44;
int timer2MinutesAddress = 48;
int timer3MinutesAddress = 52;
int timer4MinutesAddress = 56;
int timer5MinutesAddress = 60;
int timer6MinutesAddress = 64;
int timer1SetPointAddress = 68;
int timer2SetPointAddress = 72;
int timer3SetPointAddress = 76;
int timer4SetPointAddress = 80;
int timer5SetPointAddress = 84;
int timer6SetPointAddress = 88;
int finalTempAddress = 92;
int maxTempAddress = 96;

// set up the rotary encoder button pin
const int encoderButtonPin = 8;

// temp timer reading variables
const long tempTimerInterval = 1000;
unsigned long prevTempTimerMillis = 0;

// sensor value variables
int temp;
int pitTemp;
int meat1;
int meat2;
int meat3;
int meat1Temp;
int meat2Temp;
int meat3Temp;

// rotary encoder variables:
volatile int encoderPos = 0;
unsigned int lastReportedPos = 1;
static boolean rotating = false;
boolean A_set = false;
boolean B_set = false;
unsigned char encoderA;
unsigned char encoderB;
unsigned char encoderAPrev = 0;
unsigned long encoderTime;
int debounce = 2;

// create set screen variable
int setScreen = 0;

// set up button press counter
int buttonState = 0;
int lastButtonState = 1;

//variables for menu time out timer
unsigned long menuTimeOutMillis;
unsigned long prevMenuTimeOutMillis;
int menuTimeOutInterval = 20000;

//clear screen variable
bool lcdReset;

// set up relay output pin
int relay = 6;

// PID library and variables
double Input, Output;
double setPoint;
double Kp, Ki, Kd;
PID myPID(&Input, &Output, &setPoint, Kp, Ki, Kd, DIRECT);
double newKp;
double newKii;
double newKd;
int intnewKp = (int)newKp;
int intnewKd = (int)newKd;
int newIntSetPoint;
int intSetPoint;
bool changeConstant = 0;
bool setMode = 0;

byte arrow[8] = {
  B01000,
  B00100,
  B00010,
  B11111,
  B00010,
  B00100,
  B01000,
  B00000,
};

int maxTemp;
bool advanced = 0;
unsigned long previousMillis = 0;
bool blink = 0;
int subSubMenu = 0;
int subMenu = 0;
bool kpChangeVar = 0;
bool kiChangeVar = 0;
bool kdChangeVar = 0;
int plateValue = 0;
int interimValue = 0;
bool pitProbeDetect = 0;
bool startup = 1;
bool prevstartup = 0;

//variables for warm up overshoot function
bool warmUpOverShoot = 0;
unsigned long warmUpOverShootMillis;
unsigned long prevWarmUpOverShootMillis;

//relay control variables
int plateMillis;
unsigned long plateTimerMillis;
unsigned long prevPlateTimerMillis;
unsigned long relayOnMillis;
unsigned long prevRelayOnMillis;
int loopMillis = 5000;
int percentMillis = 500;
bool relayOn = 0;

// keepwarm variables
int keepWarm;
int keepWarmMax = 180;
bool flash;
unsigned long flashMillis;

// heat profile variables
int heatProfile;
int deltaStartTemp;
int intermediate;
int deltaSubMenu = 0;
int maxTempDelta = 100;
int tempDelta;

// meat set point variable
int meatSetPoint;
bool meatTest = 0;
bool meat1Test = 0;
bool meat2Test = 0;
bool meat3Test = 0;

// timer variables
int timer1Minutes;
int timer2Minutes;
int timer3Minutes;
int timer4Minutes;
int timer5Minutes;
int timer6Minutes;
int timer1SetPoint;
int timer2SetPoint;
int timer3SetPoint;
int timer4SetPoint;
int timer5SetPoint;
int timer6SetPoint;
int finalTemp;
int stepperMenu = 0;
bool setSteps = 0;
bool stepReturn = 0;
bool defaultsReset = 0;
int stepFunction = 0;
bool stepChange = 0;
bool minTemp = 0;
bool timer1Done = 0;
bool timer2Done = 0;
bool timer3Done = 0;
bool timer4Done = 0;
bool timer5Done = 0;
bool timer6Done = 0;
unsigned long stepStartMillis = 0;
unsigned long timer1Millis;
unsigned long timer2Millis;
unsigned long timer3Millis;
unsigned long timer4Millis;
unsigned long timer5Millis;
unsigned long timer6Millis;
unsigned long currentTimerMillis;
bool clockReset = 0;
unsigned long seconds;
unsigned long minutes;
unsigned long currentTime = 0;
unsigned long prevTime = 0;

void setup() {
  //turn on serial port to log data
  Serial.begin(9600);

  //start with the PID controller off
  myPID.SetMode(MANUAL);

  // tell the arduino to use the AREF input pin to set the analog voltage reference to 3.3V
  analogReference(EXTERNAL);

  // Switch on the backlight
  lcd.backlight();

  // set up the number of columns and rows on the LCD
  lcd.init();
  lcd.clear();

  // set up encoder pins
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderButtonPin, INPUT);

  // set up relay pin
  pinMode(relay, OUTPUT);

  // turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  digitalWrite(encoderButtonPin, HIGH);

  // make sure relay is turned off
  digitalWrite(relay, LOW);

  // **** set up interrupts
  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(1, doEncoderB, CHANGE);

  setScreen = 0;
  prevTempTimerMillis = millis();

  //reset lcdReset variable
  lcdReset = 0;
  EEPROM.get(setPointAddress, intSetPoint);
  setPoint = (double)(intSetPoint);

  lcd.createChar(0, arrow);

}


void loop() {

  // assign the variable currentMillis to millis
  unsigned long currentMillis = millis();

  // clear screen
  if (lcdReset == 0) {
    lcd.clear();
    lcdReset = !lcdReset;
  }

  // get PID values and setpoint values from permanent memory
  EEPROM.get(kpAddress, newKp);
  EEPROM.get(kiAddress, newKii);
  EEPROM.get(kdAddress, newKd);
  EEPROM.get(setPointAddress, newIntSetPoint);
  EEPROM.get(keepWarmAddress, keepWarm);
  EEPROM.get(heatProfileAddress, heatProfile);
  EEPROM.get(meatSetPointAddress, meatSetPoint);
  EEPROM.get(deltaStartTempAddress, deltaStartTemp);
  EEPROM.get(tempDeltaAddress, tempDelta);
  EEPROM.get(timer1MinutesAddress, timer1Minutes);
  EEPROM.get(timer1SetPointAddress, timer1SetPoint);
  EEPROM.get(timer2MinutesAddress, timer2Minutes);
  EEPROM.get(timer2SetPointAddress, timer2SetPoint);
  EEPROM.get(timer3MinutesAddress, timer3Minutes);
  EEPROM.get(timer3SetPointAddress, timer3SetPoint);
  EEPROM.get(timer4MinutesAddress, timer4Minutes);
  EEPROM.get(timer4SetPointAddress, timer4SetPoint);
  EEPROM.get(timer5MinutesAddress, timer5Minutes);
  EEPROM.get(timer5SetPointAddress, timer5SetPoint);
  EEPROM.get(timer6MinutesAddress, timer6Minutes);
  EEPROM.get(timer6SetPointAddress, timer6SetPoint);
  EEPROM.get(finalTempAddress, finalTemp);
  EEPROM.get(maxTempAddress, maxTemp);


  //check to see if any of the PID values have changed, and if so, then set the tuning of the PID controller
  if (Kp != newKp) {
    Kp = newKp;
    changeConstant = 1;
  }
  if (Ki != newKii) {
    Ki = newKii;
    changeConstant = 1;
  }
  if (Kd != newKd) {
    Kd = newKd;
    changeConstant = 1;
  }
  if ((heatProfile == 0) && (meatTest == 0)) {
    if (intSetPoint != newIntSetPoint) {
      intSetPoint = newIntSetPoint;
      setPoint = (double)intSetPoint;
    }
  }
  if ((heatProfile == 1) && (startup == 0) && (prevstartup == 1) && (warmUpOverShoot == 1) && (setMode == 1) && (meatTest == 0)) {
    if (meat1Temp < (deltaStartTemp - tempDelta)) {
      setPoint = (double)deltaStartTemp;
    }
    if (meat1Temp >= (deltaStartTemp - tempDelta)) {
      intermediate = (meat1Temp + tempDelta);
      if (intermediate > maxTemp) {
        intermediate = maxTemp;
      }
      setPoint = (double)intermediate;
    }
  }

  // this section is the stepper timer control

  if ((heatProfile == 2) && (startup == 0) && (prevstartup == 1) && (warmUpOverShoot == 1) && (setMode == 1) && (meatTest == 0)) {
    if (timer1Done == 0) {
      if (timer1Minutes == 0) {
        timer1Done = 1;
      }
      setPoint = (double)timer1SetPoint;
      timer1Millis = (timer1Minutes * 60000);
      if (clockReset == 0) {
        stepStartMillis = millis();
        minutes = timer1Minutes;
        seconds = 0;
        prevTime = millis();
        clockReset = 1;
      }
      //check to see if timer is up
      currentTimerMillis = millis();
      if ((currentTimerMillis - stepStartMillis) >= timer1Millis) {
        clockReset = 0;
        timer1Done = 1;
      }
    }
    if ((timer1Done == 1) && (timer2Done == 0)) {
      if (timer2Minutes == 0) {
        timer2Done = 1;
      }
      setPoint = (double)timer2SetPoint;
      timer2Millis = (timer2Minutes * 60000);
      if (clockReset == 0) {
        stepStartMillis = millis();
        minutes = timer2Minutes;
        seconds = 0;
        prevTime = millis();
        clockReset = 1;
      }
      //check to see if timer is up
      currentTimerMillis = millis();
      if ((currentTimerMillis - stepStartMillis) >= timer2Millis) {
        clockReset = 0;
        timer2Done = 1;
      }
    }
    if ((timer1Done == 1) && (timer2Done == 1) && (timer3Done == 0)) {
      if (timer3Minutes == 0) {
        timer3Done = 1;
      }
      setPoint = (double)timer3SetPoint;
      timer3Millis = (timer3Minutes * 60000);
      if (clockReset == 0) {
        stepStartMillis = millis();
        minutes = timer3Minutes;
        seconds = 0;
        prevTime = millis();
        clockReset = 1;
      }
      //check to see if timer is up
      currentTimerMillis = millis();
      if ((currentTimerMillis - stepStartMillis) >= timer3Millis) {
        clockReset = 0;
        timer3Done = 1;
      }
    }
    if ((timer1Done == 1) && (timer2Done == 1) && (timer3Done == 1) && (timer4Done == 0)) {
      if (timer4Minutes == 0) {
        timer4Done = 1;
      }
      setPoint = (double)timer4SetPoint;
      timer4Millis = (timer4Minutes * 60000);
      if (clockReset == 0) {
        stepStartMillis = millis();
        minutes = timer4Minutes;
        seconds = 0;
        prevTime = millis();
        clockReset = 1;
      }
      //check to see if timer is up
      currentTimerMillis = millis();
      if ((currentTimerMillis - stepStartMillis) >= timer4Millis) {
        clockReset = 0;
        timer4Done = 1;
      }
    }
    if ((timer1Done == 1) && (timer2Done == 1) && (timer3Done == 1) && (timer4Done == 1) && (timer5Done == 0)) {
      if (timer5Minutes == 0) {
        timer5Done = 1;
      }
      setPoint = (double)timer5SetPoint;
      timer5Millis = (timer5Minutes * 60000);
      if (clockReset == 0) {
        stepStartMillis = millis();
        minutes = timer5Minutes;
        seconds = 0;
        prevTime = millis();
        clockReset = 1;
      }
      //check to see if timer is up
      currentTimerMillis = millis();
      if ((currentTimerMillis - stepStartMillis) >= timer5Millis) {
        clockReset = 0;
        timer5Done = 1;
      }
    }
    if ((timer1Done == 1) && (timer2Done == 1) && (timer3Done == 1) && (timer4Done == 1) && (timer5Done == 1) && (timer6Done == 0)) {
      if (timer6Minutes == 0) {
        timer6Done = 1;
      }
      setPoint = (double)timer6SetPoint;
      timer6Millis = (timer6Minutes * 60000);
      if (clockReset == 0) {
        stepStartMillis = millis();
        minutes = timer6Minutes;
        seconds = 0;
        prevTime = millis();
        clockReset = 1;
      }
      //check to see if timer is up
      currentTimerMillis = millis();
      if ((currentTimerMillis - stepStartMillis) >= timer6Millis) {
        clockReset = 0;
        timer6Done = 1;
      }
    }
    if ((timer1Done == 1) && (timer2Done == 1) && (timer3Done == 1) && (timer4Done == 1) && (timer5Done == 1) && (timer6Done == 1)) {
      setPoint = (double)finalTemp;
    }

    //print out the time left of the current step as a countdown
    if ((timer1Done == 1) && (timer2Done == 1) && (timer3Done == 1) && (timer4Done == 1) && (timer5Done == 1) && (timer6Done == 1)) {
      lcd.setCursor(14, 3);
      lcd.print("      ");
    }
    else {
      currentTime = millis();
      if (currentTime - prevTime >= 1000) {
        prevTime = currentTime;
        seconds--;
      }
      if (seconds == -1) {
        minutes--;
        seconds = 59;
      }
      lcd.setCursor(14, 3);
      if (minutes < 100) {
        lcd.print(" ");
      }
      if (minutes < 10) {
        lcd.print(" ");
      }
      lcd.print(minutes);
      lcd.print(":");
      if (seconds < 10) {
        lcd.print("0");
      }
      lcd.print(seconds);
    }
  }


  // turn the pid controller on if it's already done the startup and warm up overshoot,
  // and set the tunings of the pid controller if they've been changed
  if ((pitProbeDetect == 1) && (startup == 0) && (prevstartup == 1) && (warmUpOverShoot == 1) && (setMode == 0)) {
    myPID.SetMode(AUTOMATIC);
    if (changeConstant == 1) {
      myPID.SetTunings(Kp, Ki, Kd);
      changeConstant = 0;
    }
    setMode = 1;
  }

  // if meat is at setpoint, change temp target to keepwarm temp
  if (meatTest == 1) {
    setPoint = (double)keepWarm;
  }
  //flash screen if meat is done
  if (meatTest == 1) {
    if (flash == 1) {
      lcd.backlight();
      if (currentMillis - flashMillis >= 750) {
        flashMillis = currentMillis;
        flash = 0;
      }
    }
    if (flash == 0) {
      lcd.noBacklight();
      if (currentMillis - flashMillis >= 250) {
        flashMillis = currentMillis;
        flash = 1;
      }
    }
  }
  if (meatTest == 0) {
    lcd.backlight();
    flash = 1;
  }

  // read temperature on pit thermometer every second and then plug it into the PID and send it to serial logger
  unsigned long tempMillis = millis();
  if (tempMillis - prevTempTimerMillis >= tempTimerInterval) {
    prevTempTimerMillis = tempMillis;
    pitTemp = analogRead(pitTempPin);
    meat1 = analogRead(meat1Pin);
    meat2 = analogRead(meat2Pin);
    meat3 = analogRead(meat3Pin);
    Serial.print(int(Input));
    Serial.print(",");
    Serial.print(int(setPoint));
    Serial.print(",");
    Serial.print(meat1Temp);
    Serial.println();
  }
  Input = tempFunctionHot(pitTemp);
  meat1Temp = tempFunctionHot(meat1);
  meat2Temp = tempFunctionHot(meat2);
  meat3Temp = tempFunctionHot(meat3);

  // check to see if any of the meat probes have hit the target temp
  if (meat1Temp >= meatSetPoint) {
    meat1Test = 1;
  }
  if (meat2Temp >= meatSetPoint) {
    meat2Test = 1;
  }
  if (meat3Temp >= meatSetPoint) {
    meat3Test = 1;
  }
  if ((meat1Test == 1) || (meat2Test == 1) || (meat3Test == 1)) {
    meatTest = 1;
  }
  if (meat1Temp < -10) {
    meat1Test = 0;
  }
  if (meat2Temp < -10) {
    meat2Test = 0;
  }
  if (meat3Temp < -10) {
    meat3Test = 0;
  }
  if ((meat1Test == 0) && (meat2Test == 0) && (meat3Test == 0)) {
    meatTest = 0;
  }

  // check to see if there's a pit probe
  if (Input > -10) {
    pitProbeDetect = 1;
  }
  if (Input <= -10) {
    pitProbeDetect = 0;
  }

  //check to see if it is in startup mode
  if ((pitProbeDetect == 1) && (startup == 0) && (prevstartup == 0) && (meatTest == 0)) {
    startup = 1;
  }
  if ((heatProfile == 2) && (pitProbeDetect == 1) && (startup == 1) && (meatTest == 0)) {
    setPoint = double(timer1SetPoint);
  }
  if (((heatProfile == 0) || (heatProfile == 2)) && (pitProbeDetect == 1) && ((setPoint - Input) <= 0) && (startup == 1) && (meatTest == 0)) {
    startup = 0;
    prevstartup = 1;
    warmUpOverShootMillis = millis();
    prevWarmUpOverShootMillis = warmUpOverShootMillis;
    stepStartMillis = millis();
  }
  if ((heatProfile == 1) && (pitProbeDetect == 1) && (Input >= deltaStartTemp) && (startup == 1) && (meatTest == 0)) {
    startup = 0;
    prevstartup = 1;
    warmUpOverShootMillis = millis();
    prevWarmUpOverShootMillis = warmUpOverShootMillis;
  }
  // this is the output during startup mode
  if ((startup == 1) && (pitProbeDetect == 1) && (meatTest == 0)) {
    myPID.SetMode(MANUAL);
    plateValue = 10;
  }
  // this changes the hotplate to 0 for a quarter second before switching on the PID controller to control overshoot
  if ((startup == 0) && (pitProbeDetect == 1) && (warmUpOverShoot == 0) && (meatTest == 0)) {
    myPID.SetMode(MANUAL);
    plateValue = 0;
    warmUpOverShootMillis = millis();

    //put in the quarter second timer
    if (warmUpOverShootMillis - prevWarmUpOverShootMillis >= 250) {
      warmUpOverShoot = 1;
    }
  }

  if ((startup == 0) && (pitProbeDetect == 1) && (warmUpOverShoot == 1) && (prevstartup == 1) && (setMode == 1)) {
    myPID.Compute();
    plateValue = map(Output, 0, 255, 0, 10);
  }

  if (pitProbeDetect == 1) {
    lcd.setCursor(0, 0);
    lcd.print("Pit: ");
    lcd.print(int(Input));
    lcd.print(" (");
    if ((startup == 1) && (heatProfile == 1)) {
      lcd.print(deltaStartTemp);
    }
    else {
      lcd.print(int(setPoint));
    }
    lcd.print(") ");
  }

  // if there's not a pit probe, print it out and turn off PID controller and set output to 0
  if (pitProbeDetect == 0) {
    myPID.SetMode(MANUAL);
    plateValue = 0;
    setMode = 0;
    lcd.setCursor(0, 0);
    lcd.print("No Pit Probe ");
  }
  // check to see if there's a meat1 probe and if so print out the temp
  if (meat1Temp > -10) {
    lcd.setCursor(0, 1);
    lcd.print(int(meat1Temp));
    lcd.print(" ");
  }

  // check to see if there's a meat2 probe and if so print out the temp
  if (meat2Temp > -10) {
    lcd.setCursor(4, 1);
    lcd.print(int(meat2Temp));
    lcd.print(" ");
  }

  // check to see if there's a meat3 probe and if so print out the temp
  if (meat3Temp > -10) {
    lcd.setCursor(8, 1);
    lcd.print(int(meat3Temp));
    lcd.print(" ");
  }

  // notify if there's no meat1 probe
  if (meat1Temp < -10) {
    lcd.setCursor(0, 1);
    lcd.print("n/a");
  }

  // notify if there's no meat2 probe
  if (meat2Temp < -10) {
    lcd.setCursor(4, 1);
    lcd.print("n/a");
  }

  // notify if there's no meat3 probe
  if (meat3Temp < -10) {
    lcd.setCursor(8, 1);
    lcd.print("n/a");
  }

  // print out meat set point
  lcd.setCursor(15, 1);
  lcd.print("(");
  lcd.print(meatSetPoint);
  lcd.print(")");
  if (meatSetPoint < 100) {
    lcd.print(" ");
  }

  // print out keepwarm set point
  lcd.setCursor(15, 0);
  lcd.print("(");
  lcd.print(keepWarm);
  lcd.print(")");
  if (keepWarm < 100) {
    lcd.print(" ");
  }


  // print out PID output and plate value
  if (startup == 0) {
    lcd.setCursor(0, 2);
    lcd.print("PID: ");
    lcd.print(int(Output));
    lcd.print(" Heat: ");
    lcd.print(plateValue);
    if (plateValue > 0) {
      lcd.print("0% ");
    }
    if (plateValue == 0) {
      lcd.print("%   ");
    }
  }
  if (startup == 1) {
    lcd.setCursor(0, 2);
    lcd.print("Startup");
    lcd.print(" Heat: ");
    lcd.print(plateValue);
    lcd.print("0% ");
  }

  //check to see if the encoder button has been pressed to enter set mode
  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      // add 1 to the setScreen variable
      lastReportedPos = encoderPos;
      setScreen++;
      if ((heatProfile == 1) || (heatProfile == 2)){
        setScreen++;
      }
    }
    // if set button has been pressed once, run pit temp function
    while ((setScreen == 1) && (heatProfile == 0)) {
      // run the pit temp function
      pitSetPointFun();
    }
    lcdReset = 0;
    while (setScreen == 2) {
      // run the meat setpoint function
      meatSetPointFun();
    }
    lcdReset = 0;

    while (setScreen == 3) {

      menuFun();
    }
    lcdReset = 0;
    while ((setScreen == 4) && (advanced == 1)) {
      advancedOptionsFun();
    }
    lcdReset = 0;
    while ((setScreen == 4) && (advanced == 0)) {
      setScreen++;
    }
    while ((setScreen == 5) && (subMenu == 0) && (advanced == 1)) {
      pidChangeFun();
    }
    while ((setScreen == 5) && (subMenu == 1) && (advanced == 1)) {
      tempSelectFun();
    }
    while ((setScreen == 5) && (subMenu == 2) && (advanced == 1)) {
      keepWarmFun();
    }
    while ((setScreen == 5) && (subMenu == 3) && (advanced == 1)) {
      maxTempChangeFun();
    }
    while ((setScreen == 5) && (subMenu == 4) && (advanced == 1)) {
      defaultsFun();
    }
    if ((setScreen == 5) && (subMenu == 5)) {
      setScreen++;
    }
    if ((setScreen == 5) && (advanced == 0)) {
      setScreen++;
    }
    //if the set button has been pressed three times, return to normal
    if (setScreen >= 6) {
      lcd.clear();
      setScreen = 0;
    }
  }
  lastButtonState = buttonState;

  // control the relay output
  if (plateValue >= 10) {
    relayOn = 1;
  }
  if (plateValue <= 0) {
    relayOn = 0;
  }
  if ((plateValue >= 1) && (plateValue <= 9)) {
    plateMillis = (plateValue * percentMillis);
    plateTimerMillis = millis();
    relayOnMillis = millis();
    if (relayOnMillis - prevRelayOnMillis >= plateMillis) {
      relayOn = 0;
    }
    if (plateTimerMillis - prevPlateTimerMillis >= loopMillis) {
      prevRelayOnMillis = millis();
      prevPlateTimerMillis = millis();
      relayOn = 1;
    }
  }
  digitalWrite(relay, relayOn);
  lcd.setCursor(0, 3);
  lcd.print("Element: ");
  lcd.setCursor(9, 3);
  if (relayOn == HIGH) {
    lcd.print("ON ");
  }
  lcd.setCursor(9, 3);
  if (relayOn == LOW) {
    lcd.print("OFF");
  }

  lcd.setCursor(14, 3);
  if (heatProfile == 0) {
    lcd.print("Manual");
  }
  if (heatProfile == 1) {
    lcd.print(" Delta");
  }
  if ((heatProfile == 2) && (startup == 1)) {
    lcd.print(" Timer");
  }
}


// Interrupt on A changing state
void doEncoderA() {
  // debounce
  if (rotating) {
    delay(debounce);  // wait a little until the bouncing is done
  }
  // Test transition, did things really change?
  if (digitalRead(encoderPinA) != A_set) {  // debounce once more
    A_set = !A_set;

    // adjust counter -1 if A leads B
    if (A_set && !B_set)
      encoderPos -= 1;
    prevMenuTimeOutMillis = millis();

    rotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB() {
  if (rotating) {
    delay(debounce);
  }
  if (digitalRead(encoderPinB) != B_set) {
    B_set = !B_set;
    //  adjust counter + 1 if B leads A
    if (B_set && !A_set)
      encoderPos += 1;
    prevMenuTimeOutMillis = millis();

    rotating = false;
  }
}


//calculate temperature of hot probe function
int tempFunctionHot(int value) {
  int result;
  double RR, TT;
  RR = log((1 / ((1024 / (double)value) - 1)) * (double)22000);
  TT = (1 / ((0.7876984931E-3) + (2.069666861E-4) * RR + (1.202652917E-7) * RR * RR * RR)) - 273.25;
  result = ((int)((TT * 9.0) / 5.0 + 32.0));
  return result;
}



//pit set point function
void pitSetPointFun() {

  if (heatProfile == 0) {
    menuTimeOutMillis = millis();
    if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
      setScreen = 10;
    }
    rotating = true;
    if (lcdReset == 1) {
      lcd.clear();
      lcdReset = !lcdReset;
    }

    lastButtonState = buttonState;
    // print message that you are in pit temp setpoint mode
    lcd.setCursor(2, 0);
    lcd.print("Set Target Temp:");

    if (lastReportedPos != encoderPos) {

      int encoderChange = (encoderPos - lastReportedPos);
      if (encoderChange > 0) {
        encoderChange = 1;
      }
      if (encoderChange < 0) {
        encoderChange = -1;
      }

      intSetPoint = (intSetPoint + encoderChange);
      lastReportedPos = encoderPos;
      if (intSetPoint > maxTemp) {
        intSetPoint = maxTemp;
      }
    }

    lcd.setCursor(8, 2);
    lcd.print(intSetPoint);
    lcd.print(" ");
    // check to see if the encoder button has been pressed to enter set mode
    buttonState = digitalRead(encoderButtonPin);
    if (buttonState != lastButtonState) {
      if (buttonState == HIGH) {
        encoderPos = 0;
        lastReportedPos = 0;
        EEPROM.put(setPointAddress, intSetPoint);
        lcd.clear();
        intSetPoint--;
        // reset the menu time out timer
        prevMenuTimeOutMillis = millis();
        setScreen++;
      }
    }
  }
  else {
    setScreen++;
  }
}

void menuFun() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;
  // print message that you are in fan override mode
  lcd.setCursor(2, 0);
  lcd.print("Change Advanced");
  lcd.setCursor(3, 1);
  lcd.print("Menu Options?");
  lcd.setCursor(8, 3);
  if (advanced == 0) {
    lcd.print ("NO ");
  }
  if (advanced == 1) {
    lcd.print ("YES");
  }
  if (lastReportedPos != encoderPos) {
    advanced = !advanced;
    lastReportedPos = encoderPos;
  }
  // check to see if the encoder button has been pressed to enter set mode
  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      lcd.clear();
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      setScreen++;
    }
  }

}

void pidChangeFun() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
  }
  // debounce = 1;
  rotating = true;
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 500) {
    previousMillis = currentMillis;
    blink = !blink;
  }
  if (blink == 0) {
    lcd.setCursor(0, 0);
    lcd.print("   Kp: ");
    lcd.print(newKp);
    lcd.setCursor(13, 0);
    lcd.print("(");
    lcd.print(Kp);
    lcd.print(") ");
    lcd.setCursor(0, 1);
    lcd.print("   Ki: ");
    lcd.print(newKii);
    lcd.setCursor(13, 1);
    lcd.print("(");
    lcd.print(Ki);
    lcd.print(") ");
    lcd.setCursor(0, 2);
    lcd.print("   Kd: ");
    lcd.print(newKd);
    lcd.setCursor(13, 2);
    lcd.print("(");
    lcd.print(Kd);
    lcd.print(") ");
    lcd.setCursor(0, 3);
    lcd.print("   Save and Return");
  }
  if ((subSubMenu == 0) && (blink == 1)) {
    lcd.setCursor(0, 0);
    lcd.print("-");
    lcd.write(byte(0));
    lcd.setCursor(0, 1);
    lcd.print("  ");
    lcd.setCursor(0, 3);
    lcd.print("  ");
  }
  if ((subSubMenu == 1) && (blink == 1)) {
    lcd.setCursor(0, 1);
    lcd.print("-");
    lcd.write(byte(0));
    lcd.setCursor(0, 0);
    lcd.print("  ");
    lcd.setCursor(0, 2);
    lcd.print("  ");
  }
  if ((subSubMenu == 2) && (blink == 1)) {
    lcd.setCursor(0, 2);
    lcd.print("-");
    lcd.write(byte(0));
    lcd.setCursor(0, 1);
    lcd.print("  ");
    lcd.setCursor(0, 3);
    lcd.print("  ");
  }
  if ((subSubMenu == 3) && (blink == 1)) {
    lcd.setCursor(0, 3);
    lcd.print("-");
    lcd.write(byte(0));
    lcd.setCursor(0, 2);
    lcd.print("  ");
    lcd.setCursor(0, 0);
    lcd.print("  ");
  }
  if (lastReportedPos != encoderPos) {
    blink = 1;
    previousMillis = currentMillis;
    int subSubMenutest = (subSubMenu + (encoderPos - lastReportedPos));
    if (((subSubMenutest - subSubMenu) > 1) || ((subSubMenu - subSubMenutest) > 1)) {
      subSubMenutest = subSubMenu;
    }
    subSubMenu = subSubMenutest;
    lastReportedPos = encoderPos;

    if (subSubMenu >= 4) {
      subSubMenu = 0;
    }
    if (subSubMenu < 0) {
      subSubMenu = 3;
    }
  }

  // check to see if the encoder button has been pressed to enter set mode
  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState) {
    if ((buttonState == HIGH) && (subSubMenu == 0)) {
      encoderPos = 0;
      lastReportedPos = 0;
      //reinitialize lcd just in case
      //lcd.begin(20, 4);
      //   debounce = 1;
      kpChangeVar = !kpChangeVar;
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      kpChange();
    }
    if ((buttonState == HIGH) && (subSubMenu == 1)) {
      encoderPos = 0;
      lastReportedPos = 0;
      //reinitialize lcd just in case
      //lcd.begin(20, 4);
      //  debounce = 1;
      kiChangeVar = !kiChangeVar;
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      kiChange();
    }
    if ((buttonState == HIGH) && (subSubMenu == 2)) {
      encoderPos = 0;
      lastReportedPos = 0;
      //reinitialize lcd just in case
      //lcd.begin(20, 4);
      //   debounce = 1;
      kdChangeVar = !kdChangeVar;
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      kdChange();
    }
    if ((buttonState == HIGH) && (subSubMenu == 3)) {
      encoderPos = 0;
      lastReportedPos = 0;
      //reinitialize lcd just in case
      //lcd.begin(20, 4);
      //  debounce = 1;
      lcd.clear();
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      setScreen++;
    }
  }
}

void kpChange() {
  intnewKp = (int)newKp;
  while (kpChangeVar == 1) {
    menuTimeOutMillis = millis();
    if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
      prevMenuTimeOutMillis = millis();
      kpChangeVar = !kpChangeVar;
    }
    //   debounce = 8;
    rotating = true;
    lastButtonState = buttonState;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 500) {
      previousMillis = currentMillis;
      blink = !blink;
    }
    if (blink == 0) {
      lcd.setCursor(0, 0);
      lcd.print("-");
      lcd.write(byte(0));
      lcd.setCursor(3, 0);
      lcd.print("Kp: ");
      lcd.print(newKp);
      lcd.print(" ");
      lcd.setCursor(13, 0);
      lcd.print("(");
      lcd.print(Kp);
      lcd.print(") ");
      lcd.setCursor(3, 1);
      lcd.print("Ki: ");
      lcd.print(newKii);
      lcd.setCursor(13, 1);
      lcd.print("(");
      lcd.print(Ki);
      lcd.print(") ");
      lcd.setCursor(3, 2);
      lcd.print("Kd: ");
      lcd.print(newKd);
      lcd.setCursor(13, 2);
      lcd.print("(");
      lcd.print(Kd);
      lcd.print(") ");
      lcd.setCursor(3, 3);
      lcd.print("Save and Return");
    }
    if (blink == 1) {
      lcd.setCursor(7, 0);
      lcd.print("     ");
    }
    if (lastReportedPos != encoderPos) {
      blink = 0;
      previousMillis = currentMillis;
      int encoderChange = (encoderPos - lastReportedPos);
      if (encoderChange > 0) {
        encoderChange = 1;
      }
      if (encoderChange < 0) {
        encoderChange = -1;
      }
      intnewKp = (intnewKp + encoderChange);
      lastReportedPos = encoderPos;
      if (intnewKp < 0) {
        intnewKp = 0;
      }
      newKp = (double)intnewKp;
    }
    buttonState = digitalRead(encoderButtonPin);
    if (buttonState != lastButtonState) {
      if (buttonState == HIGH) {
        encoderPos = 0;
        lastReportedPos = 0;
        // reset the menu time out timer
        prevMenuTimeOutMillis = millis();
        EEPROM.put(kpAddress, newKp);
        //      debounce = 2;
        kpChangeVar = !kpChangeVar;
      }
    }
  }
}

void kiChange() {
  while (kiChangeVar == 1) {
    menuTimeOutMillis = millis();
    if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
      prevMenuTimeOutMillis = millis();
      kiChangeVar = !kiChangeVar;
    }
    //    debounce = 8;
    rotating = true;
    lastButtonState = buttonState;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 500) {
      previousMillis = currentMillis;
      blink = !blink;
    }
    if (blink == 0) {
      lcd.setCursor(3, 0);
      lcd.print("Kp: ");
      lcd.print(newKp);
      lcd.print(" ");
      lcd.setCursor(13, 0);
      lcd.print("(");
      lcd.print(Kp);
      lcd.print(") ");
      lcd.setCursor(0, 1);
      lcd.print("-");
      lcd.write(byte(0));
      lcd.setCursor(3, 1);
      lcd.print("Ki: ");
      lcd.print(newKii);
      lcd.setCursor(13, 1);
      lcd.print("(");
      lcd.print(Ki);
      lcd.print(") ");
      lcd.setCursor(3, 2);
      lcd.print("Kd: ");
      lcd.print(newKd);
      lcd.setCursor(13, 2);
      lcd.print("(");
      lcd.print(Kd);
      lcd.print(") ");
      lcd.setCursor(3, 3);
      lcd.print("Save and Return");
    }
    if (blink == 1) {
      lcd.setCursor(7, 1);
      lcd.print("     ");
    }
    if (lastReportedPos != encoderPos) {
      blink = 0;
      previousMillis = currentMillis;
      int encoderChange = (encoderPos - lastReportedPos);
      if (encoderChange > 0) {
        encoderChange = 1;
      }
      if (encoderChange < 0) {
        encoderChange = -1;
      }
      double halfstep = ((double)encoderChange) / 100;
      newKii = newKii + halfstep;
      lastReportedPos = encoderPos;
      if (newKii < 0) {
        newKii = 0.0;
      }
    }
    buttonState = digitalRead(encoderButtonPin);
    if (buttonState != lastButtonState) {
      if (buttonState == HIGH) {
        encoderPos = 0;
        lastReportedPos = 0;
        // reset the menu time out timer
        prevMenuTimeOutMillis = millis();
        EEPROM.put(kiAddress, newKii);
        //      debounce = 2;
        kiChangeVar = !kiChangeVar;
      }
    }
  }
}

void kdChange() {
  intnewKd = (int)newKd;
  while (kdChangeVar == 1) {
    menuTimeOutMillis = millis();
    if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
      prevMenuTimeOutMillis = millis();
      kdChangeVar = !kdChangeVar;
    }
    //   debounce = 8;
    rotating = true;
    lastButtonState = buttonState;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 500) {
      previousMillis = currentMillis;
      blink = !blink;
    }
    if (blink == 0) {
      lcd.setCursor(3, 0);
      lcd.print("Kp: ");
      lcd.print(newKp);
      lcd.print(" ");
      lcd.setCursor(13, 0);
      lcd.print("(");
      lcd.print(Kp);
      lcd.print(") ");
      lcd.setCursor(3, 1);
      lcd.print("Ki: ");
      lcd.print(newKii);
      lcd.setCursor(13, 1);
      lcd.print("(");
      lcd.print(Ki);
      lcd.print(") ");
      lcd.setCursor(0, 2);
      lcd.print("-");
      lcd.write(byte(0));
      lcd.setCursor(3, 2);
      lcd.print("Kd: ");
      lcd.print(newKd);
      lcd.print(" ");
      lcd.setCursor(13, 2);
      lcd.print("(");
      lcd.print(Kd);
      lcd.print(") ");
      lcd.setCursor(3, 3);
      lcd.print("Save and Return");
    }
    if (blink == 1) {
      lcd.setCursor(7, 2);
      lcd.print("     ");
    }
    if (lastReportedPos != encoderPos) {
      blink = 0;
      previousMillis = currentMillis;
      int encoderChange = (encoderPos - lastReportedPos);
      if (encoderChange > 0) {
        encoderChange = 1;
      }
      if (encoderChange < 0) {
        encoderChange = -1;
      }
      intnewKd = (intnewKd + encoderChange);
      lastReportedPos = encoderPos;
      if (intnewKd < 0) {
        intnewKd = 0;
      }
      newKd = (double)intnewKd;
    }
    buttonState = digitalRead(encoderButtonPin);
    if (buttonState != lastButtonState) {
      if (buttonState == HIGH) {
        encoderPos = 0;
        lastReportedPos = 0;
        // reset the menu time out timer
        prevMenuTimeOutMillis = millis();
        EEPROM.put(kdAddress, newKd);
        //    debounce = 2;
        kdChangeVar = !kdChangeVar;
      }
    }
  }
}

void advancedOptionsFun() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;
  lcd.setCursor(2, 0);
  lcd.print("Select Function");
  if (subMenu == 0) {
    lcd.setCursor (0, 2);
    lcd.print("Change PID Settings ");
  }
  if (subMenu == 1) {
    lcd.setCursor (0, 2);
    lcd.print("  Set Cooking Mode  ");
  }

  if (subMenu == 2) {
    lcd.setCursor (0, 2);
    lcd.print("  Keep Warm Temp    ");
  }
  if (subMenu == 3) {
    lcd.setCursor (0, 2);
    lcd.print("   Set Max Temp     ");
  }

  if (subMenu == 4) {
    lcd.setCursor (0, 2);
    lcd.print("  Restore Defaults  ");
  }
  if (subMenu == 5) {
    lcd.setCursor (0, 2);
    lcd.print("       Return       ");
  }
  if (lastReportedPos != encoderPos) {
    int encoderChange = (encoderPos - lastReportedPos);
    if (encoderChange > 0) {
      encoderChange = 1;
    }
    if (encoderChange < 0) {
      encoderChange = -1;
    }

    subMenu = (subMenu + encoderChange);
    lastReportedPos = encoderPos;
    if (subMenu >= 6) {
      subMenu = 0;
    }
    if (subMenu < 0) {
      subMenu = 5;
    }
  }

  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      lcd.clear();
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      setScreen++;
      blink = 1;
    }
  }
}

void keepWarmFun() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;
  // print message that you are in pit temp setpoint mode
  lcd.setCursor(2, 0);
  lcd.print("Set Temp Target");
  lcd.setCursor(1, 1);
  lcd.print("For Warming Mode:");

  if (lastReportedPos != encoderPos) {

    int encoderChange = (encoderPos - lastReportedPos);
    if (encoderChange > 0) {
      encoderChange = 1;
    }
    if (encoderChange < 0) {
      encoderChange = -1;
    }

    keepWarm = (keepWarm + encoderChange);
    lastReportedPos = encoderPos;
    if (keepWarm > keepWarmMax) {
      keepWarm = keepWarmMax;
    }
  }
  lcd.setCursor(8, 3);
  lcd.print(keepWarm);
  lcd.print(" ");
  // check to see if the encoder button has been pressed to enter set mode
  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      EEPROM.put(keepWarmAddress, keepWarm);
      lcd.clear();
      setScreen++;
    }
  }
}

void tempSelectFun() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  deltaSubMenu = 0;
  stepFunction = 0;
  lastButtonState = buttonState;
  // print message that you are in pit temp setpoint mode
  lcd.setCursor(2, 0);
  lcd.print("Set Cooking Mode");

  if (lastReportedPos != encoderPos) {

    int encoderChange = (encoderPos - lastReportedPos);
    if (encoderChange > 0) {
      encoderChange = 1;
    }
    if (encoderChange < 0) {
      encoderChange = -1;
    }

    heatProfile = (heatProfile + encoderChange);
    lastReportedPos = encoderPos;
    if (heatProfile >= 3) {
      heatProfile = 0;
    }
    if (heatProfile < 0) {
      heatProfile = 2;
    }
  }

  if (heatProfile == 0) {
    lcd.setCursor(0, 2);
    lcd.print("       Manual       ");
  }
  if (heatProfile == 1) {
    lcd.setCursor(0, 2);
    lcd.print("  Temp Difference   ");
  }
  if (heatProfile == 2) {
    lcd.setCursor(0, 2);
    lcd.print("      Stepper       ");
  }
  // check to see if the encoder button has been pressed to enter set mode
  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      EEPROM.put(heatProfileAddress, heatProfile);
      lcdReset = 0;
      while ((heatProfile == 1) && (deltaSubMenu == 0)) {
        heatProfileFun();
      }
      while ((heatProfile == 1) && (deltaSubMenu == 1)) {
        heatProfileFun2();
      }
      while ((heatProfile == 2) && (stepFunction == 0)) {
        stepperFun();
      }
      lcd.clear();
      setScreen++;
    }
  }
}

//meat set point function
void meatSetPointFun() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;
  // print message that you are in pit temp setpoint mode
  lcd.setCursor(2, 0);
  lcd.print("Set Temp Target");
  lcd.setCursor(2, 1);
  lcd.print("For Meat Probe:");

  if (lastReportedPos != encoderPos) {

    int encoderChange = (encoderPos - lastReportedPos);
    if (encoderChange > 0) {
      encoderChange = 1;
    }
    if (encoderChange < 0) {
      encoderChange = -1;
    }

    meatSetPoint = (meatSetPoint + encoderChange);
    lastReportedPos = encoderPos;
    if (meatSetPoint > maxTemp) {
      meatSetPoint = maxTemp;
    }
  }

  lcd.setCursor(8, 3);
  lcd.print(meatSetPoint);
  lcd.print(" ");
  // check to see if the encoder button has been pressed to enter set mode
  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      EEPROM.put(meatSetPointAddress, meatSetPoint);
      lcd.clear();
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      setScreen++;
    }
  }
}

void heatProfileFun() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
    deltaSubMenu = 3;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;
  // print message that you are in pit temp setpoint mode
  lcd.setCursor(0, 0);
  lcd.print("  Set Start Temp    ");

  if (lastReportedPos != encoderPos) {

    int encoderChange = (encoderPos - lastReportedPos);
    if (encoderChange > 0) {
      encoderChange = 1;
    }
    if (encoderChange < 0) {
      encoderChange = -1;
    }

    deltaStartTemp = (deltaStartTemp + encoderChange);
    lastReportedPos = encoderPos;
    if (deltaStartTemp > maxTemp) {
      deltaStartTemp = maxTemp;
    }
  }

  lcd.setCursor(0, 2);
  lcd.print("        ");
  lcd.print(deltaStartTemp);
  lcd.print("          ");
  // check to see if the encoder button has been pressed to enter set mode
  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      EEPROM.put(deltaStartTempAddress, deltaStartTemp);
      lcd.clear();
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      deltaSubMenu++;
    }
  }
}

void heatProfileFun2() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
    deltaSubMenu = 3;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;
  // print message that you are in pit temp setpoint mode
  lcd.setCursor(0, 0);
  lcd.print("  Set Temp Delta    ");

  if (lastReportedPos != encoderPos) {

    int encoderChange = (encoderPos - lastReportedPos);
    if (encoderChange > 0) {
      encoderChange = 1;
    }
    if (encoderChange < 0) {
      encoderChange = -1;
    }

    tempDelta = (tempDelta + encoderChange);
    lastReportedPos = encoderPos;
    if (tempDelta > maxTempDelta) {
      tempDelta = maxTempDelta;
    }
    if (tempDelta < 1) {
      tempDelta = 1;
    }
  }

  lcd.setCursor(6, 2);
  lcd.print("  ");
  lcd.print(tempDelta);
  lcd.print(" ");
  // check to see if the encoder button has been pressed to enter set mode
  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      EEPROM.put(tempDeltaAddress, tempDelta);
      lcd.clear();
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      deltaSubMenu++;
    }
  }
}

void stepperFun() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
    setSteps = 1;
    stepFunction = 1;
    stepChange = 1;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;
  setSteps = 0;

  lcd.setCursor(0, 0);
  lcd.print("1 ");
  if (timer1Minutes > 0) {
    lcd.print(timer1Minutes);
    lcd.print(":");
    lcd.print(timer1SetPoint);
  }
  if (timer1Minutes == 0) {
    lcd.print("OFF    ");
  }

  lcd.setCursor(0, 1);
  lcd.print("2 ");
  if (timer2Minutes > 0) {
    lcd.print(timer2Minutes);
    lcd.print(":");
    lcd.print(timer2SetPoint);
  }
  if (timer2Minutes == 0) {
    lcd.print("OFF    ");
  }

  lcd.setCursor(0, 2);
  lcd.print("3 ");
  if (timer3Minutes > 0) {
    lcd.print(timer3Minutes);
    lcd.print(":");
    lcd.print(timer3SetPoint);
  }
  if (timer3Minutes == 0) {
    lcd.print("OFF    ");
  }

  lcd.setCursor(0, 3);
  lcd.print("4 ");
  if (timer4Minutes > 0) {
    lcd.print(timer4Minutes);
    lcd.print(":");
    lcd.print(timer4SetPoint);
  }
  if (timer4Minutes == 0) {
    lcd.print("OFF    ");
  }

  lcd.setCursor(10, 0);
  lcd.print("5 ");
  if (timer5Minutes > 0) {
    lcd.print(timer5Minutes);
    lcd.print(":");
    lcd.print(timer5SetPoint);
  }
  if (timer5Minutes == 0) {
    lcd.print("OFF    ");
  }

  lcd.setCursor(10, 1);
  lcd.print("6 ");
  if (timer6Minutes > 0) {
    lcd.print(timer6Minutes);
    lcd.print(":");
    lcd.print(timer6SetPoint);
  }
  if (timer6Minutes == 0) {
    lcd.print("OFF    ");
  }

  lcd.setCursor (10, 3);
  lcd.print("Final: ");
  lcd.print(finalTemp);

  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      lcdReset = 0;
      stepperMenu = 0;
      while (setSteps == 0) {
        stepperSetFun1();
      }
      lcd.clear();
    }
  }
}

void stepperSetFun1() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
    setSteps = 1;
    stepFunction = 1;
    stepChange = 1;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;

  if (lastReportedPos != encoderPos) {

    stepReturn = !stepReturn;
    lastReportedPos = encoderPos;
  }

  if (stepReturn == 0) {
    lcd.setCursor(0, 1);
    lcd.print("     Set Steps?     ");
  }
  if (stepReturn == 1) {
    lcd.setCursor(0, 1);
    lcd.print("       Return       ");
  }

  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      lcdReset = 0;
      while ((stepReturn == 0) && (stepFunction == 0) && (setSteps == 0)) {
        stepperSetFun();
      }
      if (stepReturn == 1) {
        setScreen = 11;
        setSteps = 1;
        stepFunction = 1;
        stepChange = 1;
        stepReturn = 1;
      }
      lcd.clear();
    }
  }
}

void stepperSetFun() {
  stepChange = 0;
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
    setSteps = 1;
    stepFunction = 1;
    stepChange = 1;
    stepReturn = 1;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;

  if (lastReportedPos != encoderPos) {

    int encoderChange = (encoderPos - lastReportedPos);
    if (encoderChange > 0) {
      encoderChange = 1;
    }
    if (encoderChange < 0) {
      encoderChange = -1;
    }
    stepChange = 0;
    stepperMenu = (stepperMenu + encoderChange);
    lastReportedPos = encoderPos;
    if (stepperMenu > 7) {
      stepperMenu = 0;
    }
    if (stepperMenu < 0) {
      stepperMenu = 7;
    }
  }

  if (stepperMenu == 0) {
    lcd.setCursor(0, 1);
    lcd.print("                    ");
    lcd.setCursor(0, 0);
    lcd.print("       Step 1       ");
    lcd.setCursor(0, 2);
    lcd.print("      Time: ");
    if (timer1Minutes > 0) {
      lcd.print(timer1Minutes);
      lcd.print("   ");
    }
    if (timer1Minutes == 0) {
      lcd.print("OFF");
      lcd.print("   ");
    }
    lcd.setCursor(0, 3);
    lcd.print("      Temp: ");
    if (timer1Minutes > 0) {
      lcd.print(timer1SetPoint);
      lcd.print("   ");
    }
    if (timer1Minutes == 0) {
      lcd.print("OFF");
      lcd.print("   ");
    }
  }
  if (stepperMenu == 1) {
    lcd.setCursor(0, 0);
    lcd.print("       Step 2       ");
    lcd.setCursor(0, 2);
    lcd.print("      Time: ");
    if (timer2Minutes > 0) {
      lcd.print(timer2Minutes);
      lcd.print("   ");
    }
    if (timer2Minutes == 0) {
      lcd.print("OFF");
      lcd.print("   ");
    }
    lcd.setCursor(0, 3);
    lcd.print("      Temp: ");
    if (timer2Minutes > 0) {
      lcd.print(timer2SetPoint);
      lcd.print("   ");
    }
    if (timer2Minutes == 0) {
      lcd.print("OFF");
      lcd.print("   ");
    }
  }
  if (stepperMenu == 2) {
    lcd.setCursor(0, 0);
    lcd.print("       Step 3       ");
    lcd.setCursor(0, 2);
    lcd.print("      Time: ");
    if (timer3Minutes > 0) {
      lcd.print(timer3Minutes);
      lcd.print("   ");
    }
    if (timer3Minutes == 0) {
      lcd.print("OFF");
      lcd.print("   ");
    }
    lcd.setCursor(0, 3);
    lcd.print("      Temp: ");
    if (timer3Minutes > 0) {
      lcd.print(timer3SetPoint);
      lcd.print("   ");
    }
    if (timer3Minutes == 0) {
      lcd.print("OFF");
      lcd.print("   ");
    }
  }
  if (stepperMenu == 3) {
    lcd.setCursor(0, 0);
    lcd.print("       Step 4       ");
    lcd.setCursor(0, 2);
    lcd.print("      Time: ");
    if (timer4Minutes > 0) {
      lcd.print(timer4Minutes);
      lcd.print("   ");
    }
    if (timer4Minutes == 0) {
      lcd.print("OFF");
      lcd.print("   ");
    }
    lcd.setCursor(0, 3);
    lcd.print("      Temp: ");
    if (timer4Minutes > 0) {
      lcd.print(timer4SetPoint);
      lcd.print("   ");
    }
    if (timer4Minutes == 0) {
      lcd.print("OFF");
      lcd.print("   ");
    }
  }
  if (stepperMenu == 4) {
    lcd.setCursor(0, 0);
    lcd.print("       Step 5       ");
    lcd.setCursor(0, 2);
    lcd.print("      Time: ");
    if (timer5Minutes > 0) {
      lcd.print(timer5Minutes);
      lcd.print("   ");
    }
    if (timer5Minutes == 0) {
      lcd.print("OFF");
      lcd.print("   ");
    }
    lcd.setCursor(0, 3);
    lcd.print("      Temp: ");
    if (timer5Minutes > 0) {
      lcd.print(timer5SetPoint);
      lcd.print("   ");
    }
    if (timer5Minutes == 0) {
      lcd.print("OFF");
      lcd.print("   ");
    }
  }
  if (stepperMenu == 5) {
    lcd.setCursor(0, 1);
    lcd.print("                    ");
    lcd.setCursor(0, 0);
    lcd.print("       Step 6       ");
    lcd.setCursor(0, 2);
    lcd.print("      Time: ");
    if (timer6Minutes > 0) {
      lcd.print(timer6Minutes);
      lcd.print("   ");
    }
    if (timer6Minutes == 0) {
      lcd.print("OFF");
      lcd.print("   ");
    }
    lcd.setCursor(0, 3);
    lcd.print("      Temp: ");
    if (timer6Minutes > 0) {
      lcd.print(timer6SetPoint);
      lcd.print("   ");
    }
    if (timer6Minutes == 0) {
      lcd.print("OFF");
      lcd.print("   ");
    }
  }
  if (stepperMenu == 6) {
    lcd.setCursor(0, 0);
    lcd.print("                    ");
    lcd.setCursor(0, 1);
    lcd.print("     Final Temp:     ");
    lcd.setCursor(0, 2);
    lcd.print("         ");
    lcd.print(finalTemp);
    lcd.print("    ");
    lcd.setCursor(0, 3);
    lcd.print("                    ");
  }
  if (stepperMenu == 7) {
    lcd.setCursor(0, 0);
    lcd.print("                    ");
    lcd.setCursor(0, 2);
    lcd.print("                    ");
    lcd.setCursor(0, 3);
    lcd.print("                    ");
    lcd.setCursor(0, 1);
    lcd.print("       Return       ");
  }
  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      lcdReset = 0;
      minTemp = 0;
      while ((stepperMenu >= 0) && (stepperMenu < 6) && (stepChange == 0)) {
        stepChangeFun();
      }
      minTemp = 0;

      while ((stepperMenu == 6) && (stepChange == 0)) {
        finalChangeFun();
      }
      if (stepperMenu == 7) {
        setSteps = 1;
        //       stepFunction = 1;
      }
      lcd.clear();
    }
  }
}

void stepChangeFun() {
  stepChange = 0;
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
    setSteps = 1;
    stepFunction = 1;
    stepChange = 1;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;
  if (minTemp == 0) {
    lcd.setCursor(5, 1);
    lcd.print("-");
    lcd.write(byte(0));
  }

  if (stepperMenu == 0) {
    lcd.setCursor(0, 0);
    lcd.print("  Step 1 Minutes:   ");
    lcd.setCursor(8, 1);
    lcd.print(timer1Minutes);
    lcd.print("   ");
    lcd.setCursor(0, 2);
    lcd.print("       Temp:        ");
    lcd.setCursor(8, 3);
    lcd.print(timer1SetPoint);
    lcd.print("   ");
  }
  if (stepperMenu == 1) {
    lcd.setCursor(0, 0);
    lcd.print("  Step 2 Minutes:   ");
    lcd.setCursor(8, 1);
    lcd.print(timer2Minutes);
    lcd.print("   ");
    lcd.setCursor(0, 2);
    lcd.print("       Temp:        ");
    lcd.setCursor(8, 3);
    lcd.print(timer2SetPoint);
    lcd.print("   ");
  }
  if (stepperMenu == 2) {
    lcd.setCursor(0, 0);
    lcd.print("  Step 3 Minutes:   ");
    lcd.setCursor(8, 1);
    lcd.print(timer3Minutes);
    lcd.print("   ");
    lcd.setCursor(0, 2);
    lcd.print("       Temp:        ");
    lcd.setCursor(8, 3);
    lcd.print(timer3SetPoint);
    lcd.print("   ");
  }
  if (stepperMenu == 3) {
    lcd.setCursor(0, 0);
    lcd.print("  Step 4 Minutes:   ");
    lcd.setCursor(8, 1);
    lcd.print(timer4Minutes);
    lcd.print("   ");
    lcd.setCursor(0, 2);
    lcd.print("       Temp:        ");
    lcd.setCursor(8, 3);
    lcd.print(timer4SetPoint);
    lcd.print("   ");
  }
  if (stepperMenu == 4) {
    lcd.setCursor(0, 0);
    lcd.print("  Step 5 Minutes:   ");
    lcd.setCursor(8, 1);
    lcd.print(timer5Minutes);
    lcd.print("   ");
    lcd.setCursor(0, 2);
    lcd.print("       Temp:        ");
    lcd.setCursor(8, 3);
    lcd.print(timer5SetPoint);
    lcd.print("   ");
  }
  if (stepperMenu == 5) {
    lcd.setCursor(0, 0);
    lcd.print("  Step 6 Minutes:   ");
    lcd.setCursor(8, 1);
    lcd.print(timer6Minutes);
    lcd.print("   ");
    lcd.setCursor(0, 2);
    lcd.print("       Temp:        ");
    lcd.setCursor(8, 3);
    lcd.print(timer6SetPoint);
    lcd.print("   ");
  }

  if (lastReportedPos != encoderPos) {

    int encoderChange = (encoderPos - lastReportedPos);
    if (encoderChange > 0) {
      encoderChange = 1;
    }
    if (encoderChange < 0) {
      encoderChange = -1;
    }
    if (minTemp == 0) {
      if (stepperMenu == 0) {
        timer1Minutes = (timer1Minutes + encoderChange);
        if (timer1Minutes <= 0) {
          timer1Minutes = 0;
        }
      }
      if (stepperMenu == 1) {
        timer2Minutes = (timer2Minutes + encoderChange);
        if (timer2Minutes <= 0) {
          timer2Minutes = 0;
        }
      }
      if (stepperMenu == 2) {
        timer3Minutes = (timer3Minutes + encoderChange);
        if (timer3Minutes <= 0) {
          timer3Minutes = 0;
        }
      }
      if (stepperMenu == 3) {
        timer4Minutes = (timer4Minutes + encoderChange);
        if (timer4Minutes <= 0) {
          timer4Minutes = 0;
        }
      }
      if (stepperMenu == 4) {
        timer5Minutes = (timer5Minutes + encoderChange);
        if (timer5Minutes <= 0) {
          timer5Minutes = 0;
        }
      }
      if (stepperMenu == 5) {
        timer6Minutes = (timer6Minutes + encoderChange);
        if (timer6Minutes <= 0) {
          timer6Minutes = 0;
        }
      }
    }
    if (minTemp == 1) {
      if (stepperMenu == 0) {
        timer1SetPoint = (timer1SetPoint + encoderChange);
        if (timer1SetPoint > maxTemp) {
          timer1SetPoint = maxTemp;
        }
        if (timer1SetPoint < 0) {
          timer1SetPoint = 0;
        }
      }
      if (stepperMenu == 1) {
        timer2SetPoint = (timer2SetPoint + encoderChange);
        if (timer2SetPoint > maxTemp) {
          timer2SetPoint = maxTemp;
        }
        if (timer2SetPoint < 0) {
          timer2SetPoint = 0;
        }
      }
      if (stepperMenu == 2) {
        timer3SetPoint = (timer3SetPoint + encoderChange);
        if (timer3SetPoint > maxTemp) {
          timer3SetPoint = maxTemp;
        }
        if (timer3SetPoint < 0) {
          timer3SetPoint = 0;
        }
      }
      if (stepperMenu == 3) {
        timer4SetPoint = (timer4SetPoint + encoderChange);
        if (timer4SetPoint > maxTemp) {
          timer4SetPoint = maxTemp;
        }
        if (timer4SetPoint < 0) {
          timer4SetPoint = 0;
        }
      }
      if (stepperMenu == 4) {
        timer5SetPoint = (timer5SetPoint + encoderChange);
        if (timer5SetPoint > maxTemp) {
          timer5SetPoint = maxTemp;
        }
        if (timer5SetPoint < 0) {
          timer5SetPoint = 0;
        }
      }
      if (stepperMenu == 5) {
        timer6SetPoint = (timer6SetPoint + encoderChange);
        if (timer6SetPoint > maxTemp) {
          timer6SetPoint = maxTemp;
        }
        if (timer6SetPoint < 0) {
          timer6SetPoint = 0;
        }
      }
    }
    lastReportedPos = encoderPos;
  }

  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      if (minTemp == 0) {
        if (stepperMenu == 0) {
          EEPROM.put(timer1MinutesAddress, timer1Minutes);
        }
        if (stepperMenu == 1) {
          EEPROM.put(timer2MinutesAddress, timer2Minutes);
        }
        if (stepperMenu == 2) {
          EEPROM.put(timer3MinutesAddress, timer3Minutes);
        }
        if (stepperMenu == 3) {
          EEPROM.put(timer4MinutesAddress, timer4Minutes);
        }
        if (stepperMenu == 4) {
          EEPROM.put(timer5MinutesAddress, timer5Minutes);
        }
        if (stepperMenu == 5) {
          EEPROM.put(timer6MinutesAddress, timer6Minutes);
        }
        lcd.setCursor(5, 1);
        lcd.print("  ");
        lcd.setCursor(5, 3);
        lcd.print("-");
        lcd.write(byte(0));
      }
      if (minTemp == 1) {
        if (stepperMenu == 0) {
          EEPROM.put(timer1SetPointAddress, timer1SetPoint);
        }
        if (stepperMenu == 1) {
          EEPROM.put(timer2SetPointAddress, timer2SetPoint);
        }
        if (stepperMenu == 2) {
          EEPROM.put(timer3SetPointAddress, timer3SetPoint);
        }
        if (stepperMenu == 3) {
          EEPROM.put(timer4SetPointAddress, timer4SetPoint);
        }
        if (stepperMenu == 4) {
          EEPROM.put(timer5SetPointAddress, timer5SetPoint);
        }
        if (stepperMenu == 5) {
          EEPROM.put(timer6SetPointAddress, timer6SetPoint);
        }
        encoderPos = 0;
        lastReportedPos = 0;
        // reset the menu time out timer
        prevMenuTimeOutMillis = millis();
        lcdReset = 0;
        minTemp = 0;
        stepChange = 1;
      }
      minTemp = 1;
    }
  }
}


void finalChangeFun() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
    setSteps = 1;
    stepFunction = 1;
    stepChange = 1;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;
  // print message that you are in pit temp setpoint mode
  lcd.setCursor(3, 1);
  lcd.print("Set Final Temp");

  if (lastReportedPos != encoderPos) {

    int encoderChange = (encoderPos - lastReportedPos);
    if (encoderChange > 0) {
      encoderChange = 1;
    }
    if (encoderChange < 0) {
      encoderChange = -1;
    }

    finalTemp = (finalTemp + encoderChange);
    lastReportedPos = encoderPos;
    if (finalTemp > maxTemp) {
      finalTemp = maxTemp;
    }
  }
  lcd.setCursor(6, 2);
  lcd.print("-");
  lcd.write(byte(0));
  lcd.setCursor(9, 2);
  lcd.print(finalTemp);
  lcd.print(" ");
  // check to see if the encoder button has been pressed to enter set mode
  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      EEPROM.put(finalTempAddress, finalTemp);
      lcd.clear();
      stepChange = 1;
    }
  }
}

void maxTempChangeFun() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;
  // print message that you are in pit temp setpoint mode
  lcd.setCursor(4, 0);
  lcd.print("Set Max Temp");

  if (lastReportedPos != encoderPos) {

    int encoderChange = (encoderPos - lastReportedPos);
    if (encoderChange > 0) {
      encoderChange = 1;
    }
    if (encoderChange < 0) {
      encoderChange = -1;
    }

    maxTemp = (maxTemp + encoderChange);
    lastReportedPos = encoderPos;
    if (maxTemp > maxTempMax) {
      maxTemp = maxTempMax;
    }
  }
  lcd.setCursor(8, 2);
  lcd.print(maxTemp);
  lcd.print(" ");
  // check to see if the encoder button has been pressed to enter set mode
  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      EEPROM.put(maxTempAddress, maxTemp);
      lcd.clear();
      setScreen++;
    }
  }
}

void defaultsFun() {
  menuTimeOutMillis = millis();
  if ((menuTimeOutMillis - prevMenuTimeOutMillis) >= menuTimeOutInterval) {
    setScreen = 11;
    setSteps = 1;
    stepFunction = 1;
    stepChange = 1;
  }
  rotating = true;
  digitalWrite (relay, LOW);
  if (lcdReset == 0) {
    lcd.clear();
  }
  lcdReset++;
  lastButtonState = buttonState;

  if (lastReportedPos != encoderPos) {

    defaultsReset = !defaultsReset;
    lastReportedPos = encoderPos;
  }

  if (defaultsReset == 0) {
    lcd.setCursor(0, 1);
    lcd.print("  Reset Defaults?  ");
  }
  if (defaultsReset == 1) {
    lcd.setCursor(0, 1);
    lcd.print("       Return      ");
  }

  buttonState = digitalRead(encoderButtonPin);
  if (buttonState != lastButtonState)  {
    if (buttonState == HIGH) {
      encoderPos = 0;
      lastReportedPos = 0;
      // reset the menu time out timer
      prevMenuTimeOutMillis = millis();
      lcdReset = 0;
      while ((defaultsReset == 0) && (stepFunction == 0) && (setSteps == 0)) {
        defaultsResetFun();
      }
      if (defaultsReset == 1) {
        setScreen = 11;
        setSteps = 1;
        stepFunction = 1;
        stepChange = 1;
        stepReturn = 1;
      }
      lcd.clear();
    }
  }
}

void defaultsResetFun() {
  EEPROM.put(kpAddress, 30.00);
  EEPROM.put(kiAddress, 0.30);
  EEPROM.put(kdAddress, 8.00);
  EEPROM.put(setPointAddress, 165);
  EEPROM.put(keepWarmAddress, 80);
  EEPROM.put(heatProfileAddress, 0);
  EEPROM.put(meatSetPointAddress, 150);
  EEPROM.put(deltaStartTempAddress, 100);
  EEPROM.put(tempDeltaAddress, 30);
  EEPROM.put(timer1MinutesAddress, 0);
  EEPROM.put(timer1SetPointAddress, 120);
  EEPROM.put(timer2MinutesAddress, 0);
  EEPROM.put(timer2SetPointAddress, 120);
  EEPROM.put(timer3MinutesAddress, 0);
  EEPROM.put(timer3SetPointAddress, 120);
  EEPROM.put(timer4MinutesAddress, 0);
  EEPROM.put(timer4SetPointAddress, 120);
  EEPROM.put(timer5MinutesAddress, 0);
  EEPROM.put(timer5SetPointAddress, 120);
  EEPROM.put(timer6MinutesAddress, 0);
  EEPROM.put(timer6SetPointAddress, 120);
  EEPROM.put(finalTempAddress, 165);
  EEPROM.put(maxTempAddress, 180);
  lcd.clear();
  setScreen = 11;
  setSteps = 1;
  stepFunction = 1;
  stepChange = 1;
  stepReturn = 1;
}

