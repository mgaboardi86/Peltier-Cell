/*
  PID control of Peltier cell either in cooling or heating
  Temperature is in Kelvin
  SUGGESTED limits: 250 K to 410 K (for the Peltier cell)
  POWER SUPPLY: 12-24 V (depending on the Peltier) -- works fine with 15 V, 3.6 A
  Limit current to 3.6 A to avoid over heating (if more add one fan on the transitors)

  Only use with short thermocouple cable or the temperature becomes very noisy and calibration is no longer valid!

  USE (from Python/Matlab):
  (1) enable serial communication
  (2) read temperature with "READ\n"
  (3) print setpoint with "SETPOINT "+Value (where value is a double, e.g. 300.0)
*/

#include <SPI.h>
#include "Adafruit_MAX31855.h"  // thermocouple library
#include <PID_v1.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


// IRF520 MOSFETS:
int T_heating = 9;  // pin connected to IRF520 gate to control Temperature output (PWM) - MOS1
int T_cooling = 10; // pin connected to IRF520 gate to control Temperature output (PWM) - MOS2
int Tcontrol;       // it can be either T_heating or T_cooling depending on 'mode'.
String state;

// Temperature reading stuff (Thermocouple: MAX31855):
int maxSO =  5;  // Slave Output (input)
int maxCS = 6;   // chip select (output)
int maxSCK = 12; // serial clock (output)

Adafruit_MAX31855 kTC(maxSCK, maxCS, maxSO);
double temperature_tmp, T;        // current temperature readings
double Told;                      // Temperautre at the previous cycle
int MAX_TEMP_READ_ATTEMPT = 30; // max attemped temperature readings (K)
double MAX_ALLOWED_TEMP = 400;    // (K)

// Arduino stuff:
int LoopDelay = 200;
unsigned long time;               // to be used like time=millis();
unsigned i = 0;                   // generic counter
unsigned long last_read_time = 0;
unsigned long start = 0;
String cmd = "";                  //init command for serial communication

// DC Relays stuff (H-bridge):
int R1 = 2;       // Arduino pins for realys R1: heating circuit (connected to MOS1 source)
int R3 = 7;       // Arduino pins for realys R3: heating circuit (connected to Peltier +)
int R2 = 4;       // Arduino pins for realys R2: cooling circuit (connected to MOS2 source)
int R4 = 8;       // Arduino pins for realys R4: cooling circuit (connected to Peltier -)
bool mode = true; //TRUE="heating", FALSE="cooling"

double T_treshold_min = 295; //Switching temperature (K). Used to switch relays and to limit power when heating from Low-T
double T_treshold_max = 310; //Max treshold temperature (K), used to limit power when cooling from High-T

// PID stuff:
double Setpoint, Input, Output, power;
//double Kp = 10, Ki = 0.4, Kd = 0.6;   // PID parameters (backup)
//double Kp = 10 , Ki = 0.4, Kd = 0.8;     // PID parameters (TUNED old Peltier)
double Kp = 10 , Ki = 0.6, Kd = 1.0;     // PID parameters (new Peltier)
bool pid_ena = true;                  // PID is active
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// LCD display stuff
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

//----------ARDUINO SETUP-----------------------------------------------------------------------------------------------------------
void setup() {
  // enable communication:
  Serial.begin(9600); delay(250);
  pinMode(T_heating, OUTPUT);  // define Temperature control pin as output
  pinMode(T_cooling, OUTPUT);  // define Temperature control pin as output
  pinMode(R1, OUTPUT);// sent i(th) pin as output
  pinMode(R2, OUTPUT);// sent i(th) pin as output
  pinMode(R3, OUTPUT);// sent i(th) pin as output
  pinMode(R4, OUTPUT);// sent i(th) pin as output
  pinMode(maxSO, INPUT);
  pinMode(maxCS, OUTPUT);
  pinMode(maxSCK, OUTPUT);
  //(1) For safety, remove current from any MOSFET:
  digitalWrite(T_cooling, LOW);  digitalWrite(T_heating, LOW);
  
  //digitalWrite(R2, HIGH); digitalWrite(R4, HIGH); // channel 1 open
  //digitalWrite(R3, HIGH); digitalWrite(R1, HIGH); // channel 2 closed
  //delay(100);
  mode = true;
  state = "Setup";
  //(2) Check current Temperature when Arduino is started:
  temperature_tmp = readT();
  last_read_time = millis();

  //(3) Fix setpoint to current temperature when Arduino is connected (or in case of reset)
  T = temperature_tmp;
  Told = T;
  Input = T;
  //Setpoint = 300; // Kelvin  <----remove when Arduino is driven remotely (Python/Matlab)!
  Setpoint = T; // stay where you are! <----remove for tests

  //(4) Decide whether we need to cool or to heat and run PID routine to keep temperature constant:
  /*// 3a) high temperature regime:*/
  if (Setpoint >= T_treshold_min) {
    HEAT(); Tcontrol = T_heating; myPID.Compute();
    if (T >= T_treshold_min) {
      power = Output; state = "3a-1";
    }
    else if (T < T_treshold_min) {
      power = Output; state = "3a-2";//**
    }
    analogWrite(Tcontrol, power);
  }
  /*// 3b) low temperature regime:*/
  else if (Setpoint < T_treshold_min) {
    COOL(); Tcontrol = T_cooling; myPID.Compute();
    if (T >= T_treshold_max) {
      power = (255 - Output); state = "3b-1"; //**
    }
    else if (T < T_treshold_max) {
      power = (255 - Output); state = "3b-2";
    }
    analogWrite(Tcontrol, power);
  }
  else {    // some error occurred
  }
  
  // (5) initialize PID
  start = millis();         // start counting time when Arduino is connected
  last_read_time =  start;
  myPID.SetMode(AUTOMATIC); // don't worry, be happy
  pid_ena = true;           // set PID state: enabled


  // (6) enable display
  lcd.begin();  //initialize the lcd
  //lcd.backlight();  //open the backlight
}



//----------------------------------------------------------------------------------------------------------------------------------
//---------ARDUINO LOOP-------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
void loop() {


  //(1) Read Temperature from thermocouple:
  /* a) temporarly set Output to zero to remove errors from T reading (this is cheating):*/
  //analogWrite(Tcontrol, 0);
  temperature_tmp = readT();
  last_read_time = millis();
  T = temperature_tmp;

  if (isnan(T)) {
    T = Told;
  }

  /* b) print Temperature on LCD display*/
  lcd.clear();
  //lcd.blink();
  lcd.setCursor(2, 0); lcd.print("T = ");
  lcd.setCursor(6, 0); lcd.print(String(T, 0));
  lcd.setCursor(11, 0); lcd.print("K");

  /* c) print Setpoint on LCD display*/
  lcd.setCursor(0, 1); lcd.print("Setpoint = " + String(Setpoint, 0) + " K");

  //!Remove following lines when used remotely! <--------
  //Serial.println(T, 2);
  //Serial.println(String(T) + " K,   Setpoint = " + String(Setpoint) +  " K,   state = " + state + ",    Mode  = " + mode + ", Output = " + String(power));
  //Serial.println(String(T) + " K,   Setpoint = " + String(Setpoint) +  " K");
  //Serial.print(" ");

  Input = T;

  //(2) Check for Serial Communication Input
  // Check serial connection: ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~
  if (Serial)
  {
    // Check serial input
    if (Serial.available() > 0)
    {
      cmd = Serial.readStringUntil('\n');
      // Manual mode ON
      if (cmd == "ON") {
        pid_ena = false;
      }
      else if (cmd == "READ") {
        Serial.println(T);
      }
      // Send new setpoint to Arduino
      else if (cmd.substring(0, 8) == "SETPOINT") {
        Setpoint = cmd.substring(9).toDouble();
        if (Setpoint > MAX_ALLOWED_TEMP) {
          Setpoint = MAX_ALLOWED_TEMP;
        }
      }
      // Read Setpoint
      else if (cmd == "READSETPOINT") {
        Serial.println(Setpoint);
      }
      else {
        //TODO (for errors...)
        Serial.println("Command not valid: " + cmd);
      }
    } // Close check serial input
  } // Close check serial connection

  //(3) Decide whether we need to cool or to heat and run PID routine:
  /*// 3a) high temperature regime:*/
  if (Setpoint >= T_treshold_min) {
    HEAT(); Tcontrol = T_heating; myPID.Compute();
    if (T >= T_treshold_min) {
      power = Output; state = "3a-1";
      analogWrite(Tcontrol, power);
    }
    else if (T < T_treshold_min) {
      power = Output; state = "3a-2";//**
      analogWrite(Tcontrol, power);
    }
    else {
      state = "error1";
      analogWrite(Tcontrol, 10);
    }
  }
  /*// 3b) low temperature regime:*/
  else if (Setpoint < T_treshold_min) {
    COOL(); Tcontrol = T_cooling; myPID.Compute();
    if (T >= T_treshold_max) {
      power = (255 - Output); state = "3b-1"; //**
      analogWrite(Tcontrol, power);
    }
    else if (T < T_treshold_max) {
      power = (255 - Output); state = "3b-2";
      analogWrite(Tcontrol, power);
    }
  }
  else {    // some error occurred
    state = "error2";
    analogWrite(Tcontrol, 10);
  }

  // save current temperature for following loop and restart cycle
  Told = T;
  delay(LoopDelay);

  //!Remove following lines when used remotely! <--------
  //Serial.println(T, 2);
  //Serial.println(String(T) + " K,   Setpoint = " + String(Setpoint) +  " K,   state = " + state + ",    Mode  = " + mode + ", Output = " + String(power));


} // Close Arduino Loop




// ----Functions-------------------------------------------------------------------------------------------------------------------
/*double readT() {
  double result, tempT;
  int i = 0;
  tempT = 0;
  delay(200);
  while ((isnan(tempT) or tempT == 0 or tempT == 273.15) and i < MAX_TEMP_READ_ATTEMPT) {
    tempT = (kTC.readCelsius() + 273.15);
    delay(100);
    i++;
  }
  result = tempT;
  return result;
  }*/

double readT() {
  double result, tempT, TT;
  int i = 0;
  int k = 0;
  tempT = 0;
  TT = 0;

  while ( (isnan(TT) or TT == 0) and i < MAX_TEMP_READ_ATTEMPT) {
    while (k < 3) {
      tempT = (kTC.readCelsius() + 273.15);
      delay(50);
      TT = TT + tempT;
      k++;
    }
    i++;
  }

  result = TT / 3;
  return result;
}


void HEAT() {
  if (!mode) {
    digitalWrite(T_cooling, LOW);  delay(300);
    digitalWrite(T_heating, LOW);  delay(300);
    digitalWrite(R2, HIGH); digitalWrite(R4, HIGH); // channel 1 open
    digitalWrite(R3, HIGH); digitalWrite(R1, HIGH); // channel 2 closed
    delay(200);
  }
  Tcontrol = T_heating;
  mode = true; //heating mode
}

void COOL() {
  if (mode) {
    digitalWrite(T_heating, LOW); delay(300);
    digitalWrite(T_cooling, LOW); delay(300);
    digitalWrite(R1, LOW); digitalWrite(R3, LOW);   // channel 1 closed
    digitalWrite(R4, LOW); digitalWrite(R2, LOW);   // channel 2 open
    delay(200);
  }
  Tcontrol = T_cooling;
  mode = false; //cooling mode
}

void ReleaseAllCurrent() {
  digitalWrite(T_heating, LOW);
  digitalWrite(T_cooling, LOW);
  delay(500);
}
