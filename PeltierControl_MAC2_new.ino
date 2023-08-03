/*
  PID control of Peltier cell either in cooling or heating
  Temperature is in Kelvin
  Peltier Module UEPT 340-228-060C200S: Vmax=18 V; Imax=6 A; Tmax=170 °C; DeltaTmax=111 K. Suggested limits for P-cell: 250 to 410 K
  POWER SUPPLY: 15-18 V -- works fine with 15 V;
  Limit current to 3.6 A to avoid over heating (if more add one fan on the transitors)
========================================
  CONNECT POWER SUPPLY (-) TO GROUND !!
========================================
  Only use with short thermocouple cable or the temperature becomes very noisy and calibration is no longer valid!
  USE (from Python/Matlab):
  (1) enable serial communication (choose the right port!)
  (2) read temperature with "READ\n"
  (3) print setpoint with "SETPOINT "+Value (where value is a double, e.g. 300.0)
*/

#include <SPI.h>
#include "Adafruit_MAX31855.h"  // thermocouple library
#include <PID_v1.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// PWM RC-circuit to IRLZ14 nMOS input
int T_heating = 10;  // pin connected to RC circuit#1 and IRLZ14#1-gate to control Temperature output (PWM) - MOS1-cooling
int T_cooling = 9; // pin connected to RC circuit#2 and IRLZ14#2-gate to control Temperature output (PWM) - MOS2-heating
int Tcontrol;       // it can be either T_heating or T_cooling depending on 'mode'.
//int analogRCpin = A0;              // input test pin (read after RC circuit)

String state;

// Temperature reading stuff (Thermocouple: MAX31855):
int maxSO =  5;  // Slave Output (input)
int maxCS = 6;   // chip select (output)
int maxSCK = 12; // serial clock (output)
Adafruit_MAX31855 kTC(maxSCK, maxCS, maxSO);
double temperature_tmp, T;        // current temperature readings
double Told;                      // Temperautre at the previous cycle
int MAX_TEMP_READ_ATTEMPT = 30; // max attemped temperature readings (K)
double MAX_ALLOWED_TEMP = 443;    // safety limit (K) for UEPT 340-228-060C200S
int nTreads=1; // number of Temperature readings: T = sum(T_i)/nTreads

// Arduino stuff:
int LoopDelay = 100; // The RC circuit needs at least 90 to equilibrate! 5/2Tau~100 ms
unsigned long time;               // to be used like time=millis();
unsigned i = 0;                   // generic counter
unsigned long last_read_time = 0;
unsigned long start = 0;
String cmd = "";                  //init command for serial communication

// DC Relays stuff (H-bridge):
int R3 = 2;       // Arduino pins for realys R3: (connected to Power Supply -)
int R4 = 4;       // Arduino pins for realys R4: (connected to Power Supply +)
int R2 = 8;       // Arduino pins for realys R2: cooling circuit (connected between Peltier - and nMOS-right)
int R1 = 7;       // Arduino pins for realys R1: heating circuit (connected between Peltier + and nMOS-left)

bool mode = true; //TRUE="heating", FALSE="cooling"

double T_treshold_min = 286; //Switching temperature (K). Used to switch relays and to limit power when heating from Low-T
double T_treshold_max = 310; //Max treshold temperature (K), used to limit power when cooling from High-T
double T_treshold = 288; // used as single treshold when T_treshold_min and T_treshold_max are not employed

// PID stuff:
double Setpoint, Input, Output, power;
int outputMax=225; // limit the maximum power within the MOS linear region
int outputMin=80; // limit the minimum power above the MOS open treshold
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
  closeGates();
  
  pinMode(maxSO, INPUT);
  pinMode(maxCS, OUTPUT);
  pinMode(maxSCK, OUTPUT);
  
  //start in HEAT mode (mode=true) with all Realys OFF (HIGH state):
  HEAT();//digitalWrite(R1, HIGH); digitalWrite(R2, HIGH); digitalWrite(R3, HIGH);  digitalWrite(R4, HIGH); // use pin~10 (left circuit) / COOLING

  //(1) For safety, remove current from any MOSFET:
  closeGates();
  myPID.SetOutputLimits(outputMin, outputMax); // limit gate voltage on nMOSFET. For IRLZ14, current starts flowing above 1.5 V (85 in pwm) and reaches saturation at about 4 V (around 210 in pwm)
  
  mode = true;
  state = "Setup";
  //(2) Check current Temperature when Arduino is started:
  temperature_tmp = readT();
  last_read_time = millis();

  //(3) Fix setpoint to current temperature when Arduino is connected (or in case of reset)
  T = temperature_tmp;
  Told = T;
  Input = T;
  //Setpoint = 350; // Kelvin  <----comment when Arduino is driven remotely (Python/Matlab)!
  Setpoint = T; // stay where you are! <----comment only for tests

  //(4) Decide whether we need to cool or heat and run PID routine to keep temperature constant:
  
  /*// 3a) high temperature regime (When coming from T>T_treshold_min and need to )*/ 
  if (Setpoint >= T_treshold) {
    if (Setpoint > MAX_ALLOWED_TEMP){
      Setpoint = MAX_ALLOWED_TEMP;
      }
    HEAT(); 
    myPID.Compute();
    power = Output;
    analogWrite(Tcontrol, power);
   }
  /*// 3b) low temperature regime:*/
  else if (Setpoint < T_treshold) {
    COOL();     
    myPID.Compute();
    power = -(Output);    
    analogWrite(Tcontrol, power);
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

//===================================================================================
//---------ARDUINO LOOP--------------------------------------------------------------
//===================================================================================
void loop() {
  //(1) Read Temperature from thermocouple:
  temperature_tmp = readT();
  last_read_time = millis();
  T = temperature_tmp;

  if (isnan(T)) {
    T = Told;
  }

  /* b) print Temperature on LCD display*/
  lcd.clear();
  //lcd.blink();
  lcd.setCursor(2,0); lcd.print("T = ");
  lcd.setCursor(6,0); lcd.print(String(T, 0));
  lcd.setCursor(11,0); lcd.print("K");

  /* c) print Setpoint on LCD display*/
  lcd.setCursor(0,1); lcd.print("Setpoint = " + String(Setpoint, 0) + " K");


  //[!]Remove following line when used remotely[!]:
  //Serial.print("T "); Serial.print(T); Serial.print("\t");  Serial.print("Tset "); Serial.print(Setpoint); Serial.print("\t"); Serial.print("pwr "); Serial.print(Output);Serial.print("\n");
  

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
        //for errors...
        Serial.println("Command not valid: " + cmd);
      }
    } // Close check serial input
  } // Close check serial connection

  //(3) Decide whether we need to cool or heat and run PID routine:
  /*// 3a) high temperature regime:*/
  if (Setpoint >= T_treshold) {
    if (Setpoint > MAX_ALLOWED_TEMP){
      Setpoint = MAX_ALLOWED_TEMP;
      }
    HEAT(); 
    myPID.Compute();    
    power = Output; 
    analogWrite(Tcontrol, power);
   }
   
  /*// 3b) low temperature regime:*/
  else if (Setpoint < T_treshold) {
    COOL(); 
    myPID.Compute();    
    power = -(Output); 
    analogWrite(Tcontrol, power);    
   }

  // save current temperature for following loop and restart cycle
  Told = T;
  delay(LoopDelay);

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
  while ( (isnan(TT) or TT <= 0) and i < MAX_TEMP_READ_ATTEMPT) {
      while (k < nTreads) {
        tempT = (kTC.readCelsius());
        delay(100);
        TT = TT + tempT;
        k++;
      }
      i++;  
  }  
  result = TT / nTreads;
  //}
  
  return result + 273.15;
}


void HEAT() {
  if (!mode) {
    //close gates
    digitalWrite(T_cooling, LOW);  
    digitalWrite(T_heating, LOW);  
    //switch circuits
    pinMode(R1, OUTPUT); pinMode(R2, OUTPUT); pinMode(R3, OUTPUT); pinMode(R4, OUTPUT);
    digitalWrite(R1, HIGH); digitalWrite(R2, HIGH); 
    digitalWrite(R3, HIGH); digitalWrite(R4, HIGH); 
    delay(2);
    myPID.SetOutputLimits(outputMin, outputMax); 
  }
  Tcontrol = T_heating;
  mode = true; //heating mode
}

void COOL() {
  if (mode) {
    //close gates
    digitalWrite(T_heating, LOW); 
    digitalWrite(T_cooling, LOW); 
    //switch circuits    
    pinMode(R1, OUTPUT); pinMode(R2, OUTPUT); pinMode(R3, OUTPUT); pinMode(R4, OUTPUT);
    digitalWrite(R1, LOW); digitalWrite(R2, LOW);
    digitalWrite(R3, LOW); digitalWrite(R4, LOW); 
    delay(2);
    myPID.SetOutputLimits(-outputMax, -outputMin);     
  }
  Tcontrol = T_cooling;
  mode = false; //cooling mode
}

void closeGates() {
  digitalWrite(T_heating, LOW); delay(2);
  digitalWrite(T_cooling, LOW); delay(2);
  delay(100);
}
