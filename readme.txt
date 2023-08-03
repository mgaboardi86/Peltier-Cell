install PeltierControl_Final on the Arduino 

remote control via USB communication 

/*
 PID control of Peltier cell either in cooling or heating
 Temperature is in Kelvin
 SUGGESTED limits: 250 K to 410 K (depending on the Peltier)
 POWER SUPPLY: 12 V (depending on the Peltier)
 Limit current to 2.5 A to avoid over heating

 USE (from Python/Matlab):
 (1) enable serial communication
 (2) read temperature with "READ\n"
 (3) print setpoint with "SETPOINT"+Value (where value is a double, e.g. 300.0)
 (4) read setpoint with "READSETPOINT"
*/