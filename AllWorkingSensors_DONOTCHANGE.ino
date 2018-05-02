/*
Author:   M. Luetzelberger
License:  GPL-3.0
Purpose:  Arduino Sketch for controlling multiple VL6180x TOF sensors
Version:  1.0
Date:     2017-02-14
Depends:  Wire.h
Changes:
Notes:
For Arduino UNO & Pro Mini use
SDA: A4
SCL: A5
WARNING: Some sensors require 2.8 V VCC. Connect a step-down regulator e.g. LM317.
For VOUT 2.8 V use R1 of 220 Ohm and R2 220 Ohm + 47 Ohm, respectively.
Also, a level shifter must be used for sensors driven with 2.8 V VCC!
In addition to SDA/SCL pins, most sensord have IO0 and IO1 pins. IO0 the enable pin.
After reset, an arbitrary I2C address can be configured, which differs from the hard-coded
I2C of the sensor. Thus, it is possible to use multiple sensors without changeing the
hard-coded address of the module.
*/

#include <Wire.h>
#include <VL6180X.h>
#include <MPU9250.h>

#define RANGE 1

/* List of adresses for each sensor - after reset the address can be configured */
#define address0 0x22
#define address1 0x20
#define address2 0x24

/* These Arduino pins must be wired to the IO0 pin of VL6180x */
int enablePin0 = 3;
int enablePin1 = 2;
int enablePin2 = 4;

/* Create a new instance for each sensor */
VL6180X sensor0;
VL6180X sensor1;
VL6180X sensor2;

MPU9250 myIMU;
int delayTime = 100;

int speedpinA = 9;
int speedpinB = 10;
int pinI1 = 8;
int pinI2 = 11;
int pinI3 = 12;
int pinI4 = 13;
int spead = 80;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  // Reset all connected sensors
  pinMode(enablePin0,OUTPUT);
  pinMode(enablePin1,OUTPUT);
  pinMode(enablePin2,OUTPUT);
  
  digitalWrite(enablePin0, LOW);
  digitalWrite(enablePin1, LOW);
  digitalWrite(enablePin2, LOW);

  pinMode(speedpinA, OUTPUT);
  pinMode(speedpinB, OUTPUT);
  pinMode(pinI1, OUTPUT);
  pinMode(pinI2, OUTPUT);
  pinMode(pinI3, OUTPUT);
  pinMode(pinI4, OUTPUT);
  
  delay(1000);

  myIMU.initMPU9250();
  myIMU.getAres();
  myIMU.getGres();
  
  // Sensor0
  Serial.println("Start Sensor 0");
  digitalWrite(enablePin0, HIGH);
  delay(50);
  sensor0.init();
  sensor0.configureDefault();
  sensor0.setAddress(address0);
  Serial.println(sensor0.readReg(0x212),HEX); // read I2C address
  sensor0.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor0.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor0.setTimeout(500);
  sensor0.stopContinuous();
  sensor0.setScaling(RANGE); // configure range or precision 1, 2 oder 3 mm
  delay(300);
  sensor0.startInterleavedContinuous(100);
  delay(100);
  //digitalWrite(enablePin1, LOW);
  // Sensor1
  Serial.println("Start Sensor 1");
  digitalWrite(enablePin1, HIGH);
  delay(50);
  sensor1.init();
  sensor1.configureDefault();
  sensor1.setAddress(address1);
  Serial.println(sensor1.readReg(0x212),HEX);
  sensor1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor1.setTimeout(500);
  sensor1.stopContinuous();
    sensor1.setScaling(RANGE);
  delay(300);
  sensor1.startInterleavedContinuous(100);
  delay(100);
  
  //* Sensor2
  Serial.println("Start Sensor 2");
  digitalWrite(enablePin2, HIGH);
  delay(50);
  sensor2.init();
  sensor2.configureDefault();
  sensor2.setAddress(address2);
  Serial.println(sensor2.readReg(0x212),HEX);
  sensor2.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor2.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor2.setTimeout(500);
  sensor2.stopContinuous();
  sensor2.setScaling(RANGE);
  delay(300);
  sensor2.startInterleavedContinuous(100);
  delay(100);
  //*/
  delay(1000);
  digitalWrite(enablePin1, HIGH);
}

void loop()
{
  Serial.println();

  // Abstand in mm
  Serial.print("\tFront: ");
  Serial.print(sensor0.readRangeContinuousMillimeters());
  if (sensor0.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.print("\tRight: ");
  Serial.print(sensor1.readRangeContinuousMillimeters());
  if (sensor1.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.print("\tLeft: ");
  Serial.print(sensor2.readRangeContinuousMillimeters());
  if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.println();

  myIMU.readGyroData(myIMU.accelCount);
  myIMU.ax = (float)myIMU.accelCount[0] * myIMU.gRes -myIMU.gyroBias[0];
  myIMU.ay = (float)myIMU.accelCount[1] * myIMU.gRes -myIMU.gyroBias[1];
  myIMU.az = (float)myIMU.accelCount[2] * myIMU.gRes -myIMU.gyroBias[2];
  Serial.println(myIMU.ay);
  
  delay(500);
}
