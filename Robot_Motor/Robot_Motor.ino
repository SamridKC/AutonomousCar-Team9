#include <Wire.h>
#include <MPU9250.h>
#include <VL6180X.h>

bool angleTesting = true;
bool turnTesting = true;

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
int delayTime = 25;
float turnspeed = 1;
float turnBuffer = .9;

int speedpinA = 9;
int speedpinB = 10;
int pinI1 = 8;
int pinI2 = 11;
int pinI3 = 12;
int pinI4 = 13;
int spead = 140;

void forward(){
  analogWrite(speedpinA, spead);//input a simulation value to set the speed
  analogWrite(speedpinB, spead);
  digitalWrite(pinI4,LOW);//turn DC Motor B move clockwise
  digitalWrite(pinI3,HIGH);
  digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
  digitalWrite(pinI1,HIGH);
}

void backward(){
  stopMotors();
  analogWrite(speedpinA,spead);//input a simulation value to set the speed
  analogWrite(speedpinB,spead);
  digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
  digitalWrite(pinI1,LOW);

  delay(250);
  stopMotors();
}

float turnLeft(){
  stopMotors();
  float angle = 0;
  analogWrite(speedpinA,spead * turnspeed);//input a simulation value to set the speed
  analogWrite(speedpinB,spead * turnspeed);
  digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
  digitalWrite(pinI1,HIGH);
     
  while(angle < 90*turnBuffer){
    myIMU.readGyroData(myIMU.accelCount);
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.gRes -myIMU.gyroBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.gRes -myIMU.gyroBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.gRes -myIMU.gyroBias[2];

    if(!(myIMU.ay < 1.5 && myIMU.ay > -1.5)){
      angle+= myIMU.ay / (1000 / delayTime);
    }

    if(angleTesting)
      Serial.println(angle);

    delay(delayTime);
  }
  stopMotors(); 
}
float turnRight(){
  stopMotors();
  float angle = 0;
  analogWrite(speedpinA,spead * turnspeed);//input a simulation value to set the speed
  analogWrite(speedpinB,spead * turnspeed);
  digitalWrite(pinI4,LOW);//turn DC Motor B move clockwise
  digitalWrite(pinI3,HIGH);
  digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
  digitalWrite(pinI1,LOW);
     
  while(angle > -90*turnBuffer){
    myIMU.readGyroData(myIMU.accelCount);
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.gRes -myIMU.gyroBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.gRes -myIMU.gyroBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.gRes -myIMU.gyroBias[2];

    if(!(myIMU.ay < 1.5 && myIMU.ay > -1.5)){
      angle+= myIMU.ay / (1000 / delayTime);
    }
     
    if(angleTesting){
      Serial.println(angle);
    }

    delay(delayTime);
  }
  stopMotors(); 
}
float halfTurn(){
  stopMotors();
  float angle = 0;
  analogWrite(speedpinA,spead * turnspeed);//input a simulation value to set the speed
  analogWrite(speedpinB,spead * turnspeed);
  digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
  digitalWrite(pinI1,HIGH);
     
  while(angle < 180*turnBuffer){
    myIMU.readGyroData(myIMU.accelCount);
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.gRes -myIMU.gyroBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.gRes -myIMU.gyroBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.gRes -myIMU.gyroBias[2];

    if(!(myIMU.ay < 1.5 && myIMU.ay > -1.5)){
      angle += myIMU.ay / (1000 / delayTime);
    }
     
    if(angleTesting)
      Serial.println(angle);

    delay(delayTime);
  }
  stopMotors();
}

float degreeTurn(float turnAmount){
  stopMotors();
  float angle = 0;
  if(turnAmount > 0){
    analogWrite(speedpinA,spead * turnspeed);//input a simulation value to set the speed
    analogWrite(speedpinB,spead * turnspeed);
    digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
    digitalWrite(pinI3,LOW);
    digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
    digitalWrite(pinI1,HIGH);
  }else{
    analogWrite(speedpinA,spead * turnspeed);//input a simulation value to set the speed
    analogWrite(speedpinB,spead * turnspeed);
    digitalWrite(pinI4,LOW);//turn DC Motor B move clockwise
    digitalWrite(pinI3,HIGH);
    digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
    digitalWrite(pinI1,LOW);
  }
     
    while(turnAmount > 0 ? angle < turnAmount*turnBuffer : angle > turnAmount*turnBuffer){
      myIMU.readGyroData(myIMU.accelCount);
      myIMU.ax = (float)myIMU.accelCount[0] * myIMU.gRes -myIMU.gyroBias[0];
      myIMU.ay = (float)myIMU.accelCount[1] * myIMU.gRes -myIMU.gyroBias[1];
      myIMU.az = (float)myIMU.accelCount[2] * myIMU.gRes -myIMU.gyroBias[2];
  
      if(!(myIMU.ay < 1.5 && myIMU.ay > -1.5)){
        angle += myIMU.ay / (1000 / delayTime);
      }
       
      if(angleTesting)
        Serial.println(angle);

      delay(delayTime);
    }
  stopMotors();
}

void stopMotors(){
  if(turnTesting)
       Serial.println("Stopping Motors");
  
  digitalWrite(speedpinA,LOW);//input a simulation value to set the speed
  digitalWrite(speedpinB,LOW);
}

void VL6180Xsetup(){
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
  delay(1000);
}


void setup() {
  Serial.begin(115200);

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
  
  Wire.begin();

  VL6180Xsetup();
  
  myIMU.initMPU9250();
  myIMU.getAres();
  myIMU.getGres();

  forward();
  delay(2000);
}

short forwardRead = 0, rightRead = 0, leftRead = 0;
void ReadSensors(){
  int forward[5], right[5], left[5];
  for(int i = 0; i < 5; i++){
    forward[i] = sensor0.readRangeContinuousMillimeters();
    right[i] = sensor1.readRangeContinuousMillimeters();
    left[i] = sensor2.readRangeContinuousMillimeters();
  }
  forwardRead = (forward[0] + forward[1] + forward[2] + forward[3] + forward[4]) / 5;
  rightRead = (right[0] + right[1] + right[2] + right[3] + right[4]) / 5;
  leftRead = (left[0] + left[1] + left[2] + left[3] + left[4]) / 5;
}

int distance = 250;
int forwardDistance = 150;
int adjustDistance = 120;
int waitTime = 900;

void loop() {
  //*
  ReadSensors();

  if(forwardRead > forwardDistance){                          // If nothing close enough in front of you
    if(rightRead > distance && leftRead > distance){   // and if there's no walls next to you
    }                                                // then it's an intersection ,and go forward still.
    else if(rightRead > distance){                   // else if there is no wall to the right, but there was one to the left
      if(turnTesting)
        Serial.println("Right turn");
      delay(waitTime);
      stopMotors();
      ReadSensors();
      if(forwardRead > forwardDistance && rightRead > distance && leftRead < distance){
        turnRight();                                // and turn right.
      }
    }
    else if(leftRead > distance){                 // if there is a wall to the right, an no wall to the left
      if(turnTesting)
        Serial.println("Left turn");
      delay(waitTime);
      stopMotors();
      ReadSensors();
      if(forwardRead > forwardDistance && rightRead < distance && leftRead > distance){
        turnLeft();                                // and turn left.
      }                             // and turn left
    }                                       
    else{                                         // else if you're going forward
      if((leftRead - rightRead ) > 10) {                        
         if(turnTesting)
            Serial.println("Adjusting left");
         degreeTurn(2);
       }
       if((rightRead - leftRead ) > 10){                    
          if(turnTesting)
            Serial.println("Adjusting right");
         degreeTurn(-2);
       }
    }
  }
  else if(forwardRead <= forwardDistance){                    // Else if there is a wall in front of you    
    stopMotors();  
    backward();
    if(rightRead > distance){                        // and there is no wall to the right
      if(turnTesting)
        Serial.println("Right turn");
      delay(waitTime);
      turnRight();                                // and turn right,
    }                                             
    else if(leftRead > distance){                   // else if there is a wall to the right, bu no wall to the left
      if(turnTesting)
        Serial.println("Left turn");
      delay(waitTime);
      turnLeft();                                 // and turn left.
    }                                      
    else{                                         // else there's no path to go
      if(turnTesting)
        Serial.println("Half turn");
      halfTurn();                                 // and turn around
    }
  }
  delay(100);
  forward();                // go forward after all the logic of turning
  delay(50);
  //*/
  /*
  turnLeft();
  delay(2000);
  turnRight();
  delay(2000);
  degreeTurn(45);
  delay(2000);
  degreeTurn(-45);
  delay(2000);
  degreeTurn(10);
  delay(2000);
  degreeTurn(-10);
  delay(2000);
  //*/
}

