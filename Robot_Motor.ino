#include <Wire.h>
#include <MPU9250.h>
#include <SparkFun_VL6180X.h>

VL6180xIdentification identification;
#define VL6180X_ADDRESS_1 0x50
VL6180x frontSensor(VL6180X_ADDRESS_1);
#define VL6180X_ADDRESS_2 0x51
VL6180x rightSensor(VL6180X_ADDRESS_2);


bool testing = true;

MPU9250 myIMU;
int delayTime = 100;

int speedpinA = 9;
int speedpinB = 10;
int pinI1 = 8;
int pinI2 = 11;
int pinI3 = 12;
int pinI4 = 13;
int spead = 80;

void forward(){
  analogWrite(speedpinA,spead);//input a simulation value to set the speed
  analogWrite(speedpinB,spead);
  digitalWrite(pinI4,LOW);//turn DC Motor B move clockwise
  digitalWrite(pinI3,HIGH);
  digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
  digitalWrite(pinI1,HIGH);

  delay(1000);
  stop();
}
void backward(){
  analogWrite(speedpinA,spead);//input a simulation value to set the speed
  analogWrite(speedpinB,spead);
  digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
  digitalWrite(pinI1,LOW);

  delay(1000);
  stop();
}

void turnLeft(){
  float angle = 0;
  analogWrite(speedpinA,spead);//input a simulation value to set the speed
  analogWrite(speedpinB,spead);
  digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
  digitalWrite(pinI1,HIGH);
     
  while(angle < 90){
    myIMU.readGyroData(myIMU.accelCount);
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.gRes -myIMU.gyroBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.gRes -myIMU.gyroBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.gRes -myIMU.gyroBias[2];

    if(!(myIMU.ay < 1.5 && myIMU.ay > -1.5)){
      angle+= myIMU.ay / (1000 / delayTime);
    }

    if(testing)
      Serial.println(angle);

    delay(delayTime);
  }
  stop(); 
}
void turnRight(){
  float angle = 0;
  analogWrite(speedpinA,spead);//input a simulation value to set the speed
  analogWrite(speedpinB,spead);
  digitalWrite(pinI4,LOW);//turn DC Motor B move clockwise
  digitalWrite(pinI3,HIGH);
  digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
  digitalWrite(pinI1,LOW);
     
  while(angle > -90){
    myIMU.readGyroData(myIMU.accelCount);
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.gRes -myIMU.gyroBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.gRes -myIMU.gyroBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.gRes -myIMU.gyroBias[2];

    if(!(myIMU.ay < 1.5 && myIMU.ay > -1.5)){
      angle+= myIMU.ay / (1000 / delayTime);
    }
     
    if(testing){
      Serial.println(angle);
    }

    delay(delayTime);
  }
  stop(); 
}
void halfTurn(){
  float angle = 0;
  analogWrite(speedpinA,spead);//input a simulation value to set the speed
  analogWrite(speedpinB,spead);
  digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
  digitalWrite(pinI3,LOW);
  digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
  digitalWrite(pinI1,HIGH);
     
  while(angle < 180){
    myIMU.readGyroData(myIMU.accelCount);
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.gRes -myIMU.gyroBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.gRes -myIMU.gyroBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.gRes -myIMU.gyroBias[2];

    if(!(myIMU.ay < 1.5 && myIMU.ay > -1.5)){
      angle += myIMU.ay / (1000 / delayTime);
    }
     
    if(testing)
      Serial.println(angle);

    delay(delayTime);
  }
  stop(); 
}

void stop(){
  digitalWrite(speedpinA,LOW);//input a simulation value to set the speed
  digitalWrite(speedpinB,LOW);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(speedpinA, OUTPUT);
  pinMode(speedpinB, OUTPUT);
  pinMode(pinI1, OUTPUT);
  pinMode(pinI2, OUTPUT);
  pinMode(pinI3, OUTPUT);
  pinMode(pinI4, OUTPUT);
  
  Wire.begin();

  frontSensor.changeAddress(0x29, VL6180X_ADDRESS_1);
  rightSensor.changeAddress(0x29, VL6180X_ADDRESS_2);
  if(frontSensor.VL6180xInit() != 0){
    Serial.println("FAILED TO INITALIZE FRONT SENSOR"); //Initialize device and check for errors
  };
  if(rightSensor.VL6180xInit() != 0){
    Serial.println("FAILED TO INITALIZE RIGHT SENSOR"); //Initialize device and check for errors
  };

  frontSensor.VL6180xDefautSettings(); //Load default settings to get started.
  rightSensor.VL6180xDefautSettings(); //Load default settings to get started.
  
  myIMU.initMPU9250();
  myIMU.getAres();
  myIMU.getGres();
}

void loop() { 
  /*
  turnRight();
  delay(3000);
  ///
  turnLeft();
  delay(1000);
  ///
  halfTurn();
  delay(1000);
  ///
  forward();
  delay(1000);
  ///
  backward();
  delay(1000);
  //*/
  myIMU.readGyroData(myIMU.accelCount);
  myIMU.ax = (float)myIMU.accelCount[0] * myIMU.gRes -myIMU.gyroBias[0];
  myIMU.ay = (float)myIMU.accelCount[1] * myIMU.gRes -myIMU.gyroBias[1];
  myIMU.az = (float)myIMU.accelCount[2] * myIMU.gRes -myIMU.gyroBias[2];
  Serial.println(myIMU.ay);

  Serial.print("Front distance measured (mm) = ");
  Serial.println(frontSensor.getDistance() ); 
  Serial.print("Right distance measured (mm) = ");
  Serial.println(rightSensor.getDistance() ); 
  delay(100);
}

