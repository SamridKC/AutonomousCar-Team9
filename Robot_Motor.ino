#include <Wire.h>
#include <MPU9250.h>

MPU9250 myIMU1;


void setup() {
  Serial.begin(115200);
  Wire.begin();

  myIMU1.initMPU9250();

 // myIMU1.writeByte(myIMU1.sensorAddress, SMPLRT_DIV, 0);

  myIMU1.getAres();
  myIMU1.getGres();
}

float angle = 0;

void loop() {
  

  
  myIMU1.readGyroData(myIMU1.accelCount);
  myIMU1.ax = (float)myIMU1.accelCount[0] * myIMU1.gRes -myIMU1.gyroBias[0];
  myIMU1.ay = (float)myIMU1.accelCount[1] * myIMU1.gRes -myIMU1.gyroBias[1];
  myIMU1.az = (float)myIMU1.accelCount[2] * myIMU1.gRes -myIMU1.gyroBias[2];

  if(!(myIMU1.ay < 1 && myIMU1.ay > -1)){
    angle+= myIMU1.ay / 10;
  }
  if(angle < 0)
    angle += 360;
  if(angle > 360)
    angle -= 360;
  
  Serial.println(angle);
  
  /*
  Serial.print(myIMU1.ax);
  Serial.print('\t');
  Serial.print(myIMU1.ay);
  Serial.print('\t');
  Serial.println(myIMU1.az);
  */
  delay(100);
}

