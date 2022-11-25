#include <Arduino_LSM9DS1.h>
#include <BasicLinearAlgebra.h>
#include "SensorFusion.h"
#include "Arduino.h"

using namespace BLA;

SF fusion;
#define SS_PIN PB12
#define NUM_LEDS 5
#define LED_SPACING 1 //cm

float gx, gy, gz, ax, ay, az, temp;


float T[] = {0,0};
BLA::Matrix<3,NUM_LEDS> ledMatrix;
BLA::Matrix<1,3> filterX = {1,0,0}; //to show only X coordinate
BLA::Matrix<1,3> filterY = {0,1,0}; //to show only Y coordinate


float pitch, roll, yaw;
float deltat;
int status;
const float G = 9.81;
const float lengthOffset = 0;

void initLedMatrix(){
  Serial.println(ledMatrix.Cols);
  Serial.println(ledMatrix.Rows);
  for(int i = 0; i < NUM_LEDS; i++){
      ledMatrix(0, i) = 0;
      ledMatrix(1, i) = i*LED_SPACING;
      ledMatrix(2, i) = 1;
}}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  
  // initialize LED matrix
  
  initLedMatrix();
  // serial to display data

  Serial.print("got here");
  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    while (1) {}
  }
}

float getRoll() {
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  deltat = fusion.deltatUpdate();
  // fusion.MahonyUpdate(-gy*DEG_TO_RAD, gz*DEG_TO_RAD, gx*DEG_TO_RAD,-ay*G,  az*G, ax*G, deltat);
  fusion.MadgwickUpdate(-gy*DEG_TO_RAD, gz*DEG_TO_RAD, gx*DEG_TO_RAD,-ay*G,  az*G, ax*G, deltat); //Mag axis still unsure
  roll = fusion.getRollRadians();
  return roll;
}

void loop() {
  float roll;
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
  
    roll = getRoll();
    BLA::Matrix<3,3> transformationMatrix = {
    cos(roll),-sin(roll),T[0],
    sin(roll),cos(roll),T[1],
    0,0,1};
    BLA::Matrix<3,NUM_LEDS> newMatrix = transformationMatrix * ledMatrix;
    Serial << filterX*newMatrix << '\n';
    }

    
    
  }
