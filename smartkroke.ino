#include <Arduino_LSM9DS1.h>
#include <BasicLinearAlgebra.h>
#include "SensorFusion.h"
#include "Arduino.h"

using namespace BLA;

SF fusion;
#define SS_PIN PB12

float gx, gy, gz, ax, ay, az, mx, my, mz, temp;


float T[] = {0,0};
BLA::Matrix<5,3> ledMatrix = { //shape x,y,z where z is homogenous coordinates
  0,1,1,
  0,2,1,
  0,3,1,
  0,4,1,
  0,5,1
};

float pitch, roll, yaw;
float deltat;
int status;
const float G = 9.81;
const float lengthOffset = 0;

void setup() {
  // serial to display data
  Serial.begin(9600);
  while (!Serial) {}

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
  fusion.MahonyUpdate(-gy*DEG_TO_RAD, gz*DEG_TO_RAD, gx*DEG_TO_RAD,-ay*G,  az*G, ax*G, deltat);
  //fusion.MadgwickUpdate(-gy*DEG_TO_RAD, gz*DEG_TO_RAD, gx*DEG_TO_RAD,-ay*G,  az*G, ax*G deltat);
  roll = fusion.getRollRadians();
  return roll;
}

void loop() {
  float roll;
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      roll = getRoll();
  }
  BLA::Matrix<3,3> transformationMatrix = {
    cos(roll),-sin(roll),T[0],
    sin(roll),cos(roll),T[1],
    0,0,1
  };
  BLA::Matrix<5,3> newMatrix = ~(transformationMatrix * ~ledMatrix);
  //Serial << newMatrix(0,0) << " " << newMatrix(0,1) << '\n';
  temp = roll*RAD_TO_DEG;
  Serial << temp<< '\n';
  delay(10); //for readability

}
