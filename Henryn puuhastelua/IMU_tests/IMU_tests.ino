#include <Arduino_LSM9DS1.h>
#include "SensorFusion.h"
#include "Arduino.h"

SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float pitch, roll, yaw;
float deltat;
const float G = 9.81;

#define SS_PIN PB12
int status;


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

void printAttitude()
{
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  deltat = fusion.deltatUpdate();
  fusion.MahonyUpdate(-gy*0.0174532925, gz*0.0174532925, gx*0.0174532925,-ay*9.81,  az*9.81, ax*9.81, deltat);
  //fusion.MadgwickUpdate(-gy*0.0174532925, gz*0.0174532925, gx*0.0174532925,-ay*9.81,  az*9.81, ax*9.81, deltat);
  
  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();
  
  //For Serial plotter
  Serial.print("roll:"); Serial.print(roll); Serial.print(", ");
  Serial.print("pitch:"); Serial.print(pitch); Serial.print(", ");
  Serial.print("yaw:"); Serial.print(yaw); Serial.print(", ");
  Serial.println();
}

void loop() {

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){
    printAttitude();
  }
     
}
