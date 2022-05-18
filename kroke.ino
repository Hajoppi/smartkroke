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

void loop() {

  if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
    }
  
  if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
    }
  if (IMU.magneticFieldAvailable()){
    IMU.readMagneticField(mx,my,mz);
  }

  deltat = fusion.deltatUpdate();
  
  fusion.MahonyUpdate(-gy*DEG_TO_RAD, gz*DEG_TO_RAD, gx*DEG_TO_RAD,-ay*G,  az*G, ax*G, deltat);
  //fusion.MadgwickUpdate(-gy*DEG_TO_RAD, gz*DEG_TO_RAD, gx*DEG_TO_RAD,-ay*G,  az*G, ax*G, mx, my, mz, deltat); //Mag axis still unsure
  
  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();

  //For Serial plotter
  Serial.print("roll:"); Serial.print(roll); Serial.print(", ");
  Serial.print("pitch:"); Serial.print(pitch); Serial.print(", ");
  Serial.print("yaw:"); Serial.print(yaw); Serial.print(", ");
  Serial.println();

  delay(10); //for readability

}
