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
  //fusion.MahonyUpdate(gx*DEG_TO_RAD, gy*DEG_TO_RAD, gz*DEG_TO_RAD, ax*G, ay*G, az*G, deltat);  //mahony is suggested if there isn't the mag
  fusion.MadgwickUpdate(gx*DEG_TO_RAD, gy*DEG_TO_RAD, gz*DEG_TO_RAD, ax*G, ay*G, az*G, mx, my, mz, deltat);  //else use the magwick

  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();
  

  Serial.print(roll);
  Serial.print('\t');
  Serial.print(pitch);
  Serial.print('\t');
  Serial.println(yaw);

  delay(10); //for readability

}
