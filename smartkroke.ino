#include <Arduino_LSM9DS1.h>
#include <BasicLinearAlgebra.h>
#include "SensorFusion.h"
#include "Arduino.h"
// led stuff
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define led_pin 6
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, led_pin, NEO_GRB + NEO_KHZ800);

using namespace BLA;

SF fusion;
#define SS_PIN PB12

float gx, gy, gz, ax, ay, az, mx, my, mz, temp;


float T[] = {0,0};
BLA::Matrix<60,3> ledMatrix = { //shape x,y,z where z is homogenous coordinates
  0,1,1,
  0,2,1,
  0,3,1,
  0,4,1,
  0,5,1,
  0,6,1,
  0,7,1,
  0,8,1,
  0,9,1,
  0,10,1,
  0,11,1,
  0,12,1,
  0,13,1,
  0,14,1,
  0,15,1,
  0,16,1,
  0,17,1,
  0,18,1,
  0,19,1,
  0,20,1,
  0,21,1,
  0,22,1,
  0,23,1,
  0,24,1,
  0,25,1,
  0,26,1,
  0,27,1,
  0,28,1,
  0,29,1,
  0,30,1,
  0,31,1,
  0,32,1,
  0,33,1,
  0,34,1,
  0,35,1,
  0,36,1,
  0,37,1,
  0,38,1,
  0,39,1,
  0,40,1,
  0,41,1,
  0,42,1,
  0,43,1,
  0,44,1,
  0,45,1,
  0,46,1,
  0,47,1,
  0,48,1,
  0,49,1,
  0,50,1,
  0,51,1,
  0,52,1,
  0,53,1,
  0,54,1,
  0,55,1,
  0,56,1,
  0,57,1,
  0,58,1,
  0,59,1,
  0,60,1
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

  //setup leds
   #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code
  strip.begin();
  strip.setBrightness(50);
  strip.show();

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    while (1) {}
  }
  delay(1000);
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
  BLA::Matrix<3,3> transformationMatrix = {
    cos(roll),-sin(roll),T[0],
    sin(roll),cos(roll),T[1],
    0,0,1
  };
  BLA::Matrix<60,3> newMatrix = ~(transformationMatrix * ~ledMatrix);
  //Serial << newMatrix(0,0) << " " << newMatrix(0,1) << '\n';
  //Serial << ledMatrix(0,0) << " " << ledMatrix(0,1) << '\n';
  //temp = roll*RAD_TO_DEG;
  //Serial << temp<< '\n';
  //delay(1000); //for readability

  //strip.setPixelColor(i, c);
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    if(newMatrix(i,1) < -15 && newMatrix(i,1) > -17){
      strip.setPixelColor(i, strip.Color(255,0,0));
    }
    else if(newMatrix(i,0) < -15 && newMatrix(i,0) > -17){
      strip.setPixelColor(i, strip.Color(0,255,0));
    }else{
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
  }
  //Serial.println(newMatrix(10,1));
  strip.show();
}}
