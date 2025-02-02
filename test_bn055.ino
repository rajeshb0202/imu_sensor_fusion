#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include  <math.h>
 
#define BNO055_SAMPLERATE_DELAY_MS (100)
 
Adafruit_BNO055 myIMU = Adafruit_BNO055();

float theta_M= 0;
float phi_M=0;
float theta_F_old = 0;
float theta_F_new = 0;
float phi_F_old = 0;
float phi_F_new = 0;

float p1 = 0.95;
float p2 = 1 - p1;



void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
myIMU.begin();
delay(1000);
int8_t temp=myIMU.getTemp();
myIMU.setExtCrystalUse(true);
}
 
void loop() {
  // put your main code here, to run repeatedly:
uint8_t system, gyro, accel, mag = 0;
myIMU.getCalibration(&system, &gyro, &accel, &mag);

imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

theta_M = atan2(acc.x()/9.8, acc.z()/9.8) * 180 / PI;
phi_M = atan2(acc.y()/9.8, acc.z()/9.8) * 180 / PI;



theta_F_new = p1*theta_F_old + p2*theta_M;
phi_F_new = p1*phi_F_old + p2*phi_M;


Serial.print(acc.x()/9.8);
Serial.print(",");
Serial.print(acc.y()/9.8);
Serial.print(",");
Serial.print(acc.z()/9.8);

Serial.print(",");
Serial.print(accel);
Serial.print(",");
Serial.print(gyro);
Serial.print(",");
Serial.print(mag);
Serial.print(",");
Serial.print(system);
Serial.print(",");
Serial.print(theta_M);
Serial.print(",");
Serial.print(theta_F_new);
Serial.print(",");
Serial.print(phi_M);
Serial.print(",");
Serial.println(phi_F_new);

theta_F_old = theta_F_new;
phi_F_old = phi_F_new;

 
delay(BNO055_SAMPLERATE_DELAY_MS);
 
}