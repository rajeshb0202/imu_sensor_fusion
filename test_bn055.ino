#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include  <math.h>
 
#define BNO055_SAMPLERATE_DELAY_MS (100)
 
Adafruit_BNO055 myIMU = Adafruit_BNO055();

//accelerometer related variables
float theta_acc_M= 0;               //theta_acc_M is the measured angle of inclination of the accelerometer with respect to the x-axis
float phi_acc_M=0;                 //phi_acc_M is the measured angle of inclination of the accelerometer with respect to the y-axis
float theta_acc_F_old = 0;          //theta_acc_F_old is the filtered angle of inclination of the accelerometer with respect to the x-axis
float theta_acc_F_new = 0;
float phi_acc_F_old = 0;
float phi_acc_F_new = 0;


//filter related variables
float theta_gyr_M = 0;
float phi_gyr_M = 0;

float theta_overall = 0;
float phi_overall = 0;

float dt;
unsigned long millis_old = 0;



float p1 = 0.90;
float p2 = 1 - p1;



void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
myIMU.begin();
delay(1000);
int8_t temp=myIMU.getTemp();
myIMU.setExtCrystalUse(true);
millis_old = millis();
}
 
void loop() {
  // put your main code here, to run repeatedly:
uint8_t system, gyro, accel, mag = 0;
myIMU.getCalibration(&system, &gyro, &accel, &mag);

imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> gyr =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

theta_acc_M = atan2(acc.x()/9.8, acc.z()/9.8) * 180 / PI;
phi_acc_M = atan2(acc.y()/9.8, acc.z()/9.8) * 180 / PI;


// Lesson no 7: implemented loww pass filter
theta_acc_F_new = p1*theta_acc_F_old + p2*theta_acc_M;    
phi_acc_F_new = p1*phi_acc_F_old + p2*phi_acc_M;


//Lesson no 8: Gyroscope
dt = (millis() - millis_old)/1000.;
millis_old = millis();

theta_gyr_M = theta_gyr_M + gyr.y()*dt;
phi_gyr_M = phi_gyr_M + gyr.x()*dt;


//Lesson no 9: Complimentary Filter
theta_overall = (theta_overall - gyr.y()*dt) * p1  + theta_acc_M * p2;
phi_overall = (phi_overall + gyr.x()*dt) * p1  + phi_acc_M * p2;


//Lesson no-9: experimentation of complimentary filter                      //using this resulted in lagging effect. Produced worse results than using only measued acc. values
// theta_overall = (theta_overall - gyr.y()*dt) * p1  + theta_acc_F_new * p2;
// phi_overall = (phi_overall + gyr.x()*dt) * p1  + phi_acc_F_new * p2;





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


Serial.print(theta_acc_M);
Serial.print(",");
Serial.print(theta_acc_F_new);
Serial.print(",");
Serial.print(phi_acc_M);
Serial.print(",");
Serial.print(phi_acc_F_new);

Serial.print(",");
Serial.print(-1 *theta_gyr_M);
Serial.print(",");
Serial.print(phi_gyr_M);

Serial.print(",");
Serial.print(theta_overall);
Serial.print(",");
Serial.println(phi_overall);

theta_acc_F_old = theta_acc_F_new;      
phi_acc_F_old = phi_acc_F_new;

 
delay(BNO055_SAMPLERATE_DELAY_MS);
 
}