//This Code for Robotic arm 5 DOF
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"

const int mpuAddress = 0x68;  // It can be 0x68 or 0x69
MPU6050 mpu(mpuAddress);

int ax, ay, az;
int gx, gy, gz;
float orient_angle[2];

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 75   // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 475  // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;
int a = 30;
int b = 15;
int d = 150;

int m0 = 115;
int m1 = 85;
int m2 = 115;

int m15 = 120;
int m14 = 60;
int m13 = 180;

int m3 = 145;
int m4 = 140;
int m5 = 70;

int m12 = 125;
int m10 = 110;
int m9 = 65;

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  mpu.initialize();
  pwm.begin();

  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  //yield();
}


void initialiseAll() {

  //front right
  pwm.setPWM(0, 0, angleToPulse(m0));
  pwm.setPWM(1, 0, angleToPulse(m1));  //+front
  pwm.setPWM(2, 0, angleToPulse(m2));  //+down

  //back right
  pwm.setPWM(15, 0, angleToPulse(m15));
  pwm.setPWM(14, 0, angleToPulse(m14));  //+front
  pwm.setPWM(13, 0, angleToPulse(m13));  //+down

  //front left
  pwm.setPWM(3, 0, angleToPulse(m3));
  pwm.setPWM(4, 0, angleToPulse(m4));  //+back
  pwm.setPWM(5, 0, angleToPulse(m5));  //+up

  //back left
  pwm.setPWM(12, 0, angleToPulse(m12));
  pwm.setPWM(10, 0, angleToPulse(m10));  //+back
  pwm.setPWM(9, 0, angleToPulse(m9));    //+up
  //delay(1000);
}

// the code inside loop() has been updated by Robojax
void loop() {


  initialiseAll();
  delay(2500);
  //thighBack();
  for (int i = 0; i < 20; i++) {
    walking();
    delay(d);
  }
}


void walking() {

  // //---------------------------------------
  //   //front right
  pwm.setPWM(2, 0, angleToPulse(m2 - a));
  delay(d);
  pwm.setPWM(1, 0, angleToPulse(m1 + b));
  delay(d);
  pwm.setPWM(2, 0, angleToPulse(m2));
  delay(d);
  stabilise();
  thighBack1();
  delay(d);
  stabilise();
  //-----------------------------------------------
  //   //back left
  pwm.setPWM(9, 0, angleToPulse(m9 + a));  //60+40//120-40
  delay(d);
  pwm.setPWM(10, 0, angleToPulse(m10 - b));  //120-30//80+30
  delay(d);
  pwm.setPWM(9, 0, angleToPulse(m9));  //60+40//120-40
  delay(d);
  stabilise();

  //-----------------------------------------------
  //front left //140
  pwm.setPWM(5, 0, angleToPulse(m5 + a));  //120-40   //60+40 140 //60
  delay(d);
  pwm.setPWM(4, 0, angleToPulse(m4 - b));  //120-30  //80+30 //80
  delay(d);
  pwm.setPWM(5, 0, angleToPulse(m5));  //120-40   //60+40 140 //60
  delay(d);
  stabilise();
  thighBack2();
  delay(d);
  stabilise();
  //-----------------------------------------------
  //back right  //140
  pwm.setPWM(13, 0, angleToPulse(m13 - a));
  delay(d);
  pwm.setPWM(14, 0, angleToPulse(m14 + b));
  delay(d);
  pwm.setPWM(13, 0, angleToPulse(m13));
  delay(d);
  stabilise();
  //------------------------------------------------
}

int walking2() {
  initialiseAll();
  delay(1000);
  // //--------------------------------------
  //   //front right //back left
  pwm.setPWM(2, 0, angleToPulse(100));
  pwm.setPWM(9, 0, angleToPulse(100));  //120-30//80+30
  pwm.setPWM(1, 0, angleToPulse(100));
  pwm.setPWM(10, 0, angleToPulse(40));  //60+40//120-40
  delay(150);
  pwm.setPWM(2, 0, angleToPulse(160));
  pwm.setPWM(9, 0, angleToPulse(40));  //80+30//80+30
  delay(1000);
  //-----------------------------------------------
  initialiseAll();
  delay(500);
  //-----------------------------------------------
  //back right    //front left //140
  pwm.setPWM(13, 0, angleToPulse(140));
  pwm.setPWM(5, 0, angleToPulse(80));  //120-30  //80+30
  pwm.setPWM(14, 0, angleToPulse(100));
  pwm.setPWM(4, 0, angleToPulse(80));  //120-40   //60+40
  delay(150);
  pwm.setPWM(13, 0, angleToPulse(180));
  pwm.setPWM(5, 0, angleToPulse(20));  //80+30  //80+30
  delay(1000);
}

void thighBack2() {
  pwm.setPWM(1, 0, angleToPulse(m1 - b));
  pwm.setPWM(10, 0, angleToPulse(m10 + b));
  stabilise();
}

void thighBack1() {
  pwm.setPWM(4, 0, angleToPulse(m4 + b));
  pwm.setPWM(14, 0, angleToPulse(m14 - b));
  stabilise();
}
int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max
                                                     // Serial.print("Angle: ");Serial.print(ang);
                                                     // Serial.print(" pulse: ");Serial.println(pulse);67
  return pulse;
}


void orientation(){
   // Read accelerations 
  mpu.getAcceleration(&ax, &ay, &az);

  //Calculate tilt angles
  float accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2)))*(180.0 / 3.14);
  float accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2)))*(180.0 / 3.14);
  orient_angle[0] = accel_ang_x;
  orient_angle[1] = accel_ang_y;

  // Show results
  Serial.print("Tilt in X: ");
  Serial.print(accel_ang_x);
  Serial.print("\tTilt in Y:");
  Serial.println(accel_ang_y);
  delay(10);
}

void stabilise(){
  orientation();
  if(orient_angle[0] >= 30 || orient_angle[0] <= -30){
    initialise();
  }
  else if(orient_angle[1] >= 30 || orient_angle[2] <= -30){
    initialise();
  }
}
