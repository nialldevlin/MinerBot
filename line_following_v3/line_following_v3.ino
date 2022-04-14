#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>

#include "Wire.h"

#define echoPin 11
#define trigPin 31
#define debugpin 12
#define PHOTO_PIN 6 //arbitrary
#define IRPIN A11

float HOME_DIST = 26;
float CORNER_DIST = 24;
float SIDE_DIST = 20;

float MIN_DIST = 10;

//Encoder
/* Diameter of Romi wheels in inches */
float wheelDiameter = 2.7559055;
/* Number of encoder (rising) pulses every time the wheel turns completely */
int cntPerRevolution = 360;

//Color Sensor
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
uint8_t lineColor = LIGHT_LINE;
int baseline;

//Speed
int speed = 20;
int diff = 10;
float MOD = 0.97;

//Laser
int lpin = 12;
int photoPin = A6;


//MPU
#define MPU6050_ADDR 0x68 // Alternatively set AD0 to HIGH  --> Address = 0x69
float DEGREE_MOD = 0.9;
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ, tRaw;

bool found = false; //Has the ball
int angle = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


  //Laser
  pinMode(lpin, OUTPUT);
  pinMode(photoPin, INPUT);  
  
  setupRSLK();
  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);

  //MPU sensors
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // wake up!
  Wire.endTransmission(true);
  waitBtnPressed(LP_LEFT_BTN,"Wait",RED_LED);
  beep();
  goInches(26, speed);
  turnByDegrees(-145, speed);
}

int count = 0;
void loop() {
  int d;
  if (count % 2 == 0) d = 45;
  else d = 45; 
  if (!found and count < 8) {
    goCM(d, speed);
    count++;
  } else {
    goHome();
  }
  found = foundBall();
  disableMotor(BOTH_MOTORS);
  delay(100);
  goCM(-1 * d, speed);
  turnByDegrees(40, speed);
}

int dist() {
  int duration, distance;
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return distance;
}

void goHome(){
  turnByDegrees(180 - angle, speed);
  goCM(70, speed);
  beep();
  beep();
}

bool foundBall(){
  int v = analogRead(photoPin);
  Serial.println(v);
  return v > 400;
}

/* The distance the wheel turns per revolution is equal to the diameter * PI.
 * The distance the wheel turns per encoder pulse is equal to the above divided
 * by the number of pulses per revolution.
 */
float distanceTraveled(float wheel_diam, uint16_t cnt_per_rev, uint8_t current_cnt) {
  float temp = (wheel_diam * PI * current_cnt) / cnt_per_rev;
  return temp;
}


uint32_t countForDistance(float wheel_diam, uint16_t cnt_per_rev, uint32_t distance) {
  float temp = (wheel_diam * PI) / cnt_per_rev;
  temp = distance / temp;
  return int(temp);
}

void goCM(float cm, int s) {
  int in = cm / 2.54;
  goInches(in, s);
}

void goInches(float inches, int s) {
  enableMotor(BOTH_MOTORS);
  int totalCount = 0;
  /* Amount of encoder pulses needed to achieve distance */
  uint16_t x = countForDistance(wheelDiameter, cntPerRevolution, abs(inches));
  x = 0.96 * x;
  /* Set the encoder pulses count back to zero */
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  /* Cause the robot to drive forward */
  if(inches > 0)  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  else setMotorDirection(BOTH_MOTORS,MOTOR_DIR_BACKWARD);
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(LEFT_MOTOR,s*MOD);
  setMotorSpeed(RIGHT_MOTOR,s);
  /* Drive motor until it has received x pulses */
  while(totalCount < x)
  {
    totalCount = getEncoderRightCnt();
  }
  disableMotor(BOTH_MOTORS);
}

void turnByDegrees(float deg, int speed){
  float degTotal = 0;
  float previousMil = millis();
  bool turnRight = deg > 0;
  enableMotor(BOTH_MOTORS);
  while(abs(degTotal) < abs(deg*DEGREE_MOD)){ 
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. 
                                 // As a result, the connection is kept active.
    Wire.requestFrom(MPU6050_ADDR, 14, true); 
  
    accX = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accY = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    accZ = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    tRaw = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
    gyroX = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    gyroY = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  
    gyroZ = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
    float gyroDeg = (gyroZ / 131.0);
    gyroDeg = (abs(gyroDeg) < 1)? 0: gyroDeg;
    
    degTotal += (gyroDeg / 1000) * (millis() - previousMil);
    previousMil = millis();

    if(turnRight){
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    }
    else{
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    }
    setMotorSpeed(RIGHT_MOTOR, speed);
    setMotorSpeed(LEFT_MOTOR, speed);
    
    delay(10);
  }
  angle += deg;
  disableMotor(BOTH_MOTORS);
}

void beep() {
  disableMotor(BOTH_MOTORS);
  digitalWrite(debugpin, HIGH);
  delay(500);
  digitalWrite(debugpin, LOW);
  enableMotor(BOTH_MOTORS);
}
