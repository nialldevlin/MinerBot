
#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>

#include "Wire.h"

#define echoPin A13
#define trigPin A14
#define debugpin A6

float DEGREE_MOD = 0.9;

#define MPU6050_ADDR 0x68 // Alternatively set AD0 to HIGH  --> Address = 0x69

int16_t accX, accY, accZ, gyroX, gyroY, gyroZ, tRaw;

float DIST_BASE;

float NORMAL_SPEED = 20;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupRSLK();
  setupWaitBtn(LP_LEFT_BTN);

  //MPU sensors
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // wake up!
  Wire.endTransmission(true);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(debugpin, OUTPUT);

  beep();
  Serial.println("READY");

  waitBtnPressed(LP_LEFT_BTN,"Waiting",RED_LED);

  DIST_BASE = dist();

  enableMotor(BOTH_MOTORS);

  setMotorSpeed(BOTH_MOTORS, NORMAL_SPEED);
  Serial.println("HERE");
  Serial.println((String)dist() + "\t" + (String)DIST_BASE);
  while(dist() > DIST_BASE / 2.0){
    Serial.println((String)dist() + "\t" + (String)DIST_BASE);
  }
  disableMotor(BOTH_MOTORS);
  delay(1000);
  turnByDegrees(-135, NORMAL_SPEED);
  setMotorSpeed(BOTH_MOTORS, 0);
  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly: 
  setMotorSpeed(BOTH_MOTORS, NORMAL_SPEED);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  float homeDist = dist();
  while(dist() > 10);
  setMotorSpeed(BOTH_MOTORS, 0);
  delay(1000);
  
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
  while(dist() < homeDist);
  setMotorSpeed(BOTH_MOTORS, 0);
  delay(1000);

  turnByDegrees(45, NORMAL_SPEED);
  
  driftCorrect();
  
}

void driftCorrect(){
  while(abs(DIST_BASE - dist()) > 1){
    if(DIST_BASE - dist() > 0){
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
      setMotorSpeed(BOTH_MOTORS, NORMAL_SPEED / 2.0);
    }
    else if(DIST_BASE - dist() < 0){
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
      setMotorSpeed(BOTH_MOTORS, NORMAL_SPEED / 2.0);
    }
  }
}

int dist() {
  int distance;
  int duration;
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

void turnByDegrees(float deg, int speed){
  float degTotal = 0;
  float previousMil = millis();
  bool turnRight = deg > 0;
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
}

void beep() {
  disableMotor(BOTH_MOTORS);
  digitalWrite(debugpin, HIGH);
  delay(500);
  digitalWrite(debugpin, LOW);
  enableMotor(BOTH_MOTORS);
}
