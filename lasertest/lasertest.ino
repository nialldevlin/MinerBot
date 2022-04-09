#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>

#define laserpin A12
#define photopin A11

int val;

void setup() {
  // put your setup code here, to run once:
  pinMode(laserpin, OUTPUT);
  pinMode(photopin, INPUT);
  Serial.begin(9600);

  setupRSLK();
  enableMotor(BOTH_MOTORS);

  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS, 20);
  delay(500);

  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
  setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS, 20);
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
