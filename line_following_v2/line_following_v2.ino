#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>
#include <PID_v1.h>

/*
 * Energia Robot Library for Texas Instruments' Robot System Learning Kit (RSLK)
 * Line Following Example
 *
 * Summary:
 * This example has the TI Robotic System Learning Kit (TI RSLK) follow a line
 * using a basic line following algorithm. This example works on a dark floor with
 * a white line or a light floor with a dark line. The robot first needs to be calibrated
 * Then place the robot on the hit the left button again to begin the line following.
 *
 * How to run:
 * 1) Push left button on Launchpad to have the robot perform calibration.
 * 2) Robot will drive forwards and backwards by a predefined distance.
 * 3) Place the robot center on the line you want it to follow.
 * 4) Push left button again to have the robot begin to follow the line.
 *
 * Parts Info:
 * o Black eletrical tape or white electrical tape. Masking tape does not work well
 *   with IR sensors.
 *
 * Learn more about the classes, variables and functions used in this library by going to:
 * https://fcooper.github.io/Robot-Library/
 *
 * Learn more about the TI RSLK by going to http://www.ti.com/rslk
 *
 * created by Franklin Cooper Jr.
 *
 * This example code is in the public domain.
 */

#define echoPin A13
#define trigPin A14
#define debugpin A6

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

uint16_t normalSpeed = 20;

//TUNING VALS
float ps = 0.006;
float is = 0.0;
float ds = 0.002;

float pl = 0.06;
float il = 0.0;
float dl = 0.02;

int baseline;

/* Valid values are either:
 *  DARK_LINE  if your floor is lighter than your line
 *  LIGHT_LINE if your floor is darker than your line
 */
uint8_t lineColor = LIGHT_LINE;

double Setpoint, Input, Output;

PID myPID(&Input, &Output, &Setpoint, pl, il, dl, DIRECT);

//mpu
#include "Wire.h" 

float DEGREE_MOD = 0.9;

#define MPU6050_ADDR 0x68 // Alternatively set AD0 to HIGH  --> Address = 0x69

int16_t accX, accY, accZ, gyroX, gyroY, gyroZ, tRaw; 

void setup()
{
	Serial.begin(115200);

	setupRSLK();
	/* Left button on Launchpad */
	setupWaitBtn(LP_LEFT_BTN);
	/* Red led in rgb led */
	setupLed(RED_LED);
	clearMinMax(sensorMinVal,sensorMaxVal);

  Setpoint = 3500;
  Input = 0;
  //turn the PID on
  myPID.SetOutputLimits(-1*normalSpeed/4, normalSpeed/4);
  myPID.SetMode(AUTOMATIC);
  myPID.SetControllerDirection(REVERSE);

  //Distance sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(debugpin, OUTPUT);

  //MPU sensors
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // wake up!
  Wire.endTransmission(true);

  beep();

  floorCalibration();
}

//--------------------------------------------------------------------------------------
void loop()
{
	readLineSensor(sensorVal);
	readCalLineSensor(sensorVal,
					  sensorCalVal,
					  sensorMinVal,
					  sensorMaxVal,
					  lineColor);

	Input = getLinePosition(sensorCalVal,lineColor);
  Serial.print("Position: ");
  Serial.println(Input);
  for (int i = 0; i < LS_NUM_SENSORS; i++) {
    Serial.print((String)sensorVal[i] + "\t");
  }
  Serial.println();
  myPID.Compute();
  setMotorSpeed(LEFT_MOTOR,normalSpeed + Output);
  setMotorSpeed(RIGHT_MOTOR,normalSpeed - Output);
  if(lostLine()){
    hop();
  }
  if(dist() < 13) {
    turnAround();
  }
}
//---------------------------------------------------------------------------------------


void floorCalibration() {
  /* Place Robot On Floor (no line) */
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
    
  delay(1000);

  Serial.println("Running calibration on floor");
  simpleCalibrate();
  Serial.println("Reading floor values complete");

  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  delay(1000);

  enableMotor(BOTH_MOTORS);
}

void simpleCalibrate() {
  /* Set both motors direction forward */
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  /* Enable both motors */
  enableMotor(BOTH_MOTORS);
  /* Set both motors speed 20 */
  setMotorSpeed(BOTH_MOTORS,20);

  int sensorValAvgSum = 0;

  for(int x = 0;x<100;x++){
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal,sensorMinVal,sensorMaxVal);
    int sum = 0;
    for (int i = 0; i < LS_NUM_SENSORS; i++) {
      sum += sensorVal[i];
    }
    sensorValAvgSum += sum / LS_NUM_SENSORS;
  }
  baseline = sensorValAvgSum / 100 - 400;
  Serial.println(baseline);

  /* Disable both motors */
  disableMotor(BOTH_MOTORS);
}

void turnAround() {
  beep();
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS,normalSpeed);
  delay(300);
  turnByDegrees(180, normalSpeed/2);
}

void hop() {
  turnByDegrees(45, normalSpeed/2);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS,normalSpeed);
  delay(300);
  turnByDegrees(45, normalSpeed/2);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS,normalSpeed);
  delay(300);
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

bool lostLine() {
  int numBelow = 0;
  for (int i = 0; i < LS_NUM_SENSORS; i++) {
    if (sensorVal[i] < baseline) {
        return false;
    }
  }
  return true;
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
