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


uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

uint16_t normalSpeed = 20;

//TUNING VALS
float ps = 0.003;
float is = 0.0;
float ds = 0.001;

float pl = 0.01;
float il = 0.0;
float dl = 0.005;

int baseline;
int turn_or_jump = 0;

double Setpoint, Input, Output;

PID myPID(&Input, &Output, &Setpoint, ps, is, ds, DIRECT);

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
  myPID.SetOutputLimits(-1*normalSpeed/2, normalSpeed/2);
  myPID.SetMode(AUTOMATIC);
  myPID.SetControllerDirection(REVERSE);
}

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
  baseline = sensorValAvgSum / 100 + 400;
  Serial.println(baseline);

	/* Disable both motors */
	disableMotor(BOTH_MOTORS);
}

void turnAround() {
  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS,normalSpeed/2);
  delay(900);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
}

void hop() {
  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS,normalSpeed/2);
  delay(450);
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  delay(100);
}

bool lostLine() {
  for (int i = 0; i < LS_NUM_SENSORS; i++) {
    if (sensorVal[i] > baseline) {
      return false;
    }
  }
  Serial.println("Lost");
  return true;
}

bool isCalibrationComplete = false;
void loop()
{

	/* Valid values are either:
	 *  DARK_LINE  if your floor is lighter than your line
	 *  LIGHT_LINE if your floor is darker than your line
	 */
	uint8_t lineColor = DARK_LINE;

	/* Run this setup only once */
	if(isCalibrationComplete == false) {
		floorCalibration();
		isCalibrationComplete = true;
	}

	readLineSensor(sensorVal);
	readCalLineSensor(sensorVal,
					  sensorCalVal,
					  sensorMinVal,
					  sensorMaxVal,
					  lineColor);

	uint32_t linePos = getLinePosition(sensorCalVal,lineColor);
  for (int i = 0; i < LS_NUM_SENSORS; i++) {
    Serial.print(sensorVal[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();
  if ((3500 - linePos) < 500)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(ps, is, ds);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(pl, il, dl);
  }
  Input = linePos;
  myPID.Compute();
  Serial.print("Position: ");
  Serial.print(linePos);
  Serial.print(" PID Val: ");
  Serial.print(Output);
  Serial.println();
  setMotorSpeed(LEFT_MOTOR,normalSpeed + Output);
  setMotorSpeed(RIGHT_MOTOR,normalSpeed - Output);
  if (lostLine()) {
    if (turn_or_jump % 2 == 0) {
      hop();
    } else {
      turnAround();
    }
    turn_or_jump++;
  }
}
