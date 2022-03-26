//#include <PID_v1.h>

#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

uint8_t lineColor = DARK_LINE;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  setupRSLK();
  setupWaitBtn(LP_LEFT_BTN);
  setupLed(RED_LED);
  clearMinMax(sensorMinVal, sensorMaxVal);

  for(int i = 0; i < 100; i++){
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal, sensorMinVal, sensorMaxVal);
  }

//  left_motor.begin(MOTOR_L_SLP_PIN,
//           MOTOR_L_DIR_PIN,
//           MOTOR_L_PWM_PIN);
//
//  right_motor.begin(MOTOR_R_SLP_PIN,
//            MOTOR_R_DIR_PIN,
//            MOTOR_R_PWM_PIN);
}

void loop() {
  // put your main code here, to run repeatedly: 
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal, sensorCalVal, sensorMinVal, sensorMaxVal, lineColor);
  uint32_t linePos = getLinePosition(sensorCalVal, lineColor);

  double speedMod = 0;
//  PID(linePos, speedMod, 3500, 1, 1, 1, DIRECT);

  for(int i = 0; i < 8; i++){
    Serial.print(String(sensorVal[i]) + "\t");
  }

  Serial.println("\t POS: " + String(linePos));

  
  
  
}
