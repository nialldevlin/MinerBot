#include <Bump_Switch.h>
#include <Encoder.h>
#include <GP2Y0A21_Sensor.h>
#include <QTRSensors.h>
#include <Romi_Motor_Power.h>
#include <RSLK_Pins.h>
#include <SimpleRSLK.h>

#include "Wire.h" 

#define MPU6050_ADDR 0x68 // Alternatively set AD0 to HIGH  --> Address = 0x69

int16_t accX, accY, accZ, gyroX, gyroY, gyroZ, tRaw; // Raw register values (accelaration, gyroscope, temperature)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //RSLK 
  setupRSLK();

  //MPU sensors
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // wake up!
  Wire.endTransmission(true);
}

float degTotal = 0;
float previousMil = millis();
float gyroPrevious = 0;
void loop() {
  if(degTotal < 90){
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(RIGHT_MOTOR, 30);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorSpeed(LEFT_MOTOR, 30);
  }
  else {
    disableMotor(BOTH_MOTORS);
  }
  
  // put your main code here, to run repeatedly: 
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. 
                                 // As a result, the connection is kept active.
    Wire.requestFrom(MPU6050_ADDR, 14, true); // request a total of 7*2=14 registers
  
    accX = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accY = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    accZ = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    tRaw = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
    gyroX = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    gyroY = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)

    gyroZ = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
    gyroZ = (abs(gyroZ) < 1)? 0: gyroZ;
     
    // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
    Serial.println("Gyz=" + (String)gyroZ + "\tDeg/s: " + (String)(gyroZ/131.0) +"\t"+ (String)(gyroZ/65.5)); 
    degTotal += ((gyroZ / 131.0) / 1000.0) * (millis() - previousMil);
    previousMil = millis();
    Serial.println((String)degTotal);
    delay(10);


  
}
