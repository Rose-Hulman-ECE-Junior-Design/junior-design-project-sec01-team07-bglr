/*
 * Header file for ESP32 Vehicle.
 * 
 * Author: CKG
 */

#ifndef ESP32_VEHICLE_H
#define ESP32_VEHICLE_H

//#include any things I need
#include <SoftwareSerial.h>   //include the espsoftwareserial library
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "BluetoothSerial.h"


// CONSTANTS ==================================================================
#define SERIAL_BAUD 115200

#define STEERING_SERVO 32
#define SPEED_SERVO 33

#define SERVO_FREQ 50                             // 50 Hz, 
#define SERVO_PERIOD 20                           // Period = 20ms
#define PWM_RESOLUTION 12                         //12-bit resolution
#define MAX_COUNT 4095                            //2^PWM_RESOLUTION - 1

#define STEERING_MIN_PW           1               //minimum pulse width, ms
#define STEERING_MAX_PW           2               //max pulse width, ms
#define STEERING_RANGE            1
#define STEERING_MAX_ANGLE        165.0
#define STEERING_MIN_ANGLE        25.0

#define SPEED_MIN_PW              1                 //minimum pulse width, ms
#define SPEED_MAX_PW              2                 //max pulse width, ms
#define SPEED_RANGE               1
#define SPEED_MAX_DUTY            235
#define SPEED_MIN_DUTY            350 
#define SPEED_MAX_INC             65

//============================================================================

// GLOBAL VARIABLES ==========================================================

extern float steeringAngle = 90.0; 
extern float motorSpeed = 50; 
extern float shuntvoltage = 0; 
extern float busvoltage = 0; 
extern float current_mA = 0; 
extern float loadvoltage = 0; 
extern float power_mW = 0; 

HUSKYLENSResult*  huskylens_arrow;  //pointer to a HUSKYLENS result type

enum VehicleState{ 
  IDLE, 
  DRIVING, 
  RECHARGING 
} 

extern VehicleState currentState = IDLE; 

extern BluetoothSerial SerialBT;
extern HUSKYLENS huskylens;
extern Adafruit_INA219 ina219;



// FUNCTION PROTOTYPES =======================================================
void initSerialMonitor();
void initINA219();
void initHUSKYLENS();
void initSteeringServo();
void initSpeedServo();

void setSteeringAngle(float angle);
void setServoSpeed(float duty);
float calculateSteeringAngle();
float calculateServoSpeed();

void readINA219();       //
int readGUICommand();   // parse command from the GUI
void parseGUICommand();
void sendDataLog();      // send the data log to the GUI


#endif      // ESP32_VEHICLE_H
