/*
 * ESP32_Vehicle.h
 * 
 * Header file for ESP32 Vehicle.
 * 
 * Author: CKG
 */

#ifndef ESP32_VEHICLE_H
#define ESP32_VEHICLE_H

//include any things we need
#pragma once
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

#define SERVO_FREQ                50                             // 50 Hz, 
#define SERVO_PERIOD              20                           // Period = 20ms
#define PWM_RESOLUTION            12                         //12-bit resolution
#define MAX_COUNT                 4095                 //2^PWM_RESOLUTION - 1

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

#define THETA_TARGET              0.0
#define BT_BUFFER_SIZE            255

#define NUM_DIGITS_AFTER_DECIMAL  3
#define FLOAT_MIN_WIDTH           6
#define FLOAT_BUFF_SIZE           8
#define PACKAGE_SIZE              80

//============================================================================

// GLOBAL VARIABLES (Declarations Only) ======================================

extern float steeringAngle; 
extern float motorSpeed; 
extern float shuntvoltage; 
extern float busvoltage; 
extern float current_mA; 
extern float loadvoltage; 
extern float power_mW; 

enum VehicleState{ 
  IDLE, 
  DRIVING, 
  RECHARGING 
};

extern VehicleState currentState;

//extern Serial;
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
