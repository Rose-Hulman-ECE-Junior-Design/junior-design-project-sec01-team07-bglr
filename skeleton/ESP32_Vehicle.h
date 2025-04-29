/*
 * ESP32_Vehicle.h
 * 
 * Header file for ESP32 Vehicle.
 * 
 * Author: CKG, BB
 */

#ifndef ESP32_VEHICLE_H
#define ESP32_VEHICLE_H

//include any things we need
#pragma once
#include <SoftwareSerial.h>   //include the espsoftwareserial library
//#include <Wire.h>
#include <Adafruit_INA219.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "BluetoothSerial.h"


// CONSTANTS ==================================================================
#define SERIAL_BAUD               115200

#define STEERING_SERVO            32
#define SPEED_SERVO               33

#define SERVO_FREQ                50            // 50 Hz, 
#define SERVO_PERIOD              20            // Period = 20ms
#define PWM_RESOLUTION            12            // 12-bit resolution
#define MAX_COUNT                 4095          // 2^PWM_RESOLUTION - 1

#define STEERING_MIN_PW           1             // minimum pulse width, ms
#define STEERING_MAX_PW           2             // max pulse width, ms
#define STEERING_RANGE            1
#define STEERING_MAX_ANGLE        165.0
#define STEERING_MIN_ANGLE        25.0
#define STEERING_CENTER           100.0            // center (straight) steering position, degrees

#define SPEED_MIN_PW              1                 //minimum pulse width, ms
#define SPEED_MAX_PW              2                 //max pulse width, ms
#define SPEED_RANGE               1
#define SPEED_MAX_DUTY            235
#define SPEED_MIN_DUTY            350 
#define SPEED_MAX_INC             65

#define SPEED_1                   240           // low speed (duty cycle)
#define SPEED_2                   260
#define SPEED_3                   280           // medium speed
#define SPEED_4                   300
#define SPEED_5                   320           
#define SPEED_6                   350           // high speed (duty cycle)

#define THETA_TARGET              0.0
#define BT_BUFFER_SIZE            255

#define NUM_DIGITS_AFTER_DECIMAL  3
#define FLOAT_MIN_WIDTH           6
#define FLOAT_BUFF_SIZE           8
#define PACKAGE_SIZE              80
#define RAD_TO_DEG                57.29577951308232


#define HUSKYLENS_SDA_PIN         21
#define HUSKYLENS_SCL_PIN         22
#define HUSKYLENS_X_CENTER        160
#define HUSKYLENS_Y_HEIGHT        240


#define TIMER0_FREQUENCY          100000
#define TIMER0_PRESCALE           80
#define TIMER0_COUNT              50000


#define CAP_PIN                   36
#define CAP_VOLTAGE_SCALING       10
#define ADC_RESOLUTION            1024
#define ADC_MAX_VOLTAGE           3.3
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
extern int dataLog_num;
extern BluetoothSerial SerialBT;
extern HUSKYLENS huskylens;
extern Adafruit_INA219 ina219;


extern hw_timer_t *timer;
extern volatile SemaphoreHandle_t timerSemaphore;
extern portMUX_TYPE timerMux;

extern volatile uint32_t isrCounter;
extern volatile uint32_t lastIsrAt;

// FUNCTION PROTOTYPES =======================================================
// See ESP32_Vehicle.cpp for headers and details.

void initSerialMonitor();
void initINA219();
void initHUSKYLENS();
void initSteeringServo();
void initSpeedServo();
void initCapacitorPin();

void setSteeringAngle(float angle);
void setServoSpeed(float duty);
float calculateSteeringAngle();
float calculateServoSpeed();

void readINA219();       
String readGUICommand();  
void parseGUICommand();
void sendDataLog();      
HUSKYLENSResult readHUSKYLENS();

void ARDUINO_ISR_ATTR onTimer();
void init2HzTimer();


#endif      // ESP32_VEHICLE_H
