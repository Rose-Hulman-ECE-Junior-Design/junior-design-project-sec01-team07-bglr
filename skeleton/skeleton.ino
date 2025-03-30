/*
 * SKeleton Code for ESP32 Firmware
 * 
 * Author: CKG
 * 
 */


#include <SoftwareSerial.h>   //include the espsoftwareserial library
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "BluetoothSerial.h"


#include "ESP32_Vehicle.h"      //custom library

//BluetoothSerial SerialBT;
//HUSKYLENS huskylens;
//Adafruit_INA219 ina219;


// GLOBAL VARIABLES =====================================================

//float steeringAngle = 90.0; 
//float  motorSpeed = 50; 
//float shuntvoltage = 0; 
//float busvoltage = 0; 
//float current_mA = 0; 
//float loadvoltage = 0; 
//float power_mW = 0; 

//enum VehicleState{ 
//  IDLE, 
//  DRIVING, 
//  RECHARGING 
//} 

//VehicleState currentState; 
 



// VOID SETUP ===========================================================================
void setup() {
  initSerialMonitor();
  SerialBT.begin("CurrentHogs_ESP32_SPP_Device");       // This is the Bluetooth device name
  Serial.println("Bluetooth SPP Started. Pair your device.");
  initINA219();
  initHUSKYLENS();
  initSteeringServo();
  initSpeedServo();

}



// VOID LOOP =============================================================================
void loop() {

  //check for Bluetooth Input

  switch (var){
    case IDLE:
      //stuff
      break;
      
    case DRIVING:
      //stuff
      break;
      
    case RECHARGING:
      //stuff
      break;
      
    default:
      //more stuff
      break;
    }

}
