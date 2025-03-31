/*
 * Skeleton Code for ESP32 Firmware.
 * Implements state machine for vehicle.
 * 
 * Author: CKG
 * 
 */

#pragma once
#include <SoftwareSerial.h>   //include the espsoftwareserial library
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "BluetoothSerial.h"


#include "ESP32_Vehicle.h"      //custom library

// VOID SETUP ===========================================================================
void setup() {
  initSerialMonitor();
  SerialBT.begin("CurrentHogs_ESP32_SPP_Device");       // This is the Bluetooth device name
  Serial.println("Bluetooth SPP Started. Pair your device.");
  initINA219();
  initHUSKYLENS();
  initSteeringServo();
  initSpeedServo();

  currentState = IDLE;

}



// VOID LOOP =============================================================================
void loop() {

  //check for Bluetooth Input
  readGUICommand();
  sendDataLog();
  
  switch (currentState){
    case IDLE:
      Serial.println("Vehicle is now in IDLE state.");
      //turn off the 
      break;
      
    case DRIVING:
      Serial.println("Vehicle is now in DRIVING state.");
      setSteeringAngle(calculateSteeringAngle());
      setServoSpeed(calculateServoSpeed());
      //stuff
      break;
      
    case RECHARGING:
      Serial.println("Vehicle is now in RECHARGING state.");
      //stuff
      break;
      
    default:
      Serial.println("Vehicle is now in DEFAULT state.");
      //more stuff
      break;
    }

    delay(5);
 

}
