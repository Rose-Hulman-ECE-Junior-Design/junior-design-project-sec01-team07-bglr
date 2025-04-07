/*
 * Skeleton Code for ESP32 Firmware.
 * Implements state machine for vehicle.
 * 
 * Author: CKG
 * 
 */

//#pragma once
#include "HUSKYLENS.h"
#include <SoftwareSerial.h>   //include the espsoftwareserial library
//#include <Wire.h>
#include <Adafruit_INA219.h>

#include "SoftwareSerial.h"
#include "BluetoothSerial.h"


#include "ESP32_Vehicle.h"      //custom library

// VOID SETUP ===========================================================================
void setup() {
  initSerialMonitor();
  initHUSKYLENS();
  //SerialBT.begin("CurrentHogs_ESP32_SPP_Device");       // This is the Bluetooth device name
  //Serial.println("Bluetooth SPP Started. Pair your device.");
  //initINA219();
  
  //initSteeringServo();
  //initSpeedServo();

  currentState = DRIVING;


  Serial.println("FINISHED WITH SETUP.");

}



// VOID LOOP =============================================================================
void loop() {

  //check for Bluetooth Input
//  readGUICommand();
//  sendDataLog();

//  steeringAngle = calculateSteeringAngle();
//  setSteeringAngle(steeringAngle);
//  ledcWrite(SPEED_SERVO, 270);


HUSKYLENSResult result = huskylens.read();
Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);


  delay(100);
  
/*  switch (currentState){
    case IDLE:
      Serial.println("Vehicle is now in IDLE state.");
      //turn off the 
      break;
      
    case DRIVING:
      Serial.println("Vehicle is now in DRIVING state.");
      setSteeringAngle(calculateSteeringAngle());
      setServoSpeed(90.0);
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

    delay(500);
 */

}
