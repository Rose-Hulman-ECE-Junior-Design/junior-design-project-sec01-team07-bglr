/*
 * Main loop for ESP32 Vehicle.
 * 
 * Implements state machine for vehicle.
 * 
 * Author: CKG
 * 
 */

#include "ESP32_Vehicle_Copy.h"    //custom library


// VOID SETUP ===========================================================================
void setup() {
  //initialize all devices
  initSerialMonitor();
  initHUSKYLENS();
  SerialBT.begin("CurrentHogs_ESP32");       // This is the Bluetooth device name
  Serial.println("Bluetooth SPP Started. Pair your device.");
  initINA219();
  initSteeringServo();
  initSpeedServo();
  init2HzTimer();
  initCapacitorPin();
  setupPID();

  currentState = IDLE;      //initial startup state
  dataLog_num = 0;
  
  Serial.println("FINISHED WITH SETUP.");

}



// VOID LOOP =============================================================================
void loop() {

  //check for Bluetooth Input
  parseGUICommand();

  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {    
    sendDataLog();
  }

      
  switch (currentState){
    case IDLE:
      //turn off the motors
      break;
      
    case DRIVING:
      steeringAngle = calculateSteeringAngle();
      //Serial.print("Steering angle: "); Serial.println(steeringAngle);
      setSteeringAngle(STEERING_CENTER + steeringAngle);

      break;
    case RECHARGING:
      //enter sleeping state to save power
      break;
      
    default:
      Serial.println("Vehicle is now in DEFAULT state.");
      //more stuff
      break;
    }

}
