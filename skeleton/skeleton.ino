/*
 * Skeleton Code for ESP32 Firmware.
 * 
 * Implements state machine for vehicle.
 * 
 * Author: CKG
 * 
 */

#include "ESP32_Vehicle.h"    //custom library

// VOID SETUP ===========================================================================
void setup() {
  //initialize all devices
  initSerialMonitor();
  initHUSKYLENS();
  SerialBT.begin("CurrentHogs_ESP32_SPP_Device");       // This is the Bluetooth device name
  Serial.println("Bluetooth SPP Started. Pair your device.");
  initINA219();
  initSteeringServo();
  initSpeedServo();
  init2HzTimer();
  initCapacitorPin();

  currentState = IDLE;
  dataLog_num = 0;
  
  Serial.println("FINISHED WITH SETUP.");
  SerialBT.println("CONNECT ME MOTHERFUCKER");


}



// VOID LOOP =============================================================================
void loop() {

  //check for Bluetooth Input
  parseGUICommand();

//  if (timerFlag) { // Every 0.5 seconds
//    portENTER_CRITICAL(&timerMux);
//    timerFlag = false;
//    portEXIT_CRITICAL(&timerMux);
//
//    Serial.print("Sending data log number "); Serial.println(dataLog_num);
//    sendDataLog(); //Send the data log
//  }

  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
//    uint32_t isrCount = 0, isrTime = 0;
//    // Read the interrupt count and time
//    portENTER_CRITICAL(&timerMux);
//    isrCount = isrCounter;
//    isrTime = lastIsrAt;
//    portEXIT_CRITICAL(&timerMux);
//    // Print it
//    Serial.print("onTimer no. ");
//    Serial.print(isrCount);
//    Serial.print(" at ");
//    Serial.print(isrTime);
//    Serial.println(" ms");
    
    sendDataLog();
  }
  
  switch (currentState){
    case IDLE:
      //turn off the 
      ledcWrite(SPEED_SERVO, 0);
      break;
      
    case DRIVING:
      
      steeringAngle = calculateSteeringAngle();
      setSteeringAngle(STEERING_CENTER + steeringAngle);
      ledcWrite(SPEED_SERVO, 250);
      //stuff
      break;
      
    case RECHARGING:
      //stuff
      break;
      
    default:
      Serial.println("Vehicle is now in DEFAULT state.");
      //more stuff
      break;
    }

    //delay(500);


}
