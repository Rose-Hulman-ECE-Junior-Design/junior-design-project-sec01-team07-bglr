/*
 * Skeleton Code for ESP32 Firmware.
 * Implements state machine for vehicle.
 * 
 * Author: CKG
 * 
 */


#include "ESP32_Vehicle.h"      //custom library

// VOID SETUP ===========================================================================
void setup() {
  initSerialMonitor();
  initHUSKYLENS();
  SerialBT.begin("CurrentHogs_ESP32_SPP_Device");       // This is the Bluetooth device name
  Serial.println("Bluetooth SPP Started. Pair your device.");
  initINA219();
  
  initSteeringServo();
  initSpeedServo();

  currentState = IDLE;


  Serial.println("FINISHED WITH SETUP.");

}



// VOID LOOP =============================================================================
void loop() {

  //check for Bluetooth Input
  parseGUICommand();
  sendDataLog();
  
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

    delay(500);


}
