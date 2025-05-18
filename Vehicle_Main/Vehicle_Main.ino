/*
 * Main loop for ESP32 Vehicle.
 * 
 * Implements state machine for vehicle.
 * 
 * Author: CKG
 * 
 */

#include "ESP32_Vehicle.h"    //custom library

unsigned long lastMicros = 0;


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

  unsigned long currentMicros = micros();     // figure out time step (could be variable due to BT logic)
  float time_step = (currentMicros - lastMicros) / 1000000.0f; // dt in seconds
  lastMicros = currentMicros;

  if (time_step < 0.0001f || time_step > 0.1f) return;  // skip if time_step is too small or too big
      
  switch (currentState){
    case IDLE:
      //turn off the motors
      break;
      
    case DRIVING:
      // Skip if dt is very small
      steeringAngle = calculateSteeringAngle(time_step);
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
