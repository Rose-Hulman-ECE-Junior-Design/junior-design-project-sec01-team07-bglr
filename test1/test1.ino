/*
 * The purpose of this program is to demonstrate use of the steering servo, 
 * wheels, and INA219 device for completion of Milestone 1.
 * 
 * Author: CK Gallegos Rodriguez
 */

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <SoftwareSerial.h>   //include the espsoftwareserial library
#include "HUSKYLENS.h"        //include the HUSKYLENS library

 
Adafruit_INA219 ina219;


#define SERIAL_MONITOR_BAUD 115200


#define STEERING_SERVO 32
#define SPEED_SERVO 33

#define SERVO_FREQ 50                             // 50 Hz, 
#define SERVO_PERIOD (1/SERVO_FREQ)*1000          // Period = 20ms
#define PWM_RESOLUTION 12                         //12-bit resolution
#define MAX_COUNT 4095                            //2^PWM_RESOLUTION - 1

#define STEERING_MIN_PW  1               //minimum pulse width, ms
#define STEERING_MAX_PW  2               //max pulse width, ms
#define STEERING_RANGE  (STEERING_MAX_PW - STEERING_MIN_PW)

#define SPEED_MIN_PW  1                 //minimum pulse width, ms
#define SPEED_MAX_PW  2                 //max pulse width, ms
#define SPEED_RANGE  (SPEED_MAX_PW - SPEED_MIN_PW)



// ========================================================================================
void setup(void) 
{
  initSerialMonitor();
  initINA219();
  initSteeringServo();
  initSpeedServo();
}


// ========================================================================================
void loop(void) 
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  for (int i=2; i < 16; i++){
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
    
    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
    Serial.println("");

    setSteeringAngle(i*10); //will set the steering angle to 20 deg, 30 deg, 40 deg, ..., 150 deg, 160 deg
    setServoSpeed(90);      //will set the speed to 50% approx

    delay(2000);
  }

}


/* ========================================================================================
 * Initialize the serial monitor.
 */
void initSerialMonitor(void){
  Serial.begin(SERIAL_MONITOR_BAUD);         //fire up the serial monitor
  while (!Serial) {
      delay(1);
  }
  Serial.println("Hello!");
}

/* =========================================================================================
 * Initialize the INA219 device.
 */
void initINA219(void){
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with INA219 ...");
}

/* =========================================================================================
 * Initialize the Steering Servo
 */
void initSteeringServo(){
  ledcAttach(STEERING_SERVO, SERVO_FREQ, PWM_RESOLUTION);
  delay(1000);
  ledcWrite(STEERING_SERVO, 307); //initialize to 307 - 90 degrees

  Serial.println("Steering Servo initialized to 90 deg.");
}


/*
 * Initialize the speed servo
 */
void initSpeedServo(){
  
  ledcAttach(SPEED_SERVO, SERVO_FREQ, PWM_RESOLUTION);
  delay(1000);
  uint32_t zeroSpeedPWMCount = 204;
  ledcWrite(SPEED_SERVO, zeroSpeedPWMCount);
  delay(5000);
 }


/*
 * Set the angle of the steering servo, in degrees.
 */
void setSteeringAngle(uint32_t angle){

  uint32_t pw = STEERING_MIN_PW + [(angle/180) * STEERING_RANGE];
  uint32_t duty = MAX_COUNT * (pw / SERVO_PERIOD);
  Serial.println("Steering servo set to %d degrees, duty %d. " angle, duty);
  
  ledcWrite(STEERING_SERVO, duty);
}


/*
 * Set the speed of the speed servo, as a percentage of total speed.
 */
void setServoSpeed(uint32_t angle){

  uint32_t pw = SPEED_MIN_PW + [(angle/180) * SPEED_RANGE];
  uint32_t duty = MAX_COUNT * (pw / SERVO_PERIOD);
  Serial.println("Steering servo set to %d degrees, duty %d. " angle, duty);

  ledcWrite(SPEED_SERVO, duty);
}
