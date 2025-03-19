#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

#include <SoftwareSerial.h>   //include the espsoftwareserial library
#include "HUSKYLENS.h"        //include the HUSKYLENS library


//Silly Goofy program to run the servos and motor

#define STEERING 32
#define REAR_WHEELS 33

#define SERVO_FREQ 50
#define PWM_RESOLUTION 12
#define max_count 4095


void setup(void) 
{
  Serial.begin(115200);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }
    
  Serial.println("Hello!");
  
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


  ledcAttach(STEERING, SERVO_FREQ, PWM_RESOLUTION);
  delay(1000);
  ledcWrite(STEERING, 307); //initialize to 307 - 90 degrees

  //Initialize the speed servo
  ledcAttach(REAR_WHEELS, SERVO_FREQ, PWM_RESOLUTION);
  delay(1000);
  uint32_t zeroSpeedPWMCount = 204;
  ledcWrite(REAR_WHEELS, zeroSpeedPWMCount);
  delay(5000);

  
}

void loop(void) 
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

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


  // put your main code here, to run repeatedly:
  //uint32_t halfSpeedPWMCount = 307;
  ledcWrite(REAR_WHEELS, 300);
  //ledcWrite(STEERING, 307); //initialize to 307 - 90 degrees
  

  delay(2000);
}
