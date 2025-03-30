/*
 * ESP32_Vehicle Source File
 * 
 * Contains several important functions:
 * 
 *  - init functions
 *  - simple calculation functions
 *  - other stuff
 * 
 */

#include "ESP32_Vehicle.h"


float Kp, Kd, Ki;
float dt, integral, derivative;
float prev_error;


/* 
 * Initialize the serial monitor.
 */
void initSerialMonitor(void){
  Serial.begin(SERIAL_BAUD);         //fire up the serial monitor
  while (!Serial) {
      delay(1);
  }
  Serial.println("Hello! Serial Monitor is up and running. ");
  delay(1000);
}

/*
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


/* 
 * Initialize the HUSKYLENS camera.
 */
void initHUSKYLENS(){
  Wire.begin();
  while (!huskylens.begin(Wire)){
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
  }
  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING); //Switch the algorithm to line tracking.
}

/* 
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
void setSteeringAngle(float angle){

  float pw = STEERING_MIN_PW + ( (angle/180.0) * STEERING_RANGE);
  uint32_t duty = (uint32_t) (MAX_COUNT * (pw / SERVO_PERIOD));
  Serial.print("Steering servo set to "); Serial.print(angle); Serial.print(", duty "); Serial.println(duty); 
  
  ledcWrite(STEERING_SERVO, duty);
}

/*
 * Set the speed of the speed servo, as a percentage of total speed.
 */
void setServoSpeed(float angle){

  float pw = SPEED_MIN_PW + ( (angle/180.0) * SPEED_RANGE);   //pulse width, in ms
  uint32_t duty = (uint32_t) (MAX_COUNT * (pw / SERVO_PERIOD));
  Serial.print("Speed servo set to duty "); Serial.println(duty); 

  ledcWrite(SPEED_SERVO, duty);
}


/*
 * Reads values from INA219 to the global variables. 
 */
void readINA219(){
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
}


/*
 * Uses the PID control algorithm to calculate the correct steering angle
 */
float calculateSteeringAngle(){
   HUSKYLENSResult result = huskylens.read()
   float error = tan((result.xTarget - result.xOrigin) / (result.yTarget - yOrigin));

   float P = Kp * error;
   integral += error * dt;
   float I = Ki * integral;

   derivative = (error - prev_error) / dt;
   float D = Kd * derivative;

   prev_error = error;
   
   return P + I + D;
  
}

/*
 * Uses PID control algorithm to calculate correct speed
 */
float calculateServoSpeed(){
  
}

/*
 * Read data from the INA219 into the global variables
 */
void readINA219(){
   shuntvoltage = ina219.getShuntVoltage_mV();
   busvoltage = ina219.getBusVoltage_V();
   current_mA = ina219.getCurrent_mA();
   power_mW = ina219.getPower_mW();
   loadvoltage = busvoltage + (shuntvoltage / 1000);
}


/*
 * Read GUI from Bluetooth link
 */
int readGUICommand(){

  //if there is no input to be read, return 0
  //otherwise, return the message code
  
}   


/*
 * Parse GUI command
 */
void parseGUICommand(){
  //interpret the GUI command

  //could change the global state here...
}


/*
 * Send data log to the GUI via Bluetooth link
 */
void sendDataLog(){

  readINA219();
  //package up the data
  //send it out
  
}      
