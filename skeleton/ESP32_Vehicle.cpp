/*
 * ESP32_Vehicle Source File
 * 
 * Contains several important functions:
 * 
 *  - init functions
 *  - PID controller calculation functions
 *  - GUI communication functions
 * 
 * Author: CKG, LL
 */
#include <SoftwareSerial.h>   //include the espsoftwareserial library
//#include <Wire.h>
#include <Adafruit_INA219.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "BluetoothSerial.h"
#include "ESP32_Vehicle.h"


float Kp = 0.65;
//float Kd, Ki = 1;
//float dt, integral, derivative = 2;
//float prev_error = 0;

char BT_buffer_incoming[BT_BUFFER_SIZE];

// Global Variables (Initalizations) =========================
float steeringAngle = 90.0;
float motorSpeed = 50;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;

VehicleState currentState = IDLE;

BluetoothSerial SerialBT;
HUSKYLENS huskylens;
Adafruit_INA219 ina219;
//Serial;


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
  Wire.begin(HUSKYLENS_SDA_PIN, HUSKYLENS_SCL_PIN);
  while (!huskylens.begin(Wire)){
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
  }
  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING); //Switch the algorithm to line tracking.
  Serial.println("Initialized HUSKYLENS. ===================");
  delay(100);
  
  
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
 * Read the HUSKYLENS camera
 */
HUSKYLENSResult readHUSKYLENS(){
    HUSKYLENSResult result;
    if (!huskylens.request(1)) {Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));}
    else if(!huskylens.isLearned()) {Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));}
    else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
    else
    {
        result = huskylens.read();
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
        //printResult(result);
    }

    return result;
  
}

/*
 * Uses the PID control algorithm to calculate the correct steering angle
 */
float calculateSteeringAngle(){
    
    HUSKYLENSResult result = readHUSKYLENS();
    
    if (result.command != COMMAND_RETURN_ARROW){
       Serial.println("Object unknown!");
       return steeringAngle;
    }

   
   //check if this returned an arrow
   if ((result.yTarget - result.yOrigin) == 0){
       Serial.println("Did not get a valid arrow. ");
       return steeringAngle;
    }

   
   float error = THETA_TARGET - ( atan((result.xTarget - result.xOrigin) / (result.yTarget - result.yOrigin)) ) * RAD_TO_DEG;
   Serial.print("Error angle (deg)"); Serial.println(error);

//   float P = Kp * error;
//   integral += error * dt;
//   float I = Ki * integral;

//   derivative = (error - prev_error) / dt;
//   float D = Kd * derivative;

//   prev_error = error;
   
   return Kp * error;

   //TODO:
   //make sure the returned steering angle does not exceed servo range
  
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

  if (SerialBT.available()) {

    //fill up a byte buffer rather than getting a string 
    String incoming = SerialBT.readStringUntil('\n');
    Serial.println("Received via BT: " + incoming);

    return 1;
    
  }

  //if there is no input to be read, return 0
  //otherwise, return the message code
  return 0;
  
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

  //update sensor readings
  readINA219();

  char current[FLOAT_BUFF_SIZE], voltage[FLOAT_BUFF_SIZE], power[FLOAT_BUFF_SIZE], state[FLOAT_BUFF_SIZE];
  
  dtostrf(current_mA, FLOAT_MIN_WIDTH, NUM_DIGITS_AFTER_DECIMAL, current);           //convert INA219 floats to strings
  dtostrf(loadvoltage, FLOAT_MIN_WIDTH, NUM_DIGITS_AFTER_DECIMAL, voltage);
  dtostrf(power_mW, FLOAT_MIN_WIDTH, NUM_DIGITS_AFTER_DECIMAL, power);
  dtostrf((float)currentState, FLOAT_MIN_WIDTH, NUM_DIGITS_AFTER_DECIMAL, state);

  //strip out the null terminator character from each??
  //build up serial package of data, do some string manipulation
  char package[PACKAGE_SIZE]; // Adjust size as needed
  sprintf(package, "I:%s V:%s P:%s S:%s ", current, voltage, power, state);
  
  //String package = "I:" + current + " V:" + voltage + " P:" + power + " S:" + state;
  Serial.println(package);
  SerialBT.println(package); //
  //SerialBT.write(package); //send raw bytes of data through BT Link

}      
