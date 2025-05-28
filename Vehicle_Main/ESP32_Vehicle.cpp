/*
 * ESP32_Vehicle Source File
 * 
 * Contains functions that will be run by the skeleton.ino file.
 * 
 *  - initialization functions
 *  - PID controller calculation functions
 *  - GUI communication functions
 * 
 * Author: CKG, LL, BB
 */
#include <SoftwareSerial.h>   //include the espsoftwareserial library
#include <QuickPID.h>
#include <Adafruit_INA219.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "BluetoothSerial.h"
#include "ESP32_Vehicle.h"



float Kp1 = DEFAULT_KP1;                // proportional angle error parameter            
float Kp2 = DEFAULT_KP2;                // proportional center error parameter
float Ki = DEFAULT_KI;
float Kd = DEFAULT_KD;

float angle_error = 0.0;
float control_output = 0.0;
float setpoint = 0.0;
float filtered_angle_error = 0.0;    // declare globally or as a static inside function
const float alpha = 0.2;              // smoothing factor (0.1 to 0.3 recommended) 

QuickPID pid(&angle_error, &control_output, &setpoint, Kp1, Ki, Kd, QuickPID::Action::direct);



float integral = 0; 
float derivative = 0;
float prev_error = 0;

char BT_buffer_incoming[BT_BUFFER_SIZE];

// Global Variables (Initalizations) =========================
float steeringAngle = 90.0;
float motorSpeed = 50;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
  
extern int current_speed = SPEED_3;       //initialize to Speed 3

VehicleState currentState = IDLE;
int dataLog_num;
BluetoothSerial SerialBT;
HUSKYLENS huskylens;
Adafruit_INA219 ina219;


// Timer Variables ========================
hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;
//==========================================

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
 * Initialize the Steering Servo.
 */
void initSteeringServo(){
  ledcAttach(STEERING_SERVO, SERVO_FREQ, PWM_RESOLUTION);
  delay(1000);
  setSteeringAngle(STEERING_CENTER);
  //ledcWrite(STEERING_SERVO, STEERING_CENTER); //initialize to 307 - 90 degrees

  Serial.println("Steering Servo initialized.");
}

/*
 * Initialize the speed servo.
 */
void initSpeedServo(){
  
  ledcAttach(SPEED_SERVO, SERVO_FREQ, PWM_RESOLUTION);
  delay(1000);
  uint32_t zeroSpeedPWMCount = 204;
  ledcWrite(SPEED_SERVO, zeroSpeedPWMCount);
}

/*
 * Intialize PID controller.
 */
void setupPID() {
    pid.SetMode(QuickPID::Control::automatic);
    pid.SetOutputLimits(-70.0f, 70.0f);  // degrees of steering angle
    pid.SetSampleTimeUs(0);  // let it run as often as you call it
    //pid.SetSampleTimeUs(PID_SAMPLE_TIME);  // 10 ms (default), adjust as needed
}


/*
 * Timer interrupt service routine 
 * Will execute every 0.5 seconds
 * Sets a flag to indicate timer has ticked.
 */
void ARDUINO_ISR_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter = isrCounter + 1;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}


/*
 * Initialize the 2 Hz timer
 * 
 * Uses Timer 0 at 1 MHz; count-up timer
 */
void init2HzTimer(){
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Set timer frequency to 1Mhz
  timer = timerBegin(TIMER0_FREQUENCY);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer, TIMER0_COUNT, true, 0);


}


/*
 * Initialize power supply capacitor analog pin for reading
 * the voltage across the capacitors.
 */
 void initCapacitorPin(){
  pinMode(CAP_PIN, INPUT);
  analogReadResolution(12);  // Optional: Default is 12-bit (0–4095)
  
 }


/*
 * Set the angle of the steering servo, in degrees.
 * 
 * Inputs - angle, as a float
 * Outputs - none
 * 
 */
void setSteeringAngle(float angle){

  if ( angle > STEERING_MAX_ANGLE || angle < STEERING_MIN_ANGLE ) {
    return;
  }
  
  float pw = STEERING_MIN_PW + ( (angle/180.0) * STEERING_RANGE);
  uint32_t duty = (uint32_t) (MAX_COUNT * (pw / SERVO_PERIOD));
  //Serial.print("Steering servo set to "); Serial.print(angle); Serial.print(", duty "); Serial.println(duty); 
  
  ledcWrite(STEERING_SERVO, duty);
}

/*
 * Set the speed of the speed servo, as an angle
 * 
 * Inputs - angle, as a float
 * Outputs - none
 * 
 * Speed will only be ever set to one of 6 known duty cycles
 */
void setServoSpeed(uint32_t duty){
  ledcWrite(SPEED_SERVO, duty);
}

/*
 * Read the HUSKYLENS camera. Make all appropriate checks.
 * 
 * Inputs - none
 * Outputs - HUSKYLENSResult type, aka the arrow
 */
HUSKYLENSResult readHUSKYLENS(){
    HUSKYLENSResult result;
    if (!huskylens.request(1)) {Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));}
    else if(!huskylens.isLearned()) {Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));}
    else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
    else
    {
        result = huskylens.read();
        //Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }

    return result;
  
}

/*
 * Use a P control algorithm to calculate the correct steering angle
 * 
 * Inputs - none
 * Outputs - target steering angle, in degrees, as a float
 */
float calculateSteeringAngle(){
    HUSKYLENSResult result = readHUSKYLENS();

    if (result.command != COMMAND_RETURN_ARROW) {
        // No valid line
        pid.SetMode(QuickPID::Control::manual);  // disable PID temporarily
        control_output = 0.0f;
        return steeringAngle;
    }

    // Re-enable PID if it was off
    if (pid.GetMode() != (uint8_t)QuickPID::Control::automatic) {
        pid.SetMode(QuickPID::Control::automatic);
    }

    // Compute angle error
    float dy = (float)(result.yTarget - result.yOrigin);
    if (dy == 0) return control_output;  // let's not divide-by-zero

    float r = (float)(result.xTarget - result.xOrigin) / dy;
    angle_error = THETA_TARGET + atan(r) * RAD_TO_DEG;

    // Angle Error LPF
    static float filtered_angle_error = 0.0;
    const float alpha = 0.2;
    filtered_angle_error = alpha * angle_error + (1.0 - alpha) * filtered_angle_error;
    angle_error = filtered_angle_error;

    // Centerline error (optional)
    float center_error = ((float)(result.xOrigin + result.xTarget)) / 2.0f - HUSKYLENS_X_CENTER;

    // Center Error LPF
    static float filtered_center_error = 0.0f;
    filtered_center_error = alpha * center_error + (1.0f - alpha) * filtered_center_error;
    center_error = filtered_center_error;
    
    // Run PID computation
    pid.Compute();  // internally uses micros() for timing
                    // loads result into control_output

    // Apply center bias
    float final_output = control_output - (Kp2 * center_error);

    return final_output;
}


/*
 * Read data from the INA219 into the global variables.
 * Will be called every time ESP32 Vehicle wants to package
 * and send a data log to the GUI.
 * 
 * Inputs - none
 * Outputs - none
 * 
 * Updates globals.
 */
void readINA219(){
   shuntvoltage = ina219.getShuntVoltage_mV();
   busvoltage = ina219.getBusVoltage_V();
   current_mA = ina219.getCurrent_mA();
   power_mW = ina219.getPower_mW();
   loadvoltage = busvoltage + (shuntvoltage / 1000);
}


/*
 * Read data from Bluetooth link.
 * 
 * Inputs - none
 * Outputs - serial data read, as a String
 */
String readGUICommand(){

  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    //Serial.println("Received via BT: " + incoming);

    return incoming;
    
  }

  return "";  //return an empty string if not available
  
}   


/*
 * Parse an incoming GUI command. Will be called repeatedly
 * in the main loop of main.ino so the system may
 * always be checking for updates/commands from the GUI.
 * 
 * This function updates the global states. 
 * 
 * Inputs - none
 * Outputs - none
 */
void parseGUICommand(){
  //interpret the GUI command
  String command = readGUICommand();
  
  if (command.length() == 0){  //no command was received, return
    return;
  }
  
  Serial.print("Received: "); Serial.println(command);

  //interpret each command, change state if necessary
  if (command.equals("Start")){
      currentState = DRIVING;
      setServoSpeed(current_speed);
      //pid.SetTunings(Kp1, Ki, Kd);
          
   }else if (command.equals("Stop")){
      currentState = IDLE;
      ledcWrite(SPEED_SERVO, SPEED_STOP);
      setSteeringAngle(STEERING_CENTER);
      pid.SetMode(QuickPID::Control::manual);  // disable PID temporarily
  
   } else if (command.equals("Recharge")){
      currentState = RECHARGING;
      setSteeringAngle(STEERING_CENTER);
      ledcWrite(SPEED_SERVO, SPEED_STOP);
      pid.SetMode(QuickPID::Control::manual);  // disable PID temporarily
      //Serial.println("Vehicle is now in RECHARGING state.");
    
   } else if (command.equals("S1")){
       //Serial.print("Speed set to "); Serial.println(SPEED_1);
       currentState = DRIVING;
       current_speed = SPEED_1;
//       Kp1 = DEFAULT_KP1;
//       Kp2 = DEFAULT_KP2;
//       Ki = DEFAULT_KI;
//       Kd = DEFAULT_KD;
       setServoSpeed(current_speed);
    
   }else if (command.equals("S2")){
       //Serial.print("Speed set to "); Serial.println(SPEED_2);
       currentState = DRIVING;
       current_speed = SPEED_2;
//       Kp1 = DEFAULT_KP1;
//       Kp2 = DEFAULT_KP2;
//       Ki = DEFAULT_KI;
//       Kd = DEFAULT_KD;
       setServoSpeed(current_speed);
    
   }else if (command.equals("S3")){
       Serial.print("Speed set to "); Serial.println(SPEED_3);
       currentState = DRIVING;
       current_speed = SPEED_3;
//       Kp1 = DEFAULT_KP1;
//       Kp2 = DEFAULT_KP2;
//       Ki = DEFAULT_KI;
//       Kd = DEFAULT_KD;     
       setServoSpeed(current_speed);
    
   }else if (command.equals("S4")){
      //Serial.print("Speed set to "); Serial.println(SPEED_4);
      currentState = DRIVING;
      current_speed = SPEED_4;
//       Kp1 = DEFAULT_KP1;
//       Kp2 = DEFAULT_KP2;
//       Ki = DEFAULT_KI;
//       Kd = DEFAULT_KD;
      setServoSpeed(current_speed);
    
   }else if (command.equals("RECHARGE")){
      
      currentState = RECHARGING;
      ledcWrite(SPEED_SERVO, SPEED_STOP);
          
   } else if (command.startsWith("Kp1=")){
      String newVal = command.substring(4);   //strip out the first 4 chars
      Kp1 = newVal.toFloat();
      pid.SetTunings(Kp1, Ki, Kd);
      //Serial.print("Kp1 set to "); Serial.println(Kp1);
    
    
   } else if (command.startsWith("Kp2=")){
      String newVal = command.substring(4);   //strip out the first 4 chars
      Kp1 = newVal.toFloat();
      pid.SetTunings(Kp1, Ki, Kd);
      //Serial.print("Kp1 set to "); Serial.println(Kp1);
    
   } else if (command.startsWith("Ki=")){

      String newVal = command.substring(3);   //strip out the first 3 chars
      Ki = newVal.toFloat();
      pid.SetTunings(Kp1, Ki, Kd);
      //Serial.print("Ki set to "); Serial.println(Ki);
      
   } else if (command.startsWith("Kd=")){

      String newVal = command.substring(3);   //strip out the first 3 chars
      Kd = newVal.toFloat();
      pid.SetTunings(Kp1, Ki, Kd);
      //Serial.print("Kd set to "); Serial.println(Kd);
      
   }else {
    //Serial.println("Command could not be parsed."); 
   }
   
}

/*
 * Takes a float and converts to a string.
 * Replaces spaces with zeroes so the number of bytes
 * sent is always consistent.
 */
void floatToZeroPaddedStr(float val, int width, int precision, char* out) {
  dtostrf(val, width, precision, out);
  for (int i = 0; out[i] == ' '; i++) {
    out[i] = '0';
  }
}

/*
 * Send data log to the GUI via Bluetooth link.
 * 
 * Reads vehicle data and packages it into a string packet
 * which is sent to the Controller GUI via the serial Bluetooth
 * link. 
 * 
 * Inputs - none
 * Outputs - none
 */
void sendDataLog(){

  //update sensor readings
  readINA219();  //read INA219
  //read capacitor voltage
  float cap_voltage = analogReadMilliVolts(CAP_PIN) / 1000.0;  // Must be an ADC1 pin

  char current[FLOAT_BUFF_SIZE], voltage[FLOAT_BUFF_SIZE], state[FLOAT_BUFF_SIZE], v_cap[FLOAT_BUFF_SIZE];
  //convert INA219 values from floats to strings - ZERO PADDED SO THEY ARE ALWAYS THE SAME LENGTH!!!
  floatToZeroPaddedStr(current_mA, FLOAT_MIN_WIDTH, NUM_DIGITS_AFTER_DECIMAL, current);
  floatToZeroPaddedStr(loadvoltage, FLOAT_MIN_WIDTH, NUM_DIGITS_AFTER_DECIMAL, voltage);
  floatToZeroPaddedStr(cap_voltage, FLOAT_MIN_WIDTH, NUM_DIGITS_AFTER_DECIMAL, v_cap);

  char stateChar;
  switch (currentState){
  case IDLE:  
    stateChar = '0';
    break;
  case DRIVING:
    stateChar = '1';
    break;
  case RECHARGING:
    stateChar = '2';
    break;
  default:
    stateChar = '0';
    break;
  }


  //concatenate serial package of data, do some string manipulation
  char package[FLOAT_BUFF_SIZE*3 + 3 + 1 + 1]; // Adjust size as needed
  sprintf(package, "%s:%s:%c:%s\n", current, voltage, stateChar, v_cap);

  //Serial.println(package);
  SerialBT.print(package); 

}      
