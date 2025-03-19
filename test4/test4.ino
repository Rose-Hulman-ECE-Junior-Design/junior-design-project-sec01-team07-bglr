/*
 * The purpose of this test file is to begin integrating the
 * Bluetooth interface and collecting the INA219 sensor and
 * HUSKYLENS information. 
 */

#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
HUSKYLENS huskylens;
BluetoothSerial SerialBT;

int ID1 = 1;

#define SERIAL_BAUD 115200

void setup() {

  Serial.begin(SERIAL_BAUD);
  SerialBT.begin("CurrentHogs_ESP32_SPP_Device");       //This is the Bluetooth device name
  Serial.println("Bluetooth SPP Started. Pair your device.");
  initHUSKYLENS();
}

void loop() {

  // Receive data from the laptop via Bluetooth
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    Serial.println("Received via BT: " + incoming);
    // Optional: Echo back to laptop
    SerialBT.println("Echo: " + incoming);
  }

  int32_t error; 

  if (!huskylens.request(ID1)) {Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));}
  else if(!huskylens.isLearned()) {Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));}
  else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
  else{
    HUSKYLENSResult result = huskylens.read();
    exportHUSKYLENSResult(result);   //send the camera results to the laptop via Bluetooth

    
  }

  exportINA219Result();               //send the INA219 results to the laptop via Bluetooth


}



/* ==================================================================================
 * Initialize the HUSKYLENS camera.
 */
void initHUSKYLENS(){
  Wire.begin();
  delay(500);
  while (!huskylens.begin(Wire)){
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
  }
  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING); //Switch the algorithm to line tracking.
}


void exportHUSKYLENSResult(HUSKYLENSResult result){
    SerialBT.println("Here's the data from the HUSKYLENS! ");
    if (result.command == COMMAND_RETURN_BLOCK){
        SerialBT.write((uint8_t)result.xCenter);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        SerialBT.write((uint8_t)result.xOrigin);
    }
    else{
        SerialBT.println("Object unknown!");
    }
}


void exportINA219Result(){
  float current_mA = 0;
  float power_mW = 0;

  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();

  SerialBT.println("Here's the data from the INA219! ");
  SerialBT.write((uint8_t)current_mA);
  SerialBT.write((uint8_t)power_mW);

}
