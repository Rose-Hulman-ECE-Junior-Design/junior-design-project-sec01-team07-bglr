/*
 * The purpose of this test is to show successful interfacing with the
 * HUSKYLENS camera for completion of Milestone 1.
 * 
 * Author: CKG
 */


#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

HUSKYLENS huskylens;

void setup() {
  
  Serial.begin(115200);
  initHUSKYLENS();
}


//====================================================================================
void loop() {
  // put your main code here, to run repeatedly:
    int32_t error; 
    if (!huskylens.request(ID1)) {Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));left = 0; right = 0;}
    else if(!huskylens.isLearned()) {Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));left = 0; right = 0;}
    else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
    else
    {
        HUSKYLENSResult result = huskylens.read();
        printResult(result);
    }

}


/* ==================================================================================
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
 * Print the reading from the HUSKYLENS camera to the serial monitor.
 */
void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        Serial.println("Object unknown!");
    }
}
