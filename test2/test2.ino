/*
 * The purpose of this test is to demonstrate the use of Bluetooth communication
 * between the user laptop and the ESP32 board for completion of Milestone 1.
 * 
 * Author: CKG
 */

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define SERIAL_BAUD 115200


void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_BAUD);
  SerialBT.begin("ESP32_SPP_Device");       // This is the Bluetooth device name
  Serial.println("Bluetooth SPP Started. Pair your device.");

}

void loop() {
  // Receive data from the laptop via Bluetooth
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    Serial.println("Received via BT: " + incoming);
    // Optional: Echo back to laptop
    SerialBT.println("Echo: " + incoming);
  }

  // Send data from Serial Monitor to Bluetooth
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }

}

/*
 * Power up the ESP32 (with the code above).
Open your Bluetooth settings on your laptop.
Search for new devices → You should see "ESP32_SPP_Device".
Pair it (PIN might be 1234 or 0000 if it asks).
After pairing, a Bluetooth Serial Port (COM port) will be assigned:
Windows: Check Device Manager > Ports (COM & LPT)
 */
