#include <Arduino.h>
#include "RPLidar.h"

#include <SoftwareSerial.h>
#define RX_PIN 21
#define TX_PIN 22
SoftwareSerial mySoftwareSerial(RX_PIN, TX_PIN); // RX, TX

// 使用硬件串口Serial1
RPLidar lidar(&Serial1);
//SoftwareSerial
//RPLidar lidar(&mySoftwareSerial);

void setup() {
  // Initialize serial communication with PC
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial); // Wait for serial port to connect
  
  // Stop scan.
  lidar.stopScan();
  // Send command to get device info
  lidar.getInfo();

  // Wait for data to become available
  delay(100); // Small delay to ensure data is received

  DeviceInfo info;
  if (lidar.parseDeviceInfo(info)) {
    // Successfully parsed device info, print it
    Serial.println("RPLidar Device Info:");
    Serial.print("Model: ");
    Serial.println(info.model);
    Serial.print("Firmware Version: ");
    Serial.print(info.firmware_major);
    Serial.print(".");
    Serial.println(info.firmware_minor);
    Serial.print("Hardware Version: ");
    Serial.println(info.hardware);
    Serial.print("Serial Number: ");
    for (int i = 0; i < 16; ++i) {
      Serial.print(info.serialnumber[i], HEX);
    }
    Serial.println();
  } else {
    // Failed to parse device info
    Serial.println("Failed to parse device info.");
  }

  HealthInfo hinfo;

  lidar.getInfo();
  delay(100); // Small delay to ensure data is received

  if(lidar.parseHealthInfo(hinfo)){
    Serial.println("RPLidar HealthInfo:");
    Serial.print("status: ");
    Serial.println(hinfo.status);
    Serial.print("Error code: ");
    Serial.println(hinfo.error_code);
    Serial.println();
    }
  Serial.println("==========");
  Serial.println("Start scan.");
  lidar.startScan();
}
// Main loop code here
void loop() { 
    lidar.parseLidarData();
}
