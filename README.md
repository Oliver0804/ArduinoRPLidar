# RPLidar Arduino Library
## Overview
This Arduino library is designed to interface with the RPLidar, a popular 360-degree laser scanner (LIDAR) sensor, particularly useful for robotics and sensing applications. Developed in C++, this library facilitates the communication between an Arduino board and the RPLidar, enabling users to easily retrieve and process LIDAR data. The library provides a straightforward way to initiate scans, read scanned data, and interpret the results for further applications, including real-time mapping and object detection.

## Features
- Initialize and configure RPLidar with customizable settings.
- Start and stop data scanning operations.
- Parse LIDAR data to extract measurement information such as quality, angle, and distance.
- Debugging capabilities to ensure seamless integration and data accuracy.
- Sample output data processing script in Python for GUI display.
## Installation
Download the library as a ZIP file.
Open the Arduino IDE, navigate to Sketch > Include Library > Add .ZIP Library, and select the downloaded file.
Once installed, you can include the library in your Arduino sketches by adding #include <RPLidar.h> at the beginning of your sketch.
## Usage
To use the library, create an RPLidar object and initialize it with the serial pins connected to the RPLidar module. Here is a basic example to start a scan and read data:

```
#include <Arduino.h>
#include "RPLidar.h"

#include <SoftwareSerial.h>
#define RX_PIN 2
#define TX_PIN 3
SoftwareSerial mySoftwareSerial(RX_PIN, TX_PIN); // RX, TX

// 使用硬件串口Serial1
RPLidar lidar(&Serial1);

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
```


## Data Output Example
Upon successful data parsing, the library outputs scanned data in the following format:
```
A:302.48 D:232.25
A:304.02 D:232.00
A:304.08 D:235.25
A:310.42 D:250.25
A:310.48 D:249.50
A:312.00 D:249.00
A:313.69 D:251.25
A:315.81 D:254.00
A:317.02 D:257.00
A:317.56 D:260.75
A:319.36 D:264.00
```

## Visualization with Python GUI
To visualize the LIDAR data, a Python script can be used to create a graphical user interface (GUI). This script reads the serial output from the Arduino and displays the LIDAR scan in real-time.


Note: Ensure the serial port and baud rate match your Arduino configuration.

This README provides a basic guide to getting started with the RPLidar Arduino library and visualizing the data. For more detailed information, refer to the library documentation and RPLidar technical manuals.