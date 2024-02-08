#include "Arduino.h"
#include "RPLidar.h"

RPLidar::RPLidar(Stream *stream) : _lidarSerial(stream) {
}

void RPLidar::begin(long baudRate) {
}

void RPLidar::sendCommand(byte cmd, byte* payload, unsigned int payloadSize) {
  byte header[2] = {RPLIDAR_CMD_SYNC_BYTE, cmd};
  _lidarSerial->write(header, 2);
  if (payload && payloadSize) {
    _lidarSerial->write(payload, payloadSize);
  }
}

//以廢棄
/*
u_result RPLidar::readResponse() {
  const _u32 timeout = 200; // 超時時間設置為 200 毫秒
  const _u32 startTs = millis(); // 記錄開始讀取數據的初始時間
  _u32 elapsed = 0; // 經過時間
  rplidar_response_measurement_node_t node; // 用於存儲解析後的數據
  _u8 *nodebuf = (_u8*)&node; // 指向 node 的指針，便於按字節操作

  _u8 recvPos = 0; // 當前接收到的字節位置

  // 持續讀取數據，直到超時或完成一個測量節點的讀取
  while ((elapsed = millis() - startTs) < timeout) {
    if (_lidarSerial->available() > 0) {
      int currentByte = _lidarSerial->read(); // 從串口讀取一個字節
      if (currentByte < 0) continue; // 如果沒有讀取到數據，則繼續等待

      // 根據接收到的字節位置處理數據
      switch (recvPos) {
        case 0:
          // 檢查是否為測量節點的開始字節
          if ((currentByte >> 1) ^ currentByte & 0x1) {
            nodebuf[recvPos++] = currentByte;
          }
          break;
        case 1:
          // 確保數據包的校驗位是正確的
          if (currentByte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
            nodebuf[recvPos++] = currentByte;
          } else {
            // 如果校驗位不正確，重置接收位置，從新開始
            recvPos = 0;
          }
          break;
        default:
          // 存儲後續字節
          nodebuf[recvPos++] = currentByte;
          if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
            // 如果已接收完一個完整的數據節點
            _currentMeasurement.distance = node.distance_q2 / 4.0f; // 轉換距離
            _currentMeasurement.angle = (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f; // 轉換角度
            _currentMeasurement.quality = node.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT; // 獲取品質
            _currentMeasurement.startBit = node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT; // 獲取起始位
            return RESULT_OK; // 成功讀取一個測量節點
          }
      }
    }
  }

  // 如果在超時時間內沒有完成一個測量節點的讀取，則返回超時結果
  return RESULT_OPERATION_TIMEOUT;
}*/



void RPLidar::startScan() {
  sendCommand(0x20, nullptr, 0); // Send the start scan command
  // Here you might need to read and process the response data
}

void RPLidar::stopScan() {
  sendCommand(0x25, nullptr, 0); // Send the stop scan command
  // Processing of the response can be done here if necessary
}

void RPLidar::getInfo() {
  sendCommand(0x50, nullptr, 0); // Send the get info command
  // Read and parse the device info here
}

void RPLidar::getHealth() {
  sendCommand(0x52, nullptr, 0); // Send the get health command
  // Read and parse the health info here
}


bool RPLidar::parseDeviceInfo(DeviceInfo& info) {
  // Assuming the command has already been sent and the response is ready
  if (_lidarSerial->available() < 20) {
    // Not enough data available yet
    return false;
  }

  // Read descriptor and data payload
  byte descriptor[7];
  for (int i = 0; i < 7; ++i) {
    descriptor[i] = _lidarSerial->read();
  }

  // Validate descriptor here (not implemented for brevity)

  // Read the actual info data
  info.model = _lidarSerial->read();
  info.firmware_minor = _lidarSerial->read();
  info.firmware_major = _lidarSerial->read();
  info.hardware = _lidarSerial->read();
  for (int i = 0; i < 16; ++i) {
    info.serialnumber[i] = _lidarSerial->read();
  }
  return true;
}

bool RPLidar::parseHealthInfo(HealthInfo& health) {
  // Assuming the command has already been sent and the response is ready
  if (_lidarSerial->available() < 9) { // 7 bytes for descriptor + 3 bytes for health info
    // Not enough data available yet
    return false;
  }

  // Read and ignore descriptor for this example
  byte descriptor[7];
  for (int i = 0; i < 7; ++i) {
    descriptor[i] = _lidarSerial->read();
  }

  // Read the health data
  health.status = _lidarSerial->read();
  health.error_code = _lidarSerial->read() | (_lidarSerial->read() << 8);
  return true;
}

void RPLidar::debugPrintResponse() {
  // Check if there is data available to read from the LIDAR
  while (_lidarSerial->available() > 0) {
    // Read a byte from the software serial port
    byte b = _lidarSerial->read();
    // If the byte is 0x3E, print a newline before printing the byte
    if (b == 0x3E) {
      Serial.println(); // Print newline
    }
    // Ensure leading zeros are printed for values less than 0x10
    if (b < 0x10) {
      Serial.print("0");
    }
    // Print the read byte to the hardware serial port in HEX format
    Serial.print(b, HEX);
    // Print a space for readability
    Serial.print(" ");
  }
  // Print a newline for separation of different reads
  Serial.println();
}


void RPLidar::parseLidarData() {
  // 持續讀取直到沒有足夠的數據來組成一筆完整的5byte數據
  while (_lidarSerial->available() >= 5) {
    // 檢查是否為一個有效的數據開始
    byte firstByte = _lidarSerial->peek(); // 僅查看不移除
    bool isValidStart = (firstByte & 0x01) == 0 && (firstByte & 0x02) == 0x02;
    if (!isValidStart) {
      _lidarSerial->read(); // 移除這個byte並繼續
      continue;
    }

    // 讀取5個byte的數據
    byte data[5];

    for (int i = 0; i < 5; i++) {
      data[i] = _lidarSerial->read();
      //Serial.print(data[i],HEX);Serial.print(" ");
    }

    // 驗證第二個byte的條件
    if ((data[1] & 0x01) != 0x01) {
      // 如果不符合，則丟棄這筆數據並繼續
      //Serial.print("pass.");
      continue;
    }
    // 解析質量
    int quality = (data[0] >> 2) & 0x3F; // 取第8到第2bit作為質量值
    // 角度
    unsigned int angle_combined = ((unsigned int)data[2] << 8) | (unsigned int)data[1];
    unsigned int angle_q6 = angle_combined >> 1;
    //Serial.print(angle_q6,HEX);
    // 解析距離
    int distanceLSB = data[3]; // 距離的[7:0]
    int distanceMSB = data[4]; // 距離的[15:8]
    unsigned int distance_q2 = ((unsigned int)data[4] << 8) | (unsigned int)data[3];

    // 計算實際的角度和距離 根據手冊
    float actualAngle = angle_q6 / 64.0f;
    float actualDistance = distance_q2 / 4.0f;
    if(quality>=15&&actualDistance!=0&&actualDistance<=3982){
      // 打印解析後的數據
      Serial.print("A:");
      Serial.print(actualAngle);
      Serial.print(" ");
      Serial.print("D:");
      Serial.println(actualDistance);
    }
  }
}

void RPLidar::debugparseLidarData() {
  // 持續讀取直到沒有足夠的數據來組成一筆完整的5byte數據
  while (Serial.available() >= 0) {
    // 檢查是否為一個有效的數據開始
    //delay(10);
    byte firstByte = Serial.peek(); // 僅查看不移除
    bool isValidStart = (firstByte & 0x01) == 0 && (firstByte & 0x02) == 0x02;
    if (!isValidStart) {
      Serial.read(); // 移除這個byte並繼續
      continue;
    }

    // 讀取5個byte的數據
    byte data[5];
    for (int i = 0; i < 5; i++) {
      data[i] = Serial.read();
      Serial.print(data[i], HEX); Serial.print(" ");
    }
    Serial.println();
    // 驗證第二個byte的條件
    if ((data[1] & 0x01) != 0x01) {
      // 如果不符合，則丟棄這筆數據並繼續
      Serial.print("pass.");
      continue;
    }
    // 解析質量
    int quality = (data[0] >> 2) & 0x3F; // 取第8到第2bit作為質量值

    // 解析角度
    int angleLSB = (data[1] >> 1) & 0x7F; // 取第8到第2bit作為角度的[6:0]
    int angleMSB = data[2]; // 完整的角度為[14:7]
    int angle_q6 = (angleMSB << 7) | angleLSB; // 合成角度
    // 解析距離
    int distanceLSB = data[3]; // 距離的[7:0]
    int distanceMSB = data[4]; // 距離的[15:8]
    int distance_q2 = (distanceMSB << 8) | distanceLSB; // 合成距離

    // 計算實際的角度和距離
    double actualAngle = angle_q6 / 64.0;
    double actualDistance = distance_q2 / 4.0;

    // 打印解析後的數據
    Serial.print("Quality: ");
    Serial.print(quality);
    Serial.print(", Angle: ");
    Serial.print(actualAngle);
    Serial.print(" degrees, Distance: ");
    Serial.print(actualDistance);
    Serial.println(" mm");
  }

}


void RPLidar::printCurrentMeasurement(float minAngle, float maxAngle, uint8_t minQuality) {
  if (_currentMeasurement.angle >= minAngle && _currentMeasurement.angle <= maxAngle && _currentMeasurement.quality >= minQuality) {
    Serial.print("Distance: ");
    Serial.print(_currentMeasurement.distance);
    Serial.print(" m, Angle: ");
    Serial.print(_currentMeasurement.angle);
    Serial.print(" degrees, Quality: ");
    Serial.println(_currentMeasurement.quality);
  }
}
