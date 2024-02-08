#ifndef RPLIDAR_H
#define RPLIDAR_H
#include "Arduino.h"

#include <Stream.h> // 包含 Stream 類以支持多種串行通訊方式
#include <stdint.h>

// Commands without payload and response
#define RPLIDAR_CMD_STOP               0x25
#define RPLIDAR_CMD_SCAN               0x20
#define RPLIDAR_CMD_FORCE_SCAN         0x21
#define RPLIDAR_CMD_RESET              0x40

// RP-Lidar Input Packets
#define RPLIDAR_CMD_SYNC_BYTE        0xA5
#define RPLIDAR_CMDFLAG_HAS_PAYLOAD  0x80

#define RPLIDAR_ANS_SYNC_BYTE1       0xA5
#define RPLIDAR_ANS_SYNC_BYTE2       0x5A

#define RPLIDAR_ANS_PKTFLAG_LOOP     0x1

// Structure to hold device information
struct DeviceInfo {
    byte model;
    byte firmware_minor;
    byte firmware_major;
    byte hardware;
    byte serialnumber[16];
};

// Structure to hold health information
struct HealthInfo {
    byte status;
    unsigned int error_code;
};


struct Measurement {
    float distance;  // 距離，單位可能為米或厘米
    float angle;     // 角度，單位為度
    uint8_t quality; // 測量品質
    bool startBit;   // 表示測量點起始的標記位
};


class RPLidar {
public:
    RPLidar(Stream *stream);
    void begin(long baudRate);
    void sendCommand(byte cmd, byte* payload = nullptr, unsigned int payloadSize = 0);
    void startScan();
    void stopScan();
    void getInfo();
    void getHealth();
    //u_result readResponse();
    bool parseDeviceInfo(DeviceInfo& info);
    bool parseHealthInfo(HealthInfo& health);
    void debugPrintResponse();
    void debugparseLidarData();
    void parseLidarData();
    void printCurrentMeasurement(float minAngle, float maxAngle, uint8_t minQuality);
private:
    Stream* _lidarSerial; // 改為使用 Stream 類型指針兼容軟體硬體串口
    Measurement _currentMeasurement; // 用於儲存當前測量結果的變量
};

#endif // RPLIDAR_H
