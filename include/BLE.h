#ifndef BLE_H_
#define BLE_H_

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

extern uint32_t txValue;
extern BLEServer *pServer;
extern BLECharacteristic *pTxCharacteristic;
extern bool deviceConnected;
extern bool oldDeviceConnected;

#define SERVICE_UUID "12a59900-17cc-11ec-9611-0242ac130002"
#define CHARACTERISTIC_UUID_RX "12a59e0a-17cc-11ec-9621-0242ac130002"
#define CHARACTERISTIC_UUID_TX "12a5a148-17cc-11ec-9621-0242ac130002"

class MyServerCallbacks : public BLEServerCallbacks // BLE回调函数
{
public:
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  };
};

class Mycallbacks : public BLECharacteristicCallbacks
{
public:
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue(); // 接收信息
    if (rxValue.length() > 0) // 向串口输出收到的值
    {
      Serial.print("RX:");
      for (int i = 0; i < (int)rxValue.length(); i++)
      {
        Serial.print(rxValue[i]);
      }
      Serial.println();
    }
  }
};
void BLE_init();// BLE初始化函数
void BLE_Connect();// BLE连接处理函数

// 低级发送函数：直接发送指定字节数组（不修改全局序号）
void BLE_send(const uint8_t* data, size_t len);

// 辅助发送：发送全局 32-bit 序号并自增
void BLE_sendSeq();

// printf 风格的发送函数：格式化后发送（内部使用 vsnprintf，缓冲区限制）
void BLE_sendf(const char *fmt, ...);

#endif
