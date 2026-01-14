#include "BLE.h"
#include <stdarg.h>
#include <stdio.h>

uint32_t txValue = 0;
BLEServer *pServer = NULL; // BLEServer pointer
BLECharacteristic *pTxCharacteristic = NULL; // BLECharacteristic pointer
bool deviceConnected = false; // current connection state
bool oldDeviceConnected = false; // previous connection state

void BLE_init() {
  // 初始化 BLE 设备并创建服务与特征，设置回调
  BLEDevice::init("RHKJ_BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX 特征用于接收客户端写入
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(new Mycallbacks());

  pService->start();
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
}

void BLE_send(const uint8_t* data, size_t len) {
  if (deviceConnected) {
    pTxCharacteristic->setValue((uint8_t *)data, len);
    pTxCharacteristic->notify();
  }
}

void BLE_sendSeq() {
  if (deviceConnected) {
    pTxCharacteristic->setValue((uint8_t *)&txValue, sizeof(txValue));
    pTxCharacteristic->notify();
    txValue++;
  }
}

void BLE_sendf(const char *fmt, ...) {
  if (!deviceConnected || pTxCharacteristic == NULL) return;
  const size_t BUF_SZ = 256;
  char buf[BUF_SZ];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf, BUF_SZ, fmt, args);
  va_end(args);
  if (n < 0) return; // formatting error
  size_t len = (size_t)n;
  if (len >= BUF_SZ) len = BUF_SZ - 1; // truncate if needed
  BLE_send((const uint8_t*)buf, len);
}

void BLE_Connect()//BLE连接处理函数
{
  if (deviceConnected) // 已连接
  {
    // 发送 32-bit 序号（小端）作为 payload，便于接收端检查丢包/新消息
    //BLE_sendSeq();
    //delay(1000); // 如果有太多包要发送，蓝牙会堵塞
  }

  if (!deviceConnected && oldDeviceConnected)//断开连接
  {
    delay(500);//留时间给蓝牙缓冲
    pServer->startAdvertising();//重新广播
    Serial.println("开始广播");
    oldDeviceConnected =deviceConnected;
  }

  if(deviceConnected && !oldDeviceConnected)//正在连接
  {
    // do stuff here on connecting
    oldDeviceConnected =deviceConnected;
  }
}