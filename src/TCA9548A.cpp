#include "TCA9548A.h"
#include <Arduino.h>

bool tcaSelect(uint8_t channel) {// 选择 TCA 通道（0..7），返回是否成功（ACK）
  if (channel > 7) return false;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  return (Wire.endTransmission() == 0);
}

bool i2cAddressExists(uint8_t addr) {// 快速检查某个 I2C 地址是否存在（ACK）
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void scanAllTCAChannels() {// 扫描 TCA 的所有通道并打印每个通道上发现的设备地址（或 None）
  Serial.println("Scanning possible TCA addresses 0x70..0x77 and their channels...");
  bool anyTcaFound = false;
  for (uint8_t tca = 0x70; tca <= 0x77; tca++) {
    Wire.beginTransmission(tca);
    uint8_t res = Wire.endTransmission();
    if (res != 0) {
      Serial.printf("No TCA at 0x%02X (res=%d)\r\n", tca, res);
      continue;
    }
    anyTcaFound = true;
    Serial.printf("Found TCA at 0x%02X -- scanning channels:\r\n", tca);

    for (uint8_t ch = 0; ch < 8; ch++) {
      Wire.beginTransmission(tca);
      Wire.write(1 << ch);
      if (Wire.endTransmission() != 0) {
        Serial.printf("  Channel %d: cannot select (no ACK)\r\n", ch);
        continue;
      }
      Serial.printf("  Channel %d: ", ch);
      bool found = false;
      for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
          Serial.printf("0x%02X ", addr);
          found = true;
        }
      }
      if (!found) Serial.println("No devices found");
      else Serial.println();
    }

    Wire.beginTransmission(tca);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();
  }
  if (!anyTcaFound) Serial.println("No TCA devices found in range 0x70..0x77.");
  else Serial.println("TCA scan complete.");
}

void I2C_Clear()
{
  Wire.beginTransmission(TCA_ADDR);// 取消所有通道选择，恢复总线
  Wire.write((uint8_t)0x00);// 写入 0 以断开所有通道
  Wire.endTransmission();// 结束传输
}