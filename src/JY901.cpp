#include "JY901.h"
#include <Arduino.h>
#include "BLE.h"

void readJY901_raw(uint8_t addr) {// 读取 JY901 原始 I2C 数据并打印（最多读取 32 字节），仅用于调试
  const uint8_t N = 32;
  uint8_t buf[N];
  uint8_t len = 0;

  Wire.requestFrom((int)addr, (int)N);
  while (Wire.available() && len < N) {
    buf[len++] = Wire.read();
  }

  if (len == 0) {
    Wire.beginTransmission(addr);
    Wire.write((uint8_t)0x00);
    if (Wire.endTransmission(false) == 0) {
      Wire.requestFrom((int)addr, (int)N);
      len = 0;
      while (Wire.available() && len < N) buf[len++] = Wire.read();
    }
  }

  // if (len == 0) {
  //   Serial.println("No data read from JY901 (length = 0). Check wiring/power.");
  //   return;
  // }

  // Serial.printf("Read %u bytes from 0x%02X:\r\n", len, addr);
  // for (uint8_t i = 0; i < len; i++) {
  //   Serial.printf("%02X ", buf[i]);
  //   if ((i + 1) % 16 == 0) Serial.println();
  // }
  // Serial.println();
}

bool readJY901_angles(uint8_t addr, JY901S_AngleData &out) {// 解析角度寄存器 0x3D..0x42（6 字节，小端）
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)0x3D);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((int)addr, (int)6);
  uint8_t buf[6];
  uint8_t i = 0;
  while (Wire.available() && i < 6) buf[i++] = Wire.read();
  if (i != 6) return false;
  out.roll  = (int16_t)((buf[1] << 8) | buf[0]);
  out.pitch = (int16_t)((buf[3] << 8) | buf[2]);
  out.yaw   = (int16_t)((buf[5] << 8) | buf[4]);
  return true;
}

bool readJY901_accel(uint8_t addr, JY901S_AccelData &out) {// 解析加速度寄存器（尝试寄存器基址 0x34..0x39）
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)0x34);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((int)addr, (int)6);
  uint8_t buf[6];
  uint8_t i = 0;
  while (Wire.available() && i < 6) buf[i++] = Wire.read();
  if (i != 6) return false;
  out.ax = (int16_t)((buf[1] << 8) | buf[0]);
  out.ay = (int16_t)((buf[3] << 8) | buf[2]);
  out.az = (int16_t)((buf[5] << 8) | buf[4]);
  const float scale = 16384.0f;
  out.ax_g = (float)out.ax / scale;
  out.ay_g = (float)out.ay / scale;
  out.az_g = (float)out.az / scale;
  return true;
}
