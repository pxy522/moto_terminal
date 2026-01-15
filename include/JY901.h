#ifndef JY901_H_
#define JY901_H_

#include <Arduino.h>
#include <Wire.h>

#define JY901_ADDR 0x50 // 常见 JY901 I2C 地址

struct JY901S_AngleData { int16_t roll; int16_t pitch; int16_t yaw; };
struct JY901S_AccelData { int16_t ax; int16_t ay; int16_t az; float ax_g; float ay_g; float az_g; };

// 读取 JY901 原始 I2C 数据并打印（最多读取 32 字节），仅用于调试
void readJY901_raw(uint8_t addr);

// 解析角度寄存器 0x3D..0x42（6 字节，小端）
bool readJY901_angles(uint8_t addr, JY901S_AngleData &out);

// 解析加速度寄存器（尝试寄存器基址 0x34..0x39），返回以 g 为单位的浮点值
bool readJY901_accel(uint8_t addr, JY901S_AccelData &out);

#endif // JY901_H_
