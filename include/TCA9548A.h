#ifndef TCA9548A_H_
#define TCA9548A_H_

#include <Arduino.h>
#include <Wire.h>

#define TCA_ADDR 0x70

// 选择 TCA 通道（0..7），返回是否成功（ACK）
bool tcaSelect(uint8_t channel);

// 快速检查某个 I2C 地址是否存在（ACK）
bool i2cAddressExists(uint8_t addr);

// 扫描 TCA 的所有通道并打印每个通道上发现的设备地址（或 None）
void scanAllTCAChannels();

void I2C_Clear();

#endif // TCA9548A_H_
