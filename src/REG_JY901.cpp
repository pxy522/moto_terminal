#include "REG_JY901.h"
#include <Wire.h>

// 辅助函数：读取16位有符号寄存器
static bool readRegister16(uint8_t addr, uint8_t reg, int16_t &value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    
    Wire.requestFrom((int)addr, (int)2);
    if (Wire.available() < 2) return false;
    
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();
    
    // 关键：按照协议，先将high转换为int16_t再移位
    value = (int16_t)(((int16_t)high << 8) | low);
    return true;
}

bool JY901_init(uint8_t addr) {
    // 初始化I2C
    Wire.begin();
    
    // 可选：设置量程
    // 加速度量程 (REG_ACCRANGE):
    // 0: ±2g, 1: ±4g, 2: ±8g, 3: ±16g
    JY901_setAccRange(addr, 3);  // 设置为±16g
    
    // 陀螺仪量程 (REG_GYRORANGE):
    // 0: ±2000°/s, 1: ±1000°/s, 2: ±500°/s, 3: ±250°/s, 4: ±125°/s
    JY901_setGyroRange(addr, 0);  // 设置为±2000°/s
    
    return true;
}

bool JY901_readAll(uint8_t addr, JY901_Data &data) {
    bool success = true;
    
    // 读取加速度（寄存器 0x34-0x36）
    success &= readRegister16(addr, REG_ACC_X, data.ax_raw);
    success &= readRegister16(addr, REG_ACC_Y, data.ay_raw);
    success &= readRegister16(addr, REG_ACC_Z, data.az_raw);
    
    // 读取角速度（寄存器 0x37-0x39）
    success &= readRegister16(addr, REG_GYRO_X, data.gx_raw);
    success &= readRegister16(addr, REG_GYRO_Y, data.gy_raw);
    success &= readRegister16(addr, REG_GYRO_Z, data.gz_raw);
    
    // 读取角度（寄存器 0x3D-0x3F）
    success &= readRegister16(addr, REG_ROLL, data.roll_raw);
    success &= readRegister16(addr, REG_PITCH, data.pitch_raw);
    success &= readRegister16(addr, REG_YAW, data.yaw_raw);
    
    if (!success) return false;
    
    // === 关键：数据转换 ===
    
    // 1. 加速度转换 (根据串口协议：原始值/32768*16g)
    // 注意：这里假设量程已设置为±16g，对应公式中的*16
    const float acc_scale = 16.0f / 32768.0f;  // 转换为g
    const float g_to_ms2 = 9.80665f;           // g转m/s²
    
    float ax_g = data.ax_raw * acc_scale;
    float ay_g = data.ay_raw * acc_scale;
    float az_g = data.az_raw * acc_scale;
    
    data.ax_mss = ax_g * g_to_ms2;
    data.ay_mss = ay_g * g_to_ms2;
    data.az_mss = az_g * g_to_ms2;
    
    // 2. 角速度转换 (根据串口协议：原始值/32768*2000°/s)
    const float gyro_scale = 2000.0f / 32768.0f;  // 转换为°/s
    data.gx_dps = data.gx_raw * gyro_scale;
    data.gy_dps = data.gy_raw * gyro_scale;
    data.gz_dps = data.gz_raw * gyro_scale;
    
    // 3. 角度转换 (根据串口协议：原始值/32768*180°)
    const float angle_scale = 180.0f / 32768.0f;
    data.roll_deg = data.roll_raw * angle_scale;
    data.pitch_deg = data.pitch_raw * angle_scale;
    data.yaw_deg = data.yaw_raw * angle_scale;
    
    return true;
}

bool JY901_setAccRange(uint8_t addr, uint8_t range) {
    // range: 0=±2g, 1=±4g, 2=±8g, 3=±16g
    if (range > 3) return false;
    
    Wire.beginTransmission(addr);
    Wire.write(REG_ACCRANGE);
    Wire.write(range);
    return (Wire.endTransmission() == 0);
}

bool JY901_setGyroRange(uint8_t addr, uint8_t range) {
    // range: 0=±2000°/s, 1=±1000°/s, 2=±500°/s, 3=±250°/s, 4=±125°/s
    if (range > 4) return false;
    
    Wire.beginTransmission(addr);
    Wire.write(REG_GYRORANGE);
    Wire.write(range);
    return (Wire.endTransmission() == 0);
}
