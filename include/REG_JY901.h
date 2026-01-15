#ifndef REG_JY901_H
#define REG_JY901_H

#include <stdint.h>

// 默认I2C地址
#define JY901_I2C_ADDR 0x50

// 关键寄存器地址（从你的寄存器表）
#define REG_ACC_X      0x34  // 加速度X
#define REG_ACC_Y      0x35  // 加速度Y
#define REG_ACC_Z      0x36  // 加速度Z
#define REG_GYRO_X     0x37  // 角速度X
#define REG_GYRO_Y     0x38  // 角速度Y
#define REG_GYRO_Z     0x39  // 角速度Z
#define REG_ROLL       0x3D  // 横滚角
#define REG_PITCH      0x3E  // 俯仰角
#define REG_YAW        0x3F  // 航向角
#define REG_ACCRANGE   0x21  // 加速度量程配置
#define REG_GYRORANGE  0x20  // 陀螺仪量程配置

// 数据结构体
typedef struct {
    // 原始值（从寄存器直接读取）
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    int16_t roll_raw, pitch_raw, yaw_raw;
    int16_t temperature_raw;
    
    // 转换后的物理量
    float ax_mss, ay_mss, az_mss;    // 加速度 (m/s²)
    float gx_dps, gy_dps, gz_dps;    // 角速度 (°/s)
    float roll_deg, pitch_deg, yaw_deg; // 角度 (°)
    float temp_c;                    // 温度 (°C)
} JY901_Data;

// 函数声明
bool JY901_init(uint8_t addr = JY901_I2C_ADDR);
bool JY901_readAll(uint8_t addr, JY901_Data &data);
bool JY901_setAccRange(uint8_t addr, uint8_t range);
bool JY901_setGyroRange(uint8_t addr, uint8_t range);

#endif // JY901_H