#ifndef _485SN_H_
#define _485SN_H_

#include <Arduino.h>

// 当前重写：仅实现发送总问询帧（寄存器500到515，一次性读取16个寄存器）
// 问询帧固定为: 01 03 01 F4 00 10 04 08
// 使用有效的 C++ 标识符：调用 `send485Request()` 来通过串口发送该帧。
void send485Request();
void send485read();
// 解析并存放天气站数据的结构体（16 个寄存器）
typedef struct {
	float windSpeed_m_s;      // 风速 (m/s) - 由寄存器值按需转换
	unsigned int windForce;        // 风力等级 (整数档位)
	unsigned int windDir_idx;      // 风向索引 (0-7 档)
	unsigned int windDir_deg;        // 风向角度 (0-360°)
	float humidity_pct;      // 湿度 (%)
	float temperature_C;     // 温度 (°C)
	float noise_dB;          // 噪声 (dB)
	unsigned int pm25_or_co2_0;   // PM2.5 值 或 CO2 值 0（视设备寄存器定义）
	unsigned int pm10_or_co2_1;   // PM10 值 或 CO2 值 1
	float pressure_kPa;      // 大气压 (kPa)
	unsigned int lux20_high;      // 20W 的 Lux 值 高 16 位
	unsigned int lux20_low;       // 20W 的 Lux 值 低 16 位
	unsigned int lux20_hundredLux;  // 20W 光照值 (单位：百 Lux)
	float rain_mm;           // 雨量 (mm)
	float compass_deg;       // 电子指南针角度 (°)
	unsigned int solar_W_m2;        // 太阳总辐射 (W/m^2)
} WeatherData;


#endif
