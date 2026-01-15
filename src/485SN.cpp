#include "485SN.h"
#include <BLE.h>

extern WeatherData weatherData;

uint16_t ws;//风速
uint16_t wf;//风力等级
uint16_t wd_idx;//风向索引
uint16_t wd_deg;//风向角度
uint16_t hum;//湿度
uint16_t temp;//温度
uint16_t noise;//噪声
uint16_t pm25;//PM2.5
uint16_t pm10;//PM10
uint16_t pres;//大气压
uint16_t lux_high;//20W光照高16位
uint16_t lux_low;//20W光照低16位
uint16_t lux_hundred;//20W光照值 (单位：百 Lux)
uint16_t rain;//雨量
uint16_t compass;//电子指南针角度
uint16_t solar;//太阳总辐射

// 发送总问询帧（地址=01，功能码=03，起始地址=0x01F4(500)，寄存器数=0x0010(16)，CRC=0x0804 (低字节在前)）
// 通过 Serial1 发送，假定串口转485硬件处理方向控制。如果你要用其它串口，请在调用前配置好相应串口。
void send485Request() {
	uint8_t frame[] = {0x01, 0x03, 0x01, 0xF4, 0x00, 0x10, 0x04, 0x08};
	Serial.write(frame, sizeof(frame));
	Serial.flush();
}

void send485read() {
    // 希望一次性把完整的 37 字节帧读完：
    // 如果串口当前只有部分数据到达，循环读取直到累计 37 字节或超时
    const size_t FRAME_LEN = 37;
    uint8_t buf[256];
    size_t total = 0;
    unsigned long start = millis();
    const unsigned long TIMEOUT_MS = 300; // 超时时间，可根据需要调整

    while (total < FRAME_LEN && (millis() - start) < TIMEOUT_MS) {
        size_t avail = (size_t)Serial.available();
        if (avail > 0) {
            size_t need = FRAME_LEN - total;
            if (avail > need) avail = need;
            size_t r = Serial.readBytes((char*)(buf + total), avail);
            if (r > 0) {
                total += r;
                // 读取到数据后重置超时计时器，以等待剩余数据
                start = millis();
            }
        } else {
            delay(1);
        }
    }

    if (total > 0) {
        // 发送调试信息：已读取字节数
        // BLE_sendf("rd=%d", (int)total);
        // 发送原始数据到 BLE（如果需要）
        // BLE_send(buf, total);
        // 如果读取到完整帧，解析数据
        if (total == FRAME_LEN) {
            // 解析寄存器数据
            ws = (uint16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[4]);//风速
            wf = (uint16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[6]);//风力等级
            wd_idx = (uint16_t)(((uint16_t)buf[7] << 8) | (uint16_t)buf[8]);//风向索引
            wd_deg = (uint16_t)(((uint16_t)buf[9] << 8) | (uint16_t)buf[10]);//风向角度
            hum = (uint16_t)(((uint16_t)buf[11] << 8) | (uint16_t)buf[12]);//湿度
            temp = (uint16_t)(((uint16_t)buf[13] << 8) | (uint16_t)buf[14]);//温度
            noise = (uint16_t)(((uint16_t)buf[15] << 8) | (uint16_t)buf[16]);//噪声
            pm25 = (uint16_t)(((uint16_t)buf[17] << 8) | (uint16_t)buf[18]);//PM2.5
            pm10 = (uint16_t)(((uint16_t)buf[19] << 8) | (uint16_t)buf[20]);//PM10
            pres = (uint16_t)(((uint16_t)buf[21] << 8) | (uint16_t)buf[22]);//大气压
            lux_high = (uint16_t)(((uint16_t)buf[23] << 8) | (uint16_t)buf[24]);//20W光照高16位
            lux_low = (uint16_t)(((uint16_t)buf[25] << 8) | (uint16_t)buf[26]);//20W光照低16位
            lux_hundred = (uint16_t)(((uint16_t)buf[27] << 8) | (uint16_t)buf[28]);//20W光照值 (单位：百 Lux)
            rain = (uint16_t)(((uint16_t)buf[29] << 8) | (uint16_t)buf[30]);//雨量
            compass = (uint16_t)(((uint16_t)buf[31] << 8) | (uint16_t)buf[32]);//电子指南针角度
            solar = (uint16_t)(((uint16_t)buf[33] << 8) | (uint16_t)buf[34]);//太阳总辐射
            weatherData.windSpeed_m_s = (float)ws / 10.0f;
            weatherData.windForce = (unsigned int)wf;
            weatherData.windDir_idx = (unsigned int)wd_idx;
            weatherData.windDir_deg = (unsigned int)wd_deg;
            weatherData.humidity_pct = (float)hum / 10.0f;
            weatherData.temperature_C = (float)temp / 10.0f;
            weatherData.noise_dB = (float)noise / 10.0f;
            weatherData.pm25_or_co2_0 = (unsigned int)pm25;
            weatherData.pm10_or_co2_1 = (unsigned int)pm10;
            weatherData.pressure_kPa = (float)pres / 10.0f;
            weatherData.lux20_high = (unsigned int)lux_high;
            weatherData.lux20_low = (unsigned int)lux_low;
            weatherData.lux20_hundredLux = (unsigned int)lux_hundred;
            weatherData.rain_mm = (float)rain / 100.0f;
            weatherData.compass_deg = (float)compass / 10.0f;
            weatherData.solar_W_m2 = (unsigned int)solar;
            BLE_sendf("wind=%.1f m/s\r\n", weatherData.windSpeed_m_s);//风速
            BLE_sendf("degree=%u *\r\n", weatherData.windDir_deg);//风向角度
            BLE_sendf("hum=%.1f %%\r\n", weatherData.humidity_pct);//湿度
            BLE_sendf("temp=%.1f C\r\n", weatherData.temperature_C);//温度
            BLE_sendf("KPa=%.1f kPa\r\n", weatherData.pressure_kPa);//大气压
            BLE_sendf("Lux=%u Lux\r\n", weatherData.lux20_hundredLux);//20W光照值
            BLE_sendf("rain=%.2f mm\r\n", weatherData.rain_mm);//雨量

        }
    }
}

// 将 16 个寄存器解析为 WeatherData
// void parseWeatherRegisters(const uint16_t regs[16], WeatherData &out) {
//     // 下面的缩放仅为示例。根据你的设备手册调整缩放/单位。
//     // regs 索引从 0 对应寄存器 500。
//     out.windSpeed_m_s = regs[0] / 10.0f;        // 假设寄存器按 0.1 m/s 存储
//     out.windForce = (uint8_t)(regs[1] & 0xFF);   // 档位直接取低字节
//     out.windDir_idx = (uint8_t)(regs[2] & 0xFF); // 0-7 档
//     out.windDir_deg = regs[3] / 10.0f;          // 假设按 0.1° 存储
//     out.humidity_pct = regs[4] / 10.0f;         // 假设按 0.1% 存储
//     out.temperature_C = regs[5] / 10.0f;        // 假设按 0.1°C 存储
//     out.noise_dB = regs[6] / 10.0f;             // 假设按 0.1 dB 存储
//     out.pm25_or_co2_0 = regs[7];
//     out.pm10_or_co2_1 = regs[8];
//     out.pressure_kPa = regs[9] / 1000.0f;       // 假设寄存器单位 Pa，转换为 kPa: /1000
//     out.lux20_high = regs[10];
//     out.lux20_low = regs[11];
//     // 合并高低 16 位为一个 32-bit 值，再转换为百 Lux（示例）
//     uint32_t lux32 = ((uint32_t)out.lux20_high << 16) | out.lux20_low;
//     out.lux20_hundredLux = lux32 / 100.0f;      // 假设单位为 Lux，这里转为“百 Lux”
//     out.rain_mm = regs[13] / 100.0f;            // 假设按 0.01 mm 存储
//     out.compass_deg = regs[14] / 10.0f;         // 假设按 0.1° 存储
//     out.solar_W_m2 = regs[15] / 10.0f;          // 假设按 0.1 W/m^2 存储
// }