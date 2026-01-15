/******************************************************************************
 *                            头文件包含区域
 ******************************************************************************/
#include <Arduino.h>
#include <WiFi.h>           // WiFi功能
#include <HTTPClient.h>     // HTTP客户端
#include <BLE.h>
#include <Wire.h>           // I2C总线
#include <TCA9548A.h>       // I2C多路复用器
#include <REG_JY901.h>          // IMU传感器
#include "DFRobot_AS7341.h"
#include "485SN.h"          // RS485 Modbus传感器
#include "wifi_wrapper.h"   // WiFi和API封装
#include "SD_card.h"        // SD卡功能
#include <SD_MMC.h>         // SD/MMC文件系统库（非SPI）
#include "Task.h"           // 任务执行
#include "DataProcessor.h"  // 数据处理
#include <Preferences.h>    // NVS存储


/******************************************************************************
 *                            全局变量声明区域
 ******************************************************************************/

// ==================== 传感器对象 ====================
Preferences prefs;
DFRobot_AS7341 as7341;                      // AS7341光谱传感器对象
WeatherData weatherData;                    // 气象数据存储结构
DFRobot_AS7341::sModeOneData_t as7341_data1;// AS7341通道0~5数据
DFRobot_AS7341::sModeTwoData_t as7341_data2;// AS7341通道6~11数据

// ==================== 定时器相关 ====================
hw_timer_t *timer = NULL;               // 主定时器(0.2s)
hw_timer_t *timer2 = NULL;              // 第二个定时器(2s)
hw_timer_t *timer3 = NULL;              // 第三个定时器(5s)
int interruptCounter = 0;               // 定时器计数

// 定时器标志位（由ISR设置，主循环清除）
volatile bool jy901_timer_flag = false;  // IMU读取标志
volatile bool as7341_timer_flag = false; // AS7341读取标志
volatile bool sd_mount_flag = false;     // SD卡挂载标志
volatile bool two_sec_flag = false;      // 2秒定时标志
volatile bool five_sec_flag = false;     // 5秒定时标志
volatile bool send485_flag = false;      // 485发送标志

// ==================== SD卡相关 ====================
static bool sd_file_created = false;     // 文件创建标志
unsigned int bootCount;                  // 启动计数
int bootname_num;                        // SD卡文件序号
char name[50];                           // SD卡文件名缓冲区
char dataString[200];                    // SD卡数据行缓冲区

// ==================== IMU数据 ====================
extern float jy901_vx;                   // x方向线速度
extern float jy901_vy;                   // y方向线速度
extern float jy901_vz;                   // z方向线速度
JY901_Data jy_data;                      // IMU数据结构体

// ==================== 手套数据 ====================
bool Golve_flag = 0;                     // 手套选择标志(0:左手, 1:右手)
extern uint8_t Glove_buf[95];            // 手套数据缓冲区(95字节)
extern uint8_t CKP_buf[10];              // CKP数据缓冲区
extern GLOVE_DATA CKP_data;              // CKP数据结构
extern GLOVE_DATA Glove_data;            // 手套数据结构

uint8_t time_2s;                         // 2秒计时器


/******************************************************************************
 *                            中断服务函数区域
 ******************************************************************************/

/**
 * @brief 主定时器中断服务函数(0.2秒触发)
 * @note 在中断中只设置标志位，避免在ISR中执行耗时操作
 */
void IRAM_ATTR TimerEvent() {
    static uint8_t toggle = 0;
    toggle++;
    
    jy901_timer_flag = true;    // 设置IMU读取标志
    
    // 每10次中断(2秒)触发以下操作
    if(toggle % 10 == 0) {
        sd_mount_flag = true;     // 触发SD卡挂载尝试
        as7341_timer_flag = true; // 触发AS7341读取
        send485_flag = true;      // 触发485帧发送
    }
}

/**
 * @brief 第二定时器中断服务函数(2秒触发)
 */
void IRAM_ATTR Timer2Event() {
    two_sec_flag = true;  // 设置2秒标志
    time_2s++;
}

/**
 * @brief 第三定时器中断服务函数(5秒触发)
 */
void IRAM_ATTR Timer3Event() {
    five_sec_flag = true;  // 设置5秒标志
}


/******************************************************************************
 *                            重力补偿函数
 ******************************************************************************/

void compensateGravity() {
    // 将角度从度转换为弧度
    float roll_rad = jy_data.roll_deg * PI / 180.0f;
    float pitch_rad = jy_data.pitch_deg * PI / 180.0f;
    
    const float g = 9.80665f;
    
    // 计算重力分量
    float gx = -g * sinf(pitch_rad);
    float gy = g * sinf(roll_rad) * cosf(pitch_rad);
    float gz = g * cosf(roll_rad) * cosf(pitch_rad);
    
    // 得到线性加速度（注意：这会修改原始数据）
    jy_data.ax_mss -= gx;
    jy_data.ay_mss -= gy;
    jy_data.az_mss -= gz;
}


/******************************************************************************
 *                            setup()函数区域
 ******************************************************************************/

void setup() {
    // ==================== 串口初始化 ====================
    Serial.begin(4800);                              // 主串口
    Serial2.begin(9600, SERIAL_8N1, 18, 17);        // 串口屏
    Serial1.begin(115600, SERIAL_8N1, 4, 5);        // 手套数据

    Serial.printf("--------");
    
    // ==================== NVS存储初始化 ====================
    prefs.begin("my_app", false);
    bootCount = prefs.getUInt("boot_count", 0);
    bootCount++;
    bootname_num = bootCount;
    prefs.putUInt("boot_count", bootCount);
    prefs.end();
    
    // ==================== GPIO初始化 ====================
    pinMode(16, OUTPUT);
    digitalWrite(16, HIGH);  // 初始设置为右手模式
    
    // ==================== WiFi连接 ====================
    // WiFi_Connect("iQOO Neo10 Pro+", "888688816");
    
    // ==================== BLE初始化 ====================
    BLE_init();
    
    // ==================== 定时器初始化 ====================
    // 主定时器(0.2s)
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &TimerEvent, true);
    timerAlarmWrite(timer, 20000, true);  // 20000us = 0.02s
    timerAlarmEnable(timer);
    
    // 第二定时器(2s)
    timer2 = timerBegin(1, 80, true);
    timerAttachInterrupt(timer2, &Timer2Event, true);
    timerAlarmWrite(timer2, 2000000, true);  // 2,000,000us = 2s
    timerAlarmEnable(timer2);
    
    // 第三定时器(5s)
    timer3 = timerBegin(2, 80, true);
    timerAttachInterrupt(timer3, &Timer3Event, true);
    timerAlarmWrite(timer3, 500000, true);   // 5,000,000us = 5s
    timerAlarmEnable(timer3);
    
    // ==================== I2C初始化 ====================
    Wire.begin(1, 2);  // SDA=IO1, SCL=IO2
    scanAllTCAChannels();  // 扫描所有TCA通道

    // ==================== JY901初始化 ====================
    if (!JY901_init(JY901_I2C_ADDR)) {
        Serial.println("JY901初始化失败！");
        while(1);
    }
    Serial.println("JY901初始化成功");

    // ==================== AS7341初始化 ====================
    tcaSelect(4);  // 选择TCA通道4
    while (as7341.begin() != 0) {
        BLE_sendf("IIC初始化失败");
        tcaSelect(4);  // 重新选择通道并重试
        delay(500);
    }
    
    // 恢复I2C总线
    Wire.beginTransmission(TCA_ADDR);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();
    
    // ==================== RS485初始化 ====================
    // send485Request();  // 发送初始化帧
    
    // ==================== SD卡初始化 ====================
    SD_card_init();
    
    // ==================== 系统就绪确认 ====================
    Serial2.println("OK");  // 通知串口屏系统已就绪
}


/******************************************************************************
 *                            loop()函数区域
 ******************************************************************************/

void loop() {
    // ==================== 串口数据处理 ====================
    // 处理485数据
    if (Serial.available()) {
        send485read();  // 读取并解析485数据
    }
    
    // 处理手套数据
    if (Serial1.available()) {
        receiveAsString();  // 接收手套数据
        
        // 切换手套选择标志
        if (Golve_flag == 1) {
            digitalWrite(16, HIGH);
            Golve_flag = 0;
        } else if (Golve_flag == 0) {
            digitalWrite(16, LOW);
            Golve_flag = 1;
        }
        
        // 将手套数据写入SD卡
        String p(name);
        File f = SD_MMC.open(p.c_str(), FILE_APPEND);
        
        if (Glove_data.RL == 'R') {
            // 右手数据格式化
            f.printf("R,%01d,%02d:1:%02d.%02d,%03d,%03d-", 
                    Glove_data.mode_R, Glove_data.temp_R,
                    (Glove_data.temp_now_R[0]/100), (Glove_data.temp_now_R[0]%100),
                    Glove_data.P_R[0], Glove_data.MAX_R[0]);
            // ... 其他手指数据类似
            f.printf("7:%02d.%02d,%03d,%03d\r\n",
                    (Glove_data.temp_now_R[6]/100), (Glove_data.temp_now_R[6]%100),
                    Glove_data.P_R[6], Glove_data.MAX_R[6]);
        } else if (Glove_data.RL == 'L') {
            // 左手数据格式化
            f.printf("L,%01d,%02d:1:%02d.%02d,%03d,%03d-",
                    Glove_data.mode_L, Glove_data.temp_L,
                    (Glove_data.temp_now_L[0]/100), (Glove_data.temp_now_L[0]%100),
                    Glove_data.P_L[0], Glove_data.MAX_L[0]);
            // ... 其他手指数据类似
            f.printf("7:%02d.%02d,%03d,%03d\r\n",
                    (Glove_data.temp_now_L[6]/100), (Glove_data.temp_now_L[6]%100),
                    Glove_data.P_L[6], Glove_data.MAX_L[6]);
        }
        f.close();
    }
    
    // 处理CKP数据
    if (Serial2.available()) {
        CKP_receive();
        BLE_sendf("%c", CKP_data.RL);  // 调试输出
        memset(CKP_buf, 0, sizeof(CKP_buf));  // 清空缓冲区
    }
    
    // ==================== BLE连接管理 ====================
    BLE_Connect();
    
    // ==================== SD卡管理 ====================
    // 尝试挂载SD卡
    if (sd_mount_flag && !SD_card_is_mounted()) {
        sd_mount_flag = false;
        SD_card_try_mount_once();
    }
    
    // 创建SD卡文件（仅一次）
    if (SD_card_is_mounted() && !sd_file_created) {
        sprintf((char*)name, "/RHKJ_%d.txt", bootCount);
        if (SD_card_create_file(name)) {
            BLE_sendf("Create OK:%s", name);
        }
        sd_file_created = true;
    }
    
    // ==================== 2秒定时任务 ====================
    if (two_sec_flag) {
        two_sec_flag = false;
        static uint32_t log_seconds = 0;  // 累计秒数
        
        if (SD_card_is_mounted()) {
            // 写入时间戳
            if (SD_card_append_timestamp(name, log_seconds)) {
                uint32_t sec = log_seconds % 86400;
                uint32_t hh = sec / 3600;
                uint32_t mm = (sec % 3600) / 60;
                uint32_t ss = sec % 60;
                BLE_sendf("TS %02u:%02u:%02u\r\n", hh, mm, ss);
            }
            
            // 打开文件并写入数据
            String p(name);
            File f = SD_MMC.open(p.c_str(), FILE_APPEND);
            
            // 写入IMU数据
            f.printf("%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\r\n",
                    jy_data.ax_mss, jy_data.ay_mss, jy_data.az_mss,
                    jy901_vx, jy901_vy, jy901_vz);
            
            // 写入AS7341数据
            f.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                    as7341_data1.ADF1, as7341_data1.ADF2, as7341_data1.ADF3,
                    as7341_data1.ADF4, as7341_data1.ADCLEAR, as7341_data1.ADNIR,
                    as7341_data2.ADF5, as7341_data2.ADF6, as7341_data2.ADF7,
                    as7341_data2.ADF8, as7341_data2.ADCLEAR, as7341_data2.ADNIR);
            
            // 写入气象数据
            if (weatherData.humidity_pct != 0) {
                f.printf("%.2f,%d,%.2f,%.2f,%.2f,%d,%.2f\r\n",
                        weatherData.windSpeed_m_s, weatherData.windDir_deg,
                        weatherData.humidity_pct, weatherData.temperature_C,
                        weatherData.pressure_kPa, weatherData.lux20_hundredLux,
                        weatherData.rain_mm);
            } else {
                f.printf("\r\n");  // 无数据时写入空行
            }
            
            f.printf("0/00\r\n");
            f.close();
        }
        
        log_seconds += 2;  // 增加2秒
    }
    
    // ==================== 485数据发送 ====================
    if (send485_flag) {
        send485_flag = false;
        send485Request();
    }
    
    // ==================== IMU数据处理 ====================
    if (jy901_timer_flag) {
        jy901_timer_flag = false;
        
      if (tcaSelect(1)) {
        // 读取原始数据
        if (JY901_readAll(JY901_I2C_ADDR, jy_data)) {

            Serial.println("\n=== IMU数据诊断 ===");
            Serial.printf("原始加速度: ax=%.3f, ay=%.3f, az=%.3f m/s²\n", 
                     jy_data.ax_mss, jy_data.ay_mss, jy_data.az_mss);
            Serial.printf("角度: pitch=%.2f°, roll=%.2f°\n", jy_data.pitch_deg, jy_data.roll_deg);
          // 消除重力投影影响
        //   compensateGravity();            // 计算线性加速度
          float ax_lin = jy_data.ax_mss;  // 注意：compensateGravity修改了jy_data.ax_mss
          float ay_lin = jy_data.ay_mss;
          float az_lin = jy_data.az_mss;

          // 解算线速度
          DP_jy901(ax_lin, ay_lin, az_lin, jy_data.pitch_deg, jy_data.roll_deg);

          I2C_Clear();  // 恢复I2C总线
        }
     }
    }
    
    // ==================== AS7341数据处理 ====================
    if (as7341_timer_flag) {
        as7341_timer_flag = false;
        
        if (tcaSelect(4)) {
            // 读取AS7341数据（两个模式）
            as7341.startMeasure(as7341.eF1F4ClearNIR);
            as7341_data1 = as7341.readSpectralDataOne();
            
            as7341.startMeasure(as7341.eF5F8ClearNIR);
            as7341_data2 = as7341.readSpectralDataTwo();
            
            I2C_Clear();  // 恢复I2C总线
            
            // 可选：发送光谱数据到BLE
            /*
            BLE_sendf("F1(405-425nm): %d\r\n", as7341_data1.ADF1);
            BLE_sendf("F2(435-455nm): %d\r\n", as7341_data1.ADF2);
            ... 其他通道数据
            */
        } else {
            // BLE_sendf("TCA select ch4 failed before F1-F8 measure");
        }
    }
    
    // ==================== 5秒定时任务 ====================
    if (five_sec_flag) {
        five_sec_flag = false;
        
        // 向串口屏更新所有数据
        // 系统信息
        Serial2.printf("main.x8.val=%03d\xff\xff\xff", bootname_num);
        Serial2.printf("main.x8.val=%03d\xff\xff\xff", bootname_num);
        
        // IMU数据
        Serial2.printf("main.x0.val=%05d\xff\xff\xff", (int)(jy_data.roll_deg*1000));
        Serial2.printf("main.x1.val=%05d\xff\xff\xff", (int)(jy_data.pitch_deg*1000));
        Serial2.printf("main.x2.val=%04d\xff\xff\xff", (int)(jy901_vx*1000));
        Serial2.printf("main.x3.val=%04d\xff\xff\xff", (int)(jy901_vy*1000));
        
        // 气象数据
        Serial2.printf("main.x4.val=%03d\xff\xff\xff", (int)(weatherData.windSpeed_m_s*10));
        Serial2.printf("main.n0.val=%d\xff\xff\xff", weatherData.windDir_deg);
        Serial2.printf("main.x5.val=%03d\xff\xff\xff", (int)(weatherData.humidity_pct*10));
        Serial2.printf("main.x6.val=%03d\xff\xff\xff", (int)(weatherData.temperature_C*10));
        Serial2.printf("main.x7.val=%03d\xff\xff\xff", (int)(weatherData.pressure_kPa*10));
        Serial2.printf("main.n4.val=%d\xff\xff\xff", weatherData.lux20_hundredLux);
        Serial2.printf("main.x9.val=%03d\xff\xff\xff", (int)(weatherData.rain_mm*10));
        
        // AS7341数据
        Serial2.printf("main.n5.val=%d\xff\xff\xff", as7341_data2.ADCLEAR);
        Serial2.printf("main.n6.val=%d\xff\xff\xff", as7341_data1.ADNIR);
        
        // 手套数据（右手）
        Serial2.printf("PID_R.x0.val=%d\xff\xff\xff", Glove_data.temp_now_R[0]);
        Serial2.printf("PID_R.x3.val=%d\xff\xff\xff", Glove_data.temp_now_R[1]);
        Serial2.printf("PID_R.x6.val=%d\xff\xff\xff", Glove_data.temp_now_R[2]);
        Serial2.printf("PID_R.x9.val=%d\xff\xff\xff", Glove_data.temp_now_R[3]);
        Serial2.printf("PID_R.x12.val=%d\xff\xff\xff", Glove_data.temp_now_R[4]);
        Serial2.printf("PID_R.x15.val=%d\xff\xff\xff", Glove_data.temp_now_R[5]);
        Serial2.printf("PID_R.x18.val=%d\xff\xff\xff", Glove_data.temp_now_R[6]);
        
        // 手套数据（左手）
        Serial2.printf("PID_L.x0.val=%d\xff\xff\xff", Glove_data.temp_now_L[0]);
        Serial2.printf("PID_L.x3.val=%d\xff\xff\xff", Glove_data.temp_now_L[1]);
        Serial2.printf("PID_L.x6.val=%d\xff\xff\xff", Glove_data.temp_now_L[2]);
        Serial2.printf("PID_L.x9.val=%d\xff\xff\xff", Glove_data.temp_now_L[3]);
        Serial2.printf("PID_L.x12.val=%d\xff\xff\xff", Glove_data.temp_now_L[4]);
        Serial2.printf("PID_L.x15.val=%d\xff\xff\xff", Glove_data.temp_now_L[5]);
        Serial2.printf("PID_L.x18.val=%d\xff\xff\xff", Glove_data.temp_now_L[6]);
    }
    
    delay(1);  // 短暂延时，防止忙等待
}

