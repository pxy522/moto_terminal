/******************************************************************************
 *                   ESP32 摩托车数据终端 - FreeRTOS
 *
 *  核心功能:
 *  - 高精度 IMU 姿态解算 (100Hz, JY901 9轴融合)
 *  - 多传感器数据采集
 *  - 实时数据记录到 SD 卡
 *  - WiFi 和 BLE 无线传输
 ******************************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <SD_MMC.h>
#include <Preferences.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

// 项目模块
#include "BLE.h"
#include "TCA9548A.h"
#include "REG_JY901.h"
#include "DFRobot_AS7341.h"
#include "485SN.h"
#include "wifi_wrapper.h"
#include "SD_card.h"
#include "DataProcessor.h"
#include "IMU_Fusion.h"
#include "IMU_Tasks.h"
#include "System_Init.h"
#include "GNSS_BE881.h"

/******************************************************************************
 *                            配置开关
 ******************************************************************************/
#define ENABLE_AS7341_TASK      0   // AS7341 光谱传感器
#define ENABLE_RS485_TASK       1   // RS485 气象站
#define ENABLE_DISPLAY_TASK     1   // 串口屏显示
#define ENABLE_BLE_TASK         1   // 蓝牙通信
#define ENABLE_GNSS_TASK        1   // GNSS 定位模块

// IMU 诊断输出（设为 0 禁用，1 启用）
#define ENABLE_IMU_DIAGNOSTICS  0
#define ENABLE_GNSS_DIAGNOSTICS 1   // GNSS 诊断输出

/******************************************************************************
 *                            任务配置
 ******************************************************************************/
// 优先级
#define TASK_PRIORITY_IMU           5
#define TASK_PRIORITY_GNSS          4   // GNSS 任务优先级
#define TASK_PRIORITY_SERIAL_RX     4
#define TASK_PRIORITY_AS7341        2
#define TASK_PRIORITY_RS485         3
#define TASK_PRIORITY_BLE           2
#define TASK_PRIORITY_SD_WRITE      2
#define TASK_PRIORITY_DISPLAY       1

// 堆栈大小
#define STACK_SIZE_IMU              4096
#define STACK_SIZE_GNSS             3072
#define STACK_SIZE_SERIAL_RX        4096
#define STACK_SIZE_AS7341           4096
#define STACK_SIZE_RS485            3072
#define STACK_SIZE_BLE              3072
#define STACK_SIZE_SD_WRITE         4096
#define STACK_SIZE_DISPLAY          3072

/******************************************************************************
 *                            全局对象和变量
 ******************************************************************************/
Preferences prefs;
DFRobot_AS7341 as7341;
DFRobot_AS7341::sModeOneData_t as7341_data1;
DFRobot_AS7341::sModeTwoData_t as7341_data2;

WeatherData weatherData;
JY901_Data jy_data;

bool Golve_flag = 0;
extern uint8_t Glove_buf[95];
extern uint8_t CKP_buf[10];
extern GLOVE_DATA CKP_data;
extern GLOVE_DATA Glove_data;

unsigned int bootCount;
int bootname_num;
char name[50];
static bool sd_file_created = false;

// IMU 速度变量
extern float jy901_vx;
extern float jy901_vy;
extern float jy901_vz;

// GNSS 速度变量（用于显示）
volatile float gnss_speed_mps = 0.0f;  // GNSS速度 (m/s)
volatile float gnss_speed_kmh = 0.0f;  // GNSS速度 (km/h)
volatile float gnss_course = 0.0f;     // GNSS航向 (度)
volatile bool gnss_valid = false;      // GNSS数据是否有效

// 重力分量（供诊断使用）
float gx_calculated = 0.0f;
float gy_calculated = 0.0f;
float gz_calculated = 0.0f;

// IMU 融合对象 (定义在 IMU_Fusion.cpp)
extern IMU_Fusion imuFusion;

/******************************************************************************
 *                            FreeRTOS 同步对象
 ******************************************************************************/
SemaphoreHandle_t mutexI2C = NULL;
SemaphoreHandle_t mutexSD = NULL;
SemaphoreHandle_t mutexSerial = NULL;
SemaphoreHandle_t mutexSerial2 = NULL;

TaskHandle_t taskHandleIMU = NULL;
TaskHandle_t taskHandleGNSS = NULL;
TaskHandle_t taskHandleAS7341 = NULL;
TaskHandle_t taskHandleRS485 = NULL;
TaskHandle_t taskHandleSerialRx = NULL;
TaskHandle_t taskHandleSDWrite = NULL;
TaskHandle_t taskHandleDisplay = NULL;
TaskHandle_t taskHandleBLE = NULL;

/******************************************************************************
 *                            安全串口打印宏
 ******************************************************************************/
#define SAFE_SERIAL_PRINT(fmt, ...) \
    do { \
        if (xSemaphoreTake(mutexSerial, pdMS_TO_TICKS(100)) == pdTRUE) { \
            Serial.printf(fmt, ##__VA_ARGS__); \
            xSemaphoreGive(mutexSerial); \
        } \
    } while(0)

#define SAFE_SERIAL_PRINTLN(msg) \
    do { \
        if (xSemaphoreTake(mutexSerial, pdMS_TO_TICKS(100)) == pdTRUE) { \
            Serial.println(msg); \
            xSemaphoreGive(mutexSerial); \
        } \
    } while(0)

/******************************************************************************
 *                            IMU 任务 - 100Hz
 ******************************************************************************/
void taskIMU(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);  // 100Hz (10ms)

    SAFE_SERIAL_PRINTLN("[IMU任务] 启动，100Hz");

    // 初始化 IMU 融合算法
    imuFusion.begin();
    imuFusion.setSampleFrequency(100.0f);
    imuFusion.setBeta(0.05f);

    // 设置针对摩托车优化的 ZUPT 阈值
    imuFusion.setStationaryThresholds(0.15f, 0.08f);  // 加速度 0.15 m/s², 速度 0.08 m/s

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (tcaSelect(1)) {
                if (JY901_readAll(JY901_I2C_ADDR, jy_data)) {
                    // ✅ 使用正确的 IMU 算法
                    processIMU_Correct();

#if ENABLE_IMU_DIAGNOSTICS
                    IMU_DiagnosePrint_Correct();
#endif
                }
                I2C_Clear();
            }
            xSemaphoreGive(mutexI2C);
        }
    }
}

/******************************************************************************
 *                            GNSS 任务
 ******************************************************************************/
#if ENABLE_GNSS_TASK
void taskGNSS(void *parameter) {
    SAFE_SERIAL_PRINTLN("[GNSS任务] 启动");

    // 初始化 GNSS
    // 物理接线: GNSS TX → GPIO10, GNSS RX → GPIO12
    // 所以 ESP32 需要配置: RX=GPIO10 (接收GNSS的TX), TX=GPIO12 (发送到GNSS的RX)
    gnss.begin(10, 12, 115200);

    static uint32_t last_print_time = 0;
    static uint32_t last_check_time = 0;

    while (1) {
        // 更新 GNSS 数据
        gnss.update();

        // 每 5 秒检查一次是否有数据接收
        if (millis() - last_check_time >= 5000) {
            last_check_time = millis();
            uint32_t msg_count = gnss.getMessageCount();

            if (msg_count == 0) {
                SAFE_SERIAL_PRINTLN("\n[GNSS] ❌ 未接收到任何NMEA数据!");
                SAFE_SERIAL_PRINTLN("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
                SAFE_SERIAL_PRINTLN("🔍 故障排查步骤:");
                SAFE_SERIAL_PRINTLN("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
                SAFE_SERIAL_PRINTLN("1️⃣  检查物理接线 (最常见问题!):");
                SAFE_SERIAL_PRINTLN("   GNSS TX(PIN3) → ESP32 GPIO10 ✓");
                SAFE_SERIAL_PRINTLN("   GNSS RX(PIN4) → ESP32 GPIO12 ✓");
                SAFE_SERIAL_PRINTLN("   GNSS VCC(PIN5) → ESP32 5V ✓");
                SAFE_SERIAL_PRINTLN("   GNSS GND(PIN2) → ESP32 GND ✓");
                SAFE_SERIAL_PRINTLN("");
                SAFE_SERIAL_PRINTLN("2️⃣  检查GNSS模块状态:");
                SAFE_SERIAL_PRINTLN("   蓝灯持续闪烁 = 正常发送数据 ✓");
                SAFE_SERIAL_PRINTLN("   蓝灯闪10秒停止 = TX线接错! ❌");
                SAFE_SERIAL_PRINTLN("   完全不亮 = 未上电或损坏 ❌");
                SAFE_SERIAL_PRINTLN("");
                SAFE_SERIAL_PRINTLN("3️⃣  如果蓝灯持续闪烁但无数据:");
                SAFE_SERIAL_PRINTLN("   - 检查TX/RX是否接反");
                SAFE_SERIAL_PRINTLN("   - 确认GPIO10没有被其他外设占用");
                SAFE_SERIAL_PRINTLN("   - 用万用表测量GNSS TX引脚是否有3.3V电平变化");
                SAFE_SERIAL_PRINTLN("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
            }
        }

        // 检查是否有新数据
        if (gnss.available()) {
            GNSS_Data data = gnss.getData();

            // 更新全局GNSS变量（用于显示任务）
            gnss_speed_mps = data.speed_mps;
            gnss_speed_kmh = data.speed_kmh;
            gnss_course = data.course;
            gnss_valid = data.is_valid;

            // 每秒打印一次 GNSS 信息
            if (millis() - last_print_time >= 1000) {
                last_print_time = millis();

                if (data.is_valid) {
                    // 定位有效 - 打印完整信息
                    SAFE_SERIAL_PRINT("[GNSS] 定位✓ | 星=%d HDOP=%.1f | 位置=%.6f,%.6f | 高度=%.1fm | 速度=%.1fkm/h | 航向=%.1f°\n",
                                     data.satellites,
                                     data.hdop,
                                     data.latitude,
                                     data.longitude,
                                     data.altitude,
                                     data.speed_kmh,
                                     data.course);
                } else {
                    // 未定位 - 打印搜星状态
                    static uint32_t no_fix_count = 0;
                    no_fix_count++;

                    SAFE_SERIAL_PRINT("[GNSS] 搜星中... | 星=%d | 质量=%d | 消息=%u",
                                     data.satellites,
                                     data.fix_quality,
                                     gnss.getMessageCount());

                    // 每30秒提示一次
                    if (no_fix_count % 30 == 1) {
                        SAFE_SERIAL_PRINT(" 💡提示: 需要室外空旷环境,初次定位需1-2分钟");
                    }
                    SAFE_SERIAL_PRINTLN("");
                }
            }

            // 定位有效时,可与 IMU 进行融合
            if (data.is_valid) {
                // TODO: 实现 IMU+GNSS 卡尔曼融合
                // 这里暂时只使用 GNSS 速度修正 IMU 漂移
            }

#if ENABLE_GNSS_DIAGNOSTICS
            // 详细诊断信息 (每 10 秒)
            static uint32_t last_diag_time = 0;
            if (millis() - last_diag_time >= 10000) {
                gnss.printDiagnostics();
                last_diag_time = millis();
            }
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms 循环
    }
}
#endif

/******************************************************************************
 *                            AS7341 任务
 ******************************************************************************/
#if ENABLE_AS7341_TASK
void taskAS7341(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(2000);  // 2秒

    SAFE_SERIAL_PRINTLN("[AS7341任务] 启动");

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(500)) == pdTRUE) {
            if (tcaSelect(4)) {
                as7341.startMeasure(as7341.eF1F4ClearNIR);
                as7341_data1 = as7341.readSpectralDataOne();
                I2C_Clear();
            }
            xSemaphoreGive(mutexI2C);
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(500)) == pdTRUE) {
            if (tcaSelect(4)) {
                as7341.startMeasure(as7341.eF5F8ClearNIR);
                as7341_data2 = as7341.readSpectralDataTwo();
                I2C_Clear();
            }
            xSemaphoreGive(mutexI2C);
        }
    }
}
#endif

/******************************************************************************
 *                            RS485 任务
 ******************************************************************************/
#if ENABLE_RS485_TASK
void taskRS485(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(200);

    SAFE_SERIAL_PRINTLN("[RS485任务] 启动");

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        send485Request();
        vTaskDelay(pdMS_TO_TICKS(50));

        if (Serial.available()) {
            send485read();
        }
    }
}
#endif

/******************************************************************************
 *                            串口接收任务
 ******************************************************************************/
void taskSerialRx(void *parameter) {
    SAFE_SERIAL_PRINTLN("[串口接收任务] 启动");

    while (1) {
        // 处理手套数据 (Serial1)
        if (Serial1.available()) {
            receiveAsString();

            if (Golve_flag == 1) {
                digitalWrite(16, HIGH);
                Golve_flag = 0;
            } else {
                digitalWrite(16, LOW);
                Golve_flag = 1;
            }

            // 写入 SD 卡
            if (xSemaphoreTake(mutexSD, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (SD_card_is_mounted() && sd_file_created) {
                    String p(name);
                    File f = SD_MMC.open(p.c_str(), FILE_APPEND);

                    if (Glove_data.RL == 'R') {
                        f.printf("R,%01d,%02d:1:%02d.%02d,%03d,%03d-",
                                Glove_data.mode_R, Glove_data.temp_R,
                                (Glove_data.temp_now_R[0]/100), (Glove_data.temp_now_R[0]%100),
                                Glove_data.P_R[0], Glove_data.MAX_R[0]);
                        for (int i = 1; i < 6; i++) {
                            f.printf("%d:%02d.%02d,%03d,%03d-",
                                    i+1,
                                    (Glove_data.temp_now_R[i]/100), (Glove_data.temp_now_R[i]%100),
                                    Glove_data.P_R[i], Glove_data.MAX_R[i]);
                        }
                        f.printf("7:%02d.%02d,%03d,%03d\r\n",
                                (Glove_data.temp_now_R[6]/100), (Glove_data.temp_now_R[6]%100),
                                Glove_data.P_R[6], Glove_data.MAX_R[6]);
                    } else if (Glove_data.RL == 'L') {
                        f.printf("L,%01d,%02d:1:%02d.%02d,%03d,%03d-",
                                Glove_data.mode_L, Glove_data.temp_L,
                                (Glove_data.temp_now_L[0]/100), (Glove_data.temp_now_L[0]%100),
                                Glove_data.P_L[0], Glove_data.MAX_L[0]);
                        for (int i = 1; i < 6; i++) {
                            f.printf("%d:%02d.%02d,%03d,%03d-",
                                    i+1,
                                    (Glove_data.temp_now_L[i]/100), (Glove_data.temp_now_L[i]%100),
                                    Glove_data.P_L[i], Glove_data.MAX_L[i]);
                        }
                        f.printf("7:%02d.%02d,%03d,%03d\r\n",
                                (Glove_data.temp_now_L[6]/100), (Glove_data.temp_now_L[6]%100),
                                Glove_data.P_L[6], Glove_data.MAX_L[6]);
                    }
                    f.close();
                }
                xSemaphoreGive(mutexSD);
            }
        }

        // 处理 CKP 数据 (Serial2)
        if (Serial2.available()) {
            CKP_receive();
            memset(CKP_buf, 0, sizeof(CKP_buf));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/******************************************************************************
 *                            SD 卡写入任务
 ******************************************************************************/
void taskSDWrite(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(2000);  // 2秒
    uint32_t log_seconds = 0;

    SAFE_SERIAL_PRINTLN("[SD卡写入任务] 启动");

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // 检查并挂载 SD 卡
        if (!SD_card_is_mounted()) {
            SD_card_try_mount_once();
        }

        // 创建文件
        if (SD_card_is_mounted() && !sd_file_created) {
            if (xSemaphoreTake(mutexSD, pdMS_TO_TICKS(100)) == pdTRUE) {
                sprintf((char*)name, "/RHKJ_%d.txt", bootCount);
                if (SD_card_create_file(name)) {
                    SAFE_SERIAL_PRINT("SD卡文件创建: %s\n", name);
                    sd_file_created = true;
                }
                xSemaphoreGive(mutexSD);
            }
        }

        // 写入数据
        if (SD_card_is_mounted() && sd_file_created) {
            if (xSemaphoreTake(mutexSD, pdMS_TO_TICKS(200)) == pdTRUE) {
                SD_card_append_timestamp(name, log_seconds);

                String p(name);
                File f = SD_MMC.open(p.c_str(), FILE_APPEND);

                // IMU 数据
                f.printf("%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\r\n",
                        jy_data.ax_mss, jy_data.ay_mss, jy_data.az_mss,
                        jy901_vx, jy901_vy, jy901_vz);

                // 光谱数据
                f.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                        as7341_data1.ADF1, as7341_data1.ADF2, as7341_data1.ADF3,
                        as7341_data1.ADF4, as7341_data1.ADCLEAR, as7341_data1.ADNIR,
                        as7341_data2.ADF5, as7341_data2.ADF6, as7341_data2.ADF7,
                        as7341_data2.ADF8, as7341_data2.ADCLEAR, as7341_data2.ADNIR);

                // 气象数据
                if (weatherData.humidity_pct != 0) {
                    f.printf("%.2f,%d,%.2f,%.2f,%.2f,%d,%.2f\r\n",
                            weatherData.windSpeed_m_s, weatherData.windDir_deg,
                            weatherData.humidity_pct, weatherData.temperature_C,
                            weatherData.pressure_kPa, weatherData.lux20_hundredLux,
                            weatherData.rain_mm);
                } else {
                    f.printf("\r\n");
                }

                f.printf("0/00\r\n");
                f.close();

                xSemaphoreGive(mutexSD);
            }
        }

        log_seconds += 2;
    }
}

/******************************************************************************
 *                            显示更新任务
 *
 *  功能说明:
 *  - x2: GNSS总速度 (m/s × 1000)
 *  - x3: GNSS航向 (度, 0-360)
 *  - GNSS速度精度: ±0.18 km/h, 无漂移
 ******************************************************************************/
#if ENABLE_DISPLAY_TASK
void taskDisplay(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000);

    SAFE_SERIAL_PRINTLN("[显示任务] 启动");
    SAFE_SERIAL_PRINTLN("[显示任务] 速度显示: x2=GNSS速度(×1000), x3=航向(度)");

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        if (xSemaphoreTake(mutexSerial2, pdMS_TO_TICKS(500)) == pdTRUE) {
            Serial2.printf("main.x8.val=%03d\xff\xff\xff", bootname_num);

            // 计算去重力后的加速度
            float linear_ax = jy_data.ax_mss - gx_calculated;
            float linear_ay = jy_data.ay_mss - gy_calculated;

            Serial2.printf("main.x0.val=%05d\xff\xff\xff", (int)(linear_ax*1000));
            Serial2.printf("main.x1.val=%05d\xff\xff\xff", (int)(linear_ay*1000));

            // 显示速度：直接使用GNSS总速度
            // x2 = 总速度 (m/s) × 1000
            // x3 = 航向 (度)
            float display_speed = gnss_speed_mps;  // 总速度 (m/s)
            float display_course = gnss_course;    // 航向 (度)

            // 调试：每10秒打印一次速度信息
            static uint32_t last_debug = 0;
            if (millis() - last_debug >= 10000) {
                last_debug = millis();
                SAFE_SERIAL_PRINT("[速度调试] GNSS有效=%d, 速度: %.2f m/s (%.1f km/h), 航向: %.1f°, 屏幕: x2=%d (速度×1000), x3=%d (航向)\n",
                                 gnss_valid,
                                 gnss_speed_mps, gnss_speed_kmh, gnss_course,
                                 (int)(display_speed*1000), (int)display_course);
            }

            Serial2.printf("main.x2.val=%04d\xff\xff\xff", (int)(display_speed*100));
            Serial2.printf("main.x3.val=%04d\xff\xff\xff", (int)display_course);

            Serial2.printf("main.x4.val=%03d\xff\xff\xff", (int)(weatherData.windSpeed_m_s*10));
            Serial2.printf("main.n0.val=%d\xff\xff\xff", weatherData.windDir_deg);
            Serial2.printf("main.x5.val=%03d\xff\xff\xff", (int)(weatherData.humidity_pct*10));
            Serial2.printf("main.x6.val=%03d\xff\xff\xff", (int)(weatherData.temperature_C*10));
            Serial2.printf("main.x7.val=%03d\xff\xff\xff", (int)(weatherData.pressure_kPa*10));
            Serial2.printf("main.n4.val=%d\xff\xff\xff", weatherData.lux20_hundredLux);
            Serial2.printf("main.x9.val=%03d\xff\xff\xff", (int)(weatherData.rain_mm*10));

            Serial2.printf("main.n5.val=%d\xff\xff\xff", as7341_data2.ADCLEAR);
            Serial2.printf("main.n6.val=%d\xff\xff\xff", as7341_data1.ADNIR);

            for (int i = 0; i < 7; i++) {
                Serial2.printf("PID_R.x%d.val=%d\xff\xff\xff",
                              i*3, Glove_data.temp_now_R[i]);
            }

            for (int i = 0; i < 7; i++) {
                Serial2.printf("PID_L.x%d.val=%d\xff\xff\xff",
                              i*3, Glove_data.temp_now_L[i]);
            }

            xSemaphoreGive(mutexSerial2);
        }
    }
}
#endif

/******************************************************************************
 *                            BLE 任务
 ******************************************************************************/
#if ENABLE_BLE_TASK
void taskBLE(void *parameter) {
    SAFE_SERIAL_PRINTLN("[BLE任务] 启动");

    while (1) {
        BLE_Connect();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#endif

/******************************************************************************
 *                            初始化 FreeRTOS
 ******************************************************************************/
void initFreeRTOS() {
    Serial.println("\n=== 初始化 FreeRTOS ===");

    mutexI2C = xSemaphoreCreateMutex();
    mutexSD = xSemaphoreCreateMutex();
    mutexSerial = xSemaphoreCreateMutex();
    mutexSerial2 = xSemaphoreCreateMutex();

    if (mutexI2C == NULL || mutexSD == NULL || mutexSerial == NULL || mutexSerial2 == NULL) {
        Serial.println("错误：互斥锁创建失败！");
        while(1);
    }

    Serial.println("✓ 互斥锁创建成功");
}

/******************************************************************************
 *                            创建任务
 ******************************************************************************/
void createTasks() {
    Serial.println("\n=== 创建 FreeRTOS 任务 ===");

    BaseType_t ret;

    // IMU 任务（Core 1, 优先级 5）
    ret = xTaskCreatePinnedToCore(taskIMU, "IMU", STACK_SIZE_IMU,
                                   NULL, TASK_PRIORITY_IMU, &taskHandleIMU, 1);
    if (ret != pdPASS) {
        Serial.println("错误：IMU任务创建失败！");
        while(1);
    }
    Serial.println("✓ IMU任务 (Core 1, P5)");

#if ENABLE_GNSS_TASK
    ret = xTaskCreatePinnedToCore(taskGNSS, "GNSS", STACK_SIZE_GNSS,
                                   NULL, TASK_PRIORITY_GNSS, &taskHandleGNSS, 1);
    if (ret == pdPASS) {
        Serial.println("✓ GNSS任务 (Core 1, P4)");
    }
#else
    Serial.println("⚠️ GNSS任务已禁用");
#endif

#if ENABLE_AS7341_TASK
    ret = xTaskCreatePinnedToCore(taskAS7341, "AS7341", STACK_SIZE_AS7341,
                                   NULL, TASK_PRIORITY_AS7341, &taskHandleAS7341, 1);
    if (ret == pdPASS) {
        Serial.println("✓ AS7341任务 (Core 1, P2)");
    }
#else
    Serial.println("⚠️ AS7341任务已禁用");
#endif

#if ENABLE_RS485_TASK
    ret = xTaskCreatePinnedToCore(taskRS485, "RS485", STACK_SIZE_RS485,
                                   NULL, TASK_PRIORITY_RS485, &taskHandleRS485, 0);
    if (ret == pdPASS) {
        Serial.println("✓ RS485任务 (Core 0, P3)");
    }
#endif

    ret = xTaskCreatePinnedToCore(taskSerialRx, "SerialRx", STACK_SIZE_SERIAL_RX,
                                   NULL, TASK_PRIORITY_SERIAL_RX, &taskHandleSerialRx, 0);
    if (ret == pdPASS) {
        Serial.println("✓ 串口接收任务 (Core 0, P4)");
    }

    ret = xTaskCreatePinnedToCore(taskSDWrite, "SDWrite", STACK_SIZE_SD_WRITE,
                                   NULL, TASK_PRIORITY_SD_WRITE, &taskHandleSDWrite, 0);
    if (ret == pdPASS) {
        Serial.println("✓ SD卡任务 (Core 0, P2)");
    }

#if ENABLE_DISPLAY_TASK
    ret = xTaskCreatePinnedToCore(taskDisplay, "Display", STACK_SIZE_DISPLAY,
                                   NULL, TASK_PRIORITY_DISPLAY, &taskHandleDisplay, 0);
    if (ret == pdPASS) {
        Serial.println("✓ 显示任务 (Core 0, P1)");
    }
#endif

#if ENABLE_BLE_TASK
    ret = xTaskCreatePinnedToCore(taskBLE, "BLE", STACK_SIZE_BLE,
                                   NULL, TASK_PRIORITY_BLE, &taskHandleBLE, 0);
    if (ret == pdPASS) {
        Serial.println("✓ BLE任务 (Core 0, P2)");
    }
#endif

    Serial.println("\n✓ 任务创建完成\n");
}

/******************************************************************************
 *                            setup()
 ******************************************************************************/
void setup() {
    Serial.begin(115200);
    delay(5000);

    Serial.println("\n\n========================================");
    Serial.println("  ESP32 摩托车数据终端 - FreeRTOS");
    Serial.println("  IMU: 100Hz | 9轴融合 | 四元数重力补偿");
    Serial.println("========================================\n");

    // 初始化串口
    Serial2.begin(9600, SERIAL_8N1, 18, 17);   // 串口屏
    Serial1.begin(4800, SERIAL_8N1, 4, 5);     // 手套数据

    // 启动计数
    prefs.begin("my_app", false);
    bootCount = prefs.getUInt("boot_count", 0);
    bootCount++;
    bootname_num = bootCount;
    prefs.putUInt("boot_count", bootCount);
    prefs.end();
    Serial.printf("启动次数: %d\n", bootCount);

    // GPIO
    pinMode(16, OUTPUT);
    digitalWrite(16, HIGH);

    // BLE
    BLE_init();
    Serial.println("✓ BLE 初始化完成");

    // I2C
    Wire.begin(1, 2);
    Serial.println("✓ I2C 初始化完成");

    scanAllTCAChannels();

    // JY901
    if (!JY901_init(JY901_I2C_ADDR)) {
        Serial.println("错误：JY901 初始化失败！");
        while(1);
    }
    Serial.println("✓ JY901 初始化成功");

#if ENABLE_AS7341_TASK
    tcaSelect(4);
    if (as7341.begin() == 0) {
        Serial.println("✓ AS7341 初始化成功");
    } else {
        Serial.println("警告：AS7341 初始化失败！");
    }
    Wire.beginTransmission(TCA_ADDR);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();
#else
    Serial.println("⚠️ AS7341 已跳过");
#endif

    // SD 卡
    SD_card_init();
    Serial.println("✓ SD 卡初始化完成");

    // FreeRTOS
    initFreeRTOS();
    createTasks();

    Serial2.println("OK");
    Serial.println("\n✅ 系统启动完成\n");
}

/******************************************************************************
 *                            loop()
 ******************************************************************************/
void loop() {
    static uint32_t last_report = 0;
    uint32_t now = millis();

    // 每 30 秒打印系统状态
    if (now - last_report >= 30000) {
        last_report = now;
        SAFE_SERIAL_PRINTLN("\n=== 系统状态 ===");
        SAFE_SERIAL_PRINT("运行时间: %lu 秒\n", now / 1000);
        SAFE_SERIAL_PRINT("空闲堆: %d 字节\n", ESP.getFreeHeap());
        SAFE_SERIAL_PRINT("最小堆: %d 字节\n", ESP.getMinFreeHeap());
        SAFE_SERIAL_PRINTLN("=================\n");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
}
