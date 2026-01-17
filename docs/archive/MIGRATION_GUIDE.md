# 从原代码迁移到FreeRTOS版本 - 详细指南

## 一、迁移准备

### 1.1 备份原代码

```bash
# 创建备份
cp -r src_code src_code_backup_$(date +%Y%m%d)
```

### 1.2 理解核心变化

| 原代码机制 | FreeRTOS机制 | 优势 |
|-----------|-------------|------|
| 硬件定时器 + 中断标志位 | FreeRTOS任务 + vTaskDelayUntil | 更精确、更可靠 |
| loop()轮询 | 独立任务 | 模块化、互不阻塞 |
| delay() | vTaskDelay() | 只阻塞当前任务 |
| 全局变量 | 队列/互斥锁 | 线程安全 |
| 单核顺序执行 | 双核并行 | 性能提升 |

## 二、分步迁移流程

### 步骤1：创建基础框架

#### 1.1 添加FreeRTOS头文件

在 `main.cpp` 顶部添加：

```cpp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
```

#### 1.2 声明同步对象

```cpp
// 互斥锁
SemaphoreHandle_t mutexI2C = NULL;
SemaphoreHandle_t mutexSD = NULL;
SemaphoreHandle_t mutexSerial = NULL;

// 任务句柄
TaskHandle_t taskHandleIMU = NULL;
TaskHandle_t taskHandleAS7341 = NULL;
// ... 其他任务句柄
```

### 步骤2：移除硬件定时器

#### 2.1 删除定时器相关代码

**原代码（删除）：**
```cpp
// ❌ 删除这些
hw_timer_t *timer = NULL;
hw_timer_t *timer2 = NULL;
hw_timer_t *timer3 = NULL;

volatile bool jy901_timer_flag = false;
volatile bool as7341_timer_flag = false;
// ...

void IRAM_ATTR onTimer() {
    jy901_timer_flag = true;
    // ...
}

timer = timerBegin(0, 80, true);
timerAttachInterrupt(timer, &onTimer, true);
```

#### 2.2 删除TimerManager模块

```bash
# 如果使用独立的定时器管理器，可以移除
rm include/TimerManager.h
rm include/TimerManager_Config.h  
rm src/TimerManager.cpp
```

### 步骤3：重构IMU任务

#### 3.1 原代码的IMU处理

**原代码（在loop()中）：**
```cpp
void loop() {
    if (TIMER_IMU_FLAG()) {  // ❌ 轮询标志位
        static uint32_t last_debug_us = 0;
        uint32_t now = micros();
        
        if (last_debug_us != 0) {
            float dt_ms = (now - last_debug_us) / 1000.0f;
            Serial.printf("IMU间隔: %.2f ms\n", dt_ms);  // 实际约1800ms！
        }
        last_debug_us = now;
        
        if (tcaSelect(1)) {
            JY901_readAll(JY901_I2C_ADDR, jy_data);
            compensateGravity();
            DP_jy901(...);
            I2C_Clear();
        }
    }
    // ... 其他代码会阻塞IMU执行
}
```

#### 3.2 FreeRTOS版本（独立任务）

**新代码：**
```cpp
void taskIMU(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);  // ✅ 精确20ms
    
    uint32_t loop_count = 0;
    uint32_t last_debug_time = 0;
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);  // ✅ 精确周期等待
        
        // 获取I2C互斥锁
        if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (tcaSelect(1)) {
                JY901_readAll(JY901_I2C_ADDR, jy_data);
                compensateGravity();
                DP_jy901(jy_data.ax_mss, jy_data.ay_mss, jy_data.az_mss,
                        jy_data.pitch_deg, jy_data.roll_deg);
                I2C_Clear();
            }
            xSemaphoreGive(mutexI2C);
        }
        
        // 频率统计（每秒打印一次）
        loop_count++;
        uint32_t now = millis();
        if (now - last_debug_time >= 1000) {
            Serial.printf("[IMU] 实际频率=%.1fHz\n",
                         loop_count * 1000.0f / (now - last_debug_time));
            loop_count = 0;
            last_debug_time = now;
        }
    }
}
```

**关键改进**：
- ✅ 独立任务，不受其他代码阻塞
- ✅ `vTaskDelayUntil()` 保证精确20ms周期
- ✅ 互斥锁保护I2C访问
- ✅ 实时频率监控

### 步骤4：重构其他传感器任务

#### 4.1 AS7341任务

**原代码（loop()中）：**
```cpp
void loop() {
    if (TIMER_AS7341_FLAG()) {
        if (tcaSelect(4)) {
            as7341.startMeasure(as7341.eF1F4ClearNIR);
            as7341_data1 = as7341.readSpectralDataOne();
            // ...
            I2C_Clear();
        }
    }
}
```

**新代码（独立任务）：**
```cpp
void taskAS7341(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(200);  // 200ms
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (tcaSelect(4)) {
                as7341.startMeasure(as7341.eF1F4ClearNIR);
                as7341_data1 = as7341.readSpectralDataOne();
                
                as7341.startMeasure(as7341.eF5F8ClearNIR);
                as7341_data2 = as7341.readSpectralDataTwo();
                
                I2C_Clear();
            }
            xSemaphoreGive(mutexI2C);
        }
    }
}
```

#### 4.2 SD卡写入任务

**原代码（loop()中）：**
```cpp
void loop() {
    if (TIMER_2S_FLAG()) {
        two_sec_flag = false;  // ❌ 手动清除标志位
        
        if (SD_card_is_mounted()) {
            SD_card_append_timestamp(name, log_seconds);
            String p(name);
            File f = SD_MMC.open(p.c_str(), FILE_APPEND);
            f.printf("...");
            f.close();
        }
        log_seconds += 2;
    }
}
```

**新代码（独立任务）：**
```cpp
void taskSDWrite(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(2000);  // 2秒
    uint32_t log_seconds = 0;
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        if (SD_card_is_mounted() && sd_file_created) {
            if (xSemaphoreTake(mutexSD, pdMS_TO_TICKS(200)) == pdTRUE) {
                SD_card_append_timestamp(name, log_seconds);
                
                String p(name);
                File f = SD_MMC.open(p.c_str(), FILE_APPEND);
                
                // 写入IMU数据
                f.printf("%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\r\n",
                        jy_data.ax_mss, jy_data.ay_mss, jy_data.az_mss,
                        jy901_vx, jy901_vy, jy901_vz);
                
                // ... 其他数据
                f.close();
                
                xSemaphoreGive(mutexSD);
            }
        }
        
        log_seconds += 2;
    }
}
```

### 步骤5：重构串口接收

#### 5.1 原代码（loop()中轮询）

```cpp
void loop() {
    if (Serial1.available()) {  // ❌ 轮询
        receiveAsString();
        // ... 处理手套数据
        
        // 写入SD卡（无保护！）
        File f = SD_MMC.open(p.c_str(), FILE_APPEND);
        f.printf("...");
        f.close();
    }
}
```

#### 5.2 FreeRTOS版本

```cpp
void taskSerialRx(void *parameter) {
    while (1) {
        // 处理手套数据
        if (Serial1.available()) {
            receiveAsString();
            
            // 切换手套标志
            if (Golve_flag == 1) {
                digitalWrite(16, HIGH);
                Golve_flag = 0;
            } else {
                digitalWrite(16, LOW);
                Golve_flag = 1;
            }
            
            // 写入SD卡（带互斥保护）
            if (xSemaphoreTake(mutexSD, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (SD_card_is_mounted() && sd_file_created) {
                    String p(name);
                    File f = SD_MMC.open(p.c_str(), FILE_APPEND);
                    
                    if (Glove_data.RL == 'R') {
                        f.printf("R,%01d,%02d:...", ...);
                    }
                    // ...
                    f.close();
                }
                xSemaphoreGive(mutexSD);
            }
        }
        
        // 处理CKP数据
        if (Serial2.available()) {
            CKP_receive();
            // ...
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 短暂延时
    }
}
```

### 步骤6：修改setup()函数

#### 6.1 原代码setup()

```cpp
void setup() {
    Serial.begin(115600);
    // ... 各种初始化
    
    // ❌ 初始化定时器
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 200000, true);
    timerAlarmEnable(timer);
    
    // ... 其他初始化
}
```

#### 6.2 FreeRTOS版本setup()

```cpp
void setup() {
    Serial.begin(115600);
    delay(500);
    
    Serial.println("\n=== ESP32数据终端 - FreeRTOS版本 ===\n");
    
    // ... 所有硬件初始化（和原代码相同）
    Serial2.begin(9600, SERIAL_8N1, 18, 17);
    Serial1.begin(4800, SERIAL_8N1, 4, 5);
    
    // NVS初始化
    prefs.begin("my_app", false);
    bootCount = prefs.getUInt("boot_count", 0);
    bootCount++;
    prefs.putUInt("boot_count", bootCount);
    prefs.end();
    
    // GPIO初始化
    pinMode(16, OUTPUT);
    digitalWrite(16, HIGH);
    
    // BLE初始化
    BLE_init();
    
    // I2C初始化
    Wire.begin(1, 2);
    scanAllTCAChannels();
    
    // JY901初始化
    if (!JY901_init(JY901_I2C_ADDR)) {
        Serial.println("错误：JY901初始化失败！");
        while(1);
    }
    
    // AS7341初始化
    tcaSelect(4);
    while (as7341.begin() != 0) {
        delay(500);
    }
    
    // SD卡初始化
    SD_card_init();
    
    // ✅ 新增：初始化FreeRTOS对象
    Serial.println("\n=== 初始化FreeRTOS ===");
    
    // 创建互斥锁
    mutexI2C = xSemaphoreCreateMutex();
    mutexSD = xSemaphoreCreateMutex();
    mutexSerial = xSemaphoreCreateMutex();
    
    if (mutexI2C == NULL || mutexSD == NULL || mutexSerial == NULL) {
        Serial.println("错误：无法创建互斥锁！");
        while(1);
    }
    
    // ✅ 新增：创建所有任务
    createTasks();
    
    Serial2.println("OK");
    Serial.println("\n系统启动完成！\n");
}
```

### 步骤7：简化loop()函数

#### 7.1 原代码loop()（复杂）

```cpp
void loop() {
    // ❌ 大量轮询代码（100+行）
    if (Serial.available()) { ... }
    if (Serial1.available()) { ... }
    if (Serial2.available()) { ... }
    
    BLE_Connect();
    
    if (TIMER_SD_MOUNT_FLAG()) { ... }
    if (TIMER_2S_FLAG()) { ... }
    if (TIMER_485_FLAG()) { ... }
    if (TIMER_IMU_FLAG()) { ... }
    if (TIMER_AS7341_FLAG()) { ... }
    if (TIMER_5S_FLAG()) { ... }
    
    delay(1);  // ❌ 阻塞整个系统
}
```

#### 7.2 FreeRTOS版本loop()（简化）

```cpp
void loop() {
    // ✅ loop()基本为空，所有工作由任务完成
    
    // 可选：系统监控
    static uint32_t last_report = 0;
    uint32_t now = millis();
    
    if (now - last_report >= 30000) {  // 每30秒
        last_report = now;
        Serial.println("\n=== 系统状态 ===");
        Serial.printf("运行时间: %lu 秒\n", now / 1000);
        Serial.printf("空闲堆内存: %d 字节\n", ESP.getFreeHeap());
        Serial.println("================\n");
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));  // 1秒延时
}
```

## 三、验证和调试

### 3.1 编译检查

```bash
# Arduino IDE: 点击"验证"按钮
# PlatformIO:
pio run
```

**常见编译错误**：

1. **缺少头文件**
   ```
   error: 'xTaskCreate' was not declared in this scope
   ```
   **解决**：添加 `#include "freertos/FreeRTOS.h"` 和 `#include "freertos/task.h"`

2. **堆栈大小单位错误**
   ```cpp
   // ❌ 错误（FreeRTOS原版使用字）
   xTaskCreate(task, "name", 1024, ...);  
   
   // ✅ 正确（ESP-IDF使用字节）
   xTaskCreate(task, "name", 4096, ...);  
   ```

### 3.2 运行时验证

#### 检查IMU频率

在IMU任务中添加频率统计：

```cpp
void taskIMU(void *parameter) {
    uint32_t loop_count = 0;
    uint32_t last_time = millis();
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        // ... IMU读取代码
        
        loop_count++;
        if (millis() - last_time >= 1000) {
            float freq = loop_count * 1000.0f / (millis() - last_time);
            Serial.printf("[IMU] 频率=%.1fHz (目标50Hz)\n", freq);
            
            loop_count = 0;
            last_time = millis();
        }
    }
}
```

**期望输出**：
```
[IMU] 频率=50.0Hz (目标50Hz)  ✅ 正常
[IMU] 频率=49.8Hz (目标50Hz)  ✅ 正常
[IMU] 频率=50.2Hz (目标50Hz)  ✅ 正常
```

**异常情况**：
```
[IMU] 频率=25.3Hz (目标50Hz)  ❌ 异常，可能被阻塞
[IMU] 频率=0.5Hz (目标50Hz)   ❌ 严重问题！
```

#### 检查堆栈使用

```cpp
void taskIMU(void *parameter) {
    while (1) {
        // ...
        
        // 定期检查堆栈剩余
        static uint32_t last_check = 0;
        if (millis() - last_check >= 10000) {  // 每10秒
            UBaseType_t stack_free = uxTaskGetStackHighWaterMark(NULL);
            Serial.printf("[IMU] 堆栈剩余: %u 字节\n", stack_free * sizeof(StackType_t));
            last_check = millis();
        }
    }
}
```

**期望输出**：
```
[IMU] 堆栈剩余: 2048 字节  ✅ 正常（50%使用率）
```

**异常情况**：
```
[IMU] 堆栈剩余: 128 字节   ❌ 警告：堆栈快用完！
[IMU] 堆栈剩余: 0 字节     ❌ 危险：即将溢出！
```

### 3.3 调试工具

#### 使用串口监视器

```cpp
// 在各个任务中添加调试输出
Serial.printf("[IMU] 开始执行\n");
Serial.printf("[AS7341] 数据读取完成\n");
Serial.printf("[SD] 写入成功\n");
```

#### 使用LED指示

```cpp
#define LED_IMU    2
#define LED_ERROR  4

void taskIMU(void *parameter) {
    pinMode(LED_IMU, OUTPUT);
    
    while (1) {
        digitalWrite(LED_IMU, HIGH);  // 任务运行
        
        // ... IMU代码
        
        digitalWrite(LED_IMU, LOW);
        vTaskDelayUntil(...);
    }
}
```

## 四、性能对比测试

### 4.1 测试方法

#### 方法1：串口输出时间戳

```cpp
// 在IMU任务中
void taskIMU(void *parameter) {
    uint32_t last_time = micros();
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        uint32_t now = micros();
        uint32_t interval = now - last_time;
        last_time = now;
        
        Serial.printf("IMU间隔: %u 微秒 (%.2f ms)\n", 
                     interval, interval / 1000.0f);
    }
}
```

**原代码输出**：
```
IMU间隔: 1823456 微秒 (1823.46 ms)  ❌
IMU间隔: 1798234 微秒 (1798.23 ms)  ❌
IMU间隔: 1856789 微秒 (1856.79 ms)  ❌
```

**FreeRTOS版本输出**：
```
IMU间隔: 20012 微秒 (20.01 ms)  ✅
IMU间隔: 19998 微秒 (19.99 ms)  ✅
IMU间隔: 20005 微秒 (20.00 ms)  ✅
```

#### 方法2：SD卡日志分析

检查SD卡中的时间戳间隔：

```
# 原代码SD卡日志（时间戳间隔不固定）
00:00:02  TS
...数据...
00:00:05  TS  ❌ 间隔3秒（应该是2秒）
...数据...
00:00:09  TS  ❌ 间隔4秒
```

```
# FreeRTOS版本SD卡日志（时间戳间隔精确）
00:00:02  TS
...数据...
00:00:04  TS  ✅ 间隔2秒
...数据...
00:00:06  TS  ✅ 间隔2秒
```

## 五、常见问题排查

### 问题1：编译错误 - 未定义的引用

**错误信息**：
```
undefined reference to `xTaskCreate'
```

**原因**：缺少FreeRTOS库链接

**解决方案**：
- Arduino IDE：确保选择了ESP32开发板
- PlatformIO：在 `platformio.ini` 中添加：
  ```ini
  framework = arduino
  platform = espressif32
  ```

### 问题2：运行时崩溃 - 堆栈溢出

**症状**：
- 系统频繁重启
- 串口输出异常字符
- 看门狗复位

**调试**：
```cpp
// 启用堆栈溢出检测
// menuconfig → FreeRTOS → Port → Check for stack overflow = Method 2
```

**解决方案**：
```cpp
// 增加堆栈大小
#define STACK_SIZE_IMU  8192  // 从4096增加到8192
```

### 问题3：IMU频率仍然不对

**可能原因**：

1. **configTICK_RATE_HZ 太低**
   ```
   # menuconfig → FreeRTOS → Kernel
   configTICK_RATE_HZ = 1000  (不要使用100)
   ```

2. **I2C被长时间占用**
   - 检查AS7341任务是否执行时间过长
   - 减少AS7341的采样频率
   - 增加互斥锁超时时间

3. **优先级设置错误**
   ```cpp
   // 确保IMU任务优先级最高
   #define TASK_PRIORITY_IMU  5  // 最高
   #define TASK_PRIORITY_AS7341  3  // 较低
   ```

### 问题4：SD卡写入失败

**症状**：
```
[SD] 无法打开文件
```

**调试**：
```cpp
if (xSemaphoreTake(mutexSD, pdMS_TO_TICKS(500)) == pdTRUE) {
    Serial.println("[SD] 获取互斥锁成功");
    
    File f = SD_MMC.open(name, FILE_APPEND);
    if (!f) {
        Serial.println("[SD] 错误：无法打开文件");
    } else {
        Serial.println("[SD] 文件打开成功");
        f.printf("test");
        f.close();
    }
    
    xSemaphoreGive(mutexSD);
} else {
    Serial.println("[SD] 错误：无法获取互斥锁");
}
```

## 六、优化建议

### 6.1 使用队列代替全局变量

**当前方案（全局变量）**：
```cpp
JY901_Data jy_data;  // 全局变量

void taskIMU() {
    JY901_readAll(JY901_I2C_ADDR, jy_data);  // 直接写入
}

void taskSDWrite() {
    // 直接读取
    f.printf("%.3f", jy_data.ax_mss);
}
```

**改进方案（队列）**：
```cpp
QueueHandle_t queueIMU;

void setup() {
    queueIMU = xQueueCreate(10, sizeof(JY901_Data));
}

void taskIMU() {
    JY901_Data data;
    JY901_readAll(JY901_I2C_ADDR, data);
    
    // 发送到队列
    xQueueSend(queueIMU, &data, 0);
}

void taskSDWrite() {
    JY901_Data data;
    
    // 从队列接收
    if (xQueueReceive(queueIMU, &data, portMAX_DELAY) == pdTRUE) {
        f.printf("%.3f", data.ax_mss);
    }
}
```

**优势**：
- 线程安全
- 自动缓冲
- 解耦任务

### 6.2 使用软件定时器

对于不需要高精度的任务，可以使用软件定时器代替任务：

```cpp
TimerHandle_t displayTimer;

void displayTimerCallback(TimerHandle_t xTimer) {
    // 更新显示
    Serial2.printf("...");
}

void setup() {
    // 创建5秒软件定时器
    displayTimer = xTimerCreate(
        "DisplayTimer",
        pdMS_TO_TICKS(5000),
        pdTRUE,  // 自动重载
        NULL,
        displayTimerCallback
    );
    
    xTimerStart(displayTimer, 0);
}
```

### 6.3 动态调整任务优先级

根据系统负载动态调整：

```cpp
void taskMonitor() {
    while (1) {
        // 检查堆内存
        if (ESP.getFreeHeap() < 10000) {
            // 内存紧张，降低非关键任务优先级
            vTaskPrioritySet(taskHandleDisplay, 0);
        } else {
            vTaskPrioritySet(taskHandleDisplay, 1);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
```

## 七、总结

### 迁移检查清单

- ✅ 添加FreeRTOS头文件
- ✅ 创建互斥锁和任务句柄
- ✅ 移除硬件定时器代码
- ✅ 移除定时器标志位
- ✅ 将loop()中的代码移到任务
- ✅ 替换所有delay()为vTaskDelay()
- ✅ 添加互斥锁保护共享资源
- ✅ 创建所有任务
- ✅ 简化loop()函数
- ✅ 测试IMU频率
- ✅ 检查堆栈使用
- ✅ 验证SD卡写入

### 预期改进

| 指标 | 原代码 | FreeRTOS | 改善倍数 |
|-----|--------|----------|---------|
| IMU采样周期 | ~1800ms | 20ms | **90倍** |
| 采样精度 | 不稳定 | ±1ms | 显著提升 |
| 系统响应 | 阻塞严重 | 并行处理 | 明显改善 |
| 代码可维护性 | 复杂 | 模块化 | 大幅提升 |

恭喜！你已经成功将项目迁移到FreeRTOS，解决了IMU时序问题！
