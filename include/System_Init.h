/******************************************************************************
 *                   系统初始化函数 - 头文件
 ******************************************************************************/

#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/******************************************************************************
 *                            FreeRTOS 同步对象
 ******************************************************************************/
extern SemaphoreHandle_t mutexI2C;
extern SemaphoreHandle_t mutexSD;
extern SemaphoreHandle_t mutexSerial;
extern SemaphoreHandle_t mutexSerial2;

/******************************************************************************
 *                            任务句柄
 ******************************************************************************/
extern TaskHandle_t taskHandleIMU;
extern TaskHandle_t taskHandleAS7341;
extern TaskHandle_t taskHandleRS485;
extern TaskHandle_t taskHandleSerialRx;
extern TaskHandle_t taskHandleSDWrite;
extern TaskHandle_t taskHandleDisplay;
extern TaskHandle_t taskHandleBLE;

/******************************************************************************
 *                            初始化函数
 ******************************************************************************/

/**
 * @brief 初始化 FreeRTOS 互斥锁
 */
void initFreeRTOS();

/**
 * @brief 创建所有 FreeRTOS 任务
 */
void createTasks();

/**
 * @brief 初始化硬件外设
 *
 * 包括：
 * - 串口
 * - I2C
 * - BLE
 * - JY901
 * - AS7341（可选）
 * - SD 卡
 */
void initHardware();

#endif // SYSTEM_INIT_H
