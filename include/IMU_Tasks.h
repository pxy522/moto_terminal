/******************************************************************************
 *                   IMU 任务和处理函数 - 头文件
 *
 *  包含正确的 IMU 处理算法（使用 JY901 内部姿态解算）
 ******************************************************************************/

#ifndef IMU_TASKS_H
#define IMU_TASKS_H

#include <Arduino.h>
#include "REG_JY901.h"
#include "IMU_Fusion.h"

/******************************************************************************
 *                            全局变量声明
 ******************************************************************************/
// 外部速度变量（兼容旧代码）
extern float jy901_vx;
extern float jy901_vy;
extern float jy901_vz;

// 重力分量（供诊断使用）
extern float gx_calculated;
extern float gy_calculated;
extern float gz_calculated;

// JY901 数据
extern JY901_Data jy_data;

// IMU 融合对象
extern IMU_Fusion imuFusion;

/******************************************************************************
 *                            IMU 处理函数
 ******************************************************************************/

/**
 * @brief ✅ 正确的 IMU 处理算法
 *
 * 使用 JY901 内部姿态解算 + 四元数精确去重力
 *
 * 工作流程:
 * 1. 直接使用 JY901 输出的 roll/pitch/yaw（9轴卡尔曼滤波）
 * 2. 转换为四元数
 * 3. 用四元数精确去除重力
 * 4. 速度积分
 * 5. 零速修正（ZUPT）
 */
void processIMU_Correct();

/**
 * @brief 📊 IMU 诊断打印（正确算法）
 *
 * 每秒打印一次 IMU 状态，包括：
 * - JY901 姿态角
 * - 原始加速度
 * - 重力分量
 * - 线性加速度
 * - 当前速度
 */
void IMU_DiagnosePrint_Correct();

#endif // IMU_TASKS_H
