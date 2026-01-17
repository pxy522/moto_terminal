/******************************************************************************
 *                   IMU 任务和处理函数 - 实现
 ******************************************************************************/

#include "IMU_Tasks.h"

/******************************************************************************
 *                            IMU 处理函数
 ******************************************************************************/

/**
 * @brief ✅ 正确的 IMU 处理算法 - 使用 JY901 内部姿态解算
 *
 * 关键改进:
 * 1. 直接使用 JY901 内部卡尔曼滤波器输出的姿态角（roll, pitch, yaw）
 * 2. 用四元数精确去除重力分量
 * 3. 不重复解算姿态，避免数据损失和滤波器冲突
 * 4. 保留原始 jy_data 数据不做修改
 */
void processIMU_Correct() {
    static uint32_t last_time_us = 0;
    uint32_t current_time_us = micros();

    // 计算时间步长
    float dt;
    if (last_time_us == 0) {
        last_time_us = current_time_us;
        dt = 0.01f;  // 假设 100Hz
        return;  // 第一次调用，跳过
    } else {
        if (current_time_us >= last_time_us) {
            dt = (current_time_us - last_time_us) / 1000000.0f;
        } else {
            dt = ((0xFFFFFFFF - last_time_us) + current_time_us + 1) / 1000000.0f;
        }
        last_time_us = current_time_us;
    }

    // 限制 dt 范围（防止异常值）
    if (dt < 0.005f || dt > 0.1f) {
        dt = 0.01f;  // 默认 100Hz
    }

    // ========== 步骤 1: 使用 JY901 内部解算的姿态角 ==========
    // JY901 内部已经用卡尔曼滤波融合了 9 轴数据（加速度+陀螺仪+磁力计）
    // 这些角度是最优的，不需要重新解算
    float roll = jy_data.roll_deg;
    float pitch = jy_data.pitch_deg;
    float yaw = jy_data.yaw_deg;

    // 转换为四元数（用于精确的向量旋转）
    imuFusion.setEulerAngles(roll, pitch, yaw);

    // ========== 步骤 2: 用四元数精确去除重力 ==========
    // 不修改原始 jy_data，使用局部变量
    Vector3 linearAccel = imuFusion.removeGravity(jy_data.ax_mss,
                                                   jy_data.ay_mss,
                                                   jy_data.az_mss);

    // 存储重力分量（供诊断）
    const float G = 9.80665f;
    gx_calculated = jy_data.ax_mss - linearAccel.x;
    gy_calculated = jy_data.ay_mss - linearAccel.y;
    gz_calculated = jy_data.az_mss - linearAccel.z;

    // ========== 步骤 3: 速度积分 ==========
    imuFusion.updateVelocity(linearAccel.x, linearAccel.y, linearAccel.z, dt);

    // ========== 步骤 4: 零速修正 (ZUPT) ==========
    imuFusion.applyZeroVelocityUpdate();

    // ========== 步骤 5: 更新全局速度变量（兼容旧代码）==========
    Vector3 velocity = imuFusion.getVelocity();
    jy901_vx = velocity.x;
    jy901_vy = velocity.y;
    jy901_vz = velocity.z;
}

/**
 * @brief 📊 简化诊断打印 - 只显示频率和速度
 */
void IMU_DiagnosePrint_Correct() {
    static uint32_t print_count = 0;
    static uint32_t last_print_time = 0;

    print_count++;

    // 每 100 次打印一次（100Hz 下约 1 秒）
    if (print_count % 100 != 0) return;

    // 计算实际频率
    uint32_t current_time = millis();
    float actual_freq = 0.0f;

    if (last_print_time > 0) {
        float elapsed_sec = (current_time - last_print_time) / 1000.0f;
        actual_freq = 100.0f / elapsed_sec;  // 100次调用的频率
    }
    last_print_time = current_time;

    // 只打印关键信息：频率和速度
    float vel_mag = sqrtf(jy901_vx * jy901_vx +
                          jy901_vy * jy901_vy +
                          jy901_vz * jy901_vz);

    Serial.printf("[IMU] 频率=%.1fHz | 速度=%.2fm/s (%.1fkm/h) | vx=%.2f vy=%.2f vz=%.2f\n",
                  actual_freq, vel_mag, vel_mag * 3.6f, jy901_vx, jy901_vy, jy901_vz);
}
