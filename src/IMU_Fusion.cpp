/******************************************************************************
 *                   IMU 传感器融合实现 - 基于 Madgwick AHRS
 *
 *  算法来源: Sebastian Madgwick, "An efficient orientation filter for
 *            inertial and inertial/magnetic sensor arrays" (2010)
 *
 *  参考资料:
 *  - https://github.com/arduino-libraries/MadgwickAHRS
 *  - https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 ******************************************************************************/

#include "IMU_Fusion.h"

/******************************************************************************
 *                            构造函数和初始化
 ******************************************************************************/

IMU_Fusion::IMU_Fusion(float sampleFrequency, float betaGain) {
    sampleFreq = sampleFrequency;
    invSampleFreq = 1.0f / sampleFreq;
    beta = betaGain;

    // 默认参数（针对摩托车优化）
    lpfAlpha = 0.5f;  // 低通滤波系数（提高响应速度）
    decayIdle = 0.998f;  // 怠速衰减 0.2% (低速时适度衰减)
    decayMoving = 0.99995f;  // 行驶衰减 0.005% (高速时几乎不衰减)
    stationaryThresholdAcc = 0.15f;  // 0.15 m/s² (摩托车怠速振动约 0.1)
    stationaryThresholdVel = 0.08f;  // 0.08 m/s (0.3 km/h)

    reset();
}

void IMU_Fusion::begin() {
    reset();
    Serial.println("[IMU_Fusion] 初始化完成");
}

void IMU_Fusion::reset() {
    // 初始化四元数为单位四元数 (无旋转)
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;

    // 重置速度和位置
    velocity.x = velocity.y = velocity.z = 0.0f;
    position.x = position.y = position.z = 0.0f;

    // 重置滤波器状态
    lpfAccel.x = lpfAccel.y = lpfAccel.z = 0.0f;

    // 重置校准
    accelBias.x = accelBias.y = accelBias.z = 0.0f;
    isCalibrated = false;

    // 重置计数器
    stationaryCount = 0;
    movingCount = 0;
}

/******************************************************************************
 *                            参数设置
 ******************************************************************************/

void IMU_Fusion::setBeta(float betaGain) {
    beta = betaGain;
}

void IMU_Fusion::setSampleFrequency(float freq) {
    sampleFreq = freq;
    invSampleFreq = 1.0f / freq;
}

void IMU_Fusion::setLowPassAlpha(float alpha) {
    lpfAlpha = constrain(alpha, 0.0f, 1.0f);
}

void IMU_Fusion::setDecayFactors(float idle, float moving) {
    decayIdle = idle;
    decayMoving = moving;
}

void IMU_Fusion::setStationaryThresholds(float accThreshold, float velThreshold) {
    stationaryThresholdAcc = accThreshold;
    stationaryThresholdVel = velThreshold;
}

/******************************************************************************
 *                            加速度计校准
 ******************************************************************************/

void IMU_Fusion::startCalibration() {
    accelBias.x = 0.0f;
    accelBias.y = 0.0f;
    accelBias.z = 0.0f;
    isCalibrated = false;
    Serial.println("[IMU_Fusion] 开始校准，请保持设备静止...");
}

void IMU_Fusion::addCalibrationSample(float ax, float ay, float az) {
    accelBias.x += ax;
    accelBias.y += ay;
    accelBias.z += az;
}

bool IMU_Fusion::finishCalibration(uint16_t numSamples) {
    if (numSamples == 0) {
        Serial.println("[IMU_Fusion] 错误：校准样本数为0");
        return false;
    }

    accelBias.x /= numSamples;
    accelBias.y /= numSamples;
    accelBias.z /= numSamples;
    isCalibrated = true;

    Serial.printf("[IMU_Fusion] 校准完成: 偏置=(%.4f, %.4f, %.4f) m/s²\n",
                  accelBias.x, accelBias.y, accelBias.z);
    return true;
}

/******************************************************************************
 *                            Madgwick AHRS 算法核心
 ******************************************************************************/

/**
 * @brief 快速平方根倒数近似 (Quake III 算法)
 */
float IMU_Fusion::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 四元数归一化
 */
void IMU_Fusion::quaternionNormalize() {
    float recipNorm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;
}

/**
 * @brief Madgwick 6DOF IMU 更新算法
 *
 * @param gx,gy,gz 陀螺仪角速度 (rad/s)
 * @param ax,ay,az 加速度计 (m/s²，已归一化)
 * @param dt 时间步长 (s)
 */
void IMU_Fusion::madgwickUpdateIMU(float gx, float gy, float gz,
                                   float ax, float ay, float az,
                                   float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // 四元数变量简化
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;

    // 如果加速度为0，跳过梯度下降（只用陀螺仪）
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 归一化加速度计数据
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 辅助变量，避免重复计算
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // 梯度下降算法：最小化加速度测量与重力方向的误差
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 陀螺仪积分 - 梯度下降修正
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    } else {
        // 只使用陀螺仪（无加速度计修正）
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    }

    // 积分四元数变化率
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // 更新四元数
    q.w = q0;
    q.x = q1;
    q.y = q2;
    q.z = q3;

    // 归一化四元数
    quaternionNormalize();
}

/******************************************************************************
 *                            姿态更新
 ******************************************************************************/

/**
 * @brief 更新姿态估计
 *
 * @param gx_dps,gy_dps,gz_dps 陀螺仪角速度 (度/秒)
 * @param ax_mss,ay_mss,az_mss 加速度 (m/s²，去重力前)
 * @param dt 时间步长 (秒)
 */
void IMU_Fusion::update(float gx_dps, float gy_dps, float gz_dps,
                        float ax_mss, float ay_mss, float az_mss,
                        float dt) {
    // 转换角速度：度/秒 -> 弧度/秒
    float gx_rad = gx_dps * DEG_TO_RAD;
    float gy_rad = gy_dps * DEG_TO_RAD;
    float gz_rad = gz_dps * DEG_TO_RAD;

    // 执行 Madgwick 算法更新姿态
    madgwickUpdateIMU(gx_rad, gy_rad, gz_rad, ax_mss, ay_mss, az_mss, dt);
}

/******************************************************************************
 *                            重力补偿
 ******************************************************************************/

/**
 * @brief 使用四元数旋转去除重力分量
 *
 * @param ax_mss,ay_mss,az_mss 原始加速度 (m/s²)
 * @return 去除重力后的加速度向量
 */
Vector3 IMU_Fusion::removeGravity(float ax_mss, float ay_mss, float az_mss) {
    // 重力向量 (世界坐标系，垂直向下)
    const float G = 9.80665f;
    Vector3 gravity = {0.0f, 0.0f, G};  // 假设 Z 轴向上

    // 将重力向量旋转到传感器坐标系
    // 使用四元数的逆旋转 (共轭四元数)
    Vector3 gravityBody = quaternionRotateVectorInverse(gravity);

    // 从测量加速度中减去重力分量
    Vector3 linearAccel;
    linearAccel.x = ax_mss - gravityBody.x;
    linearAccel.y = ay_mss - gravityBody.y;
    linearAccel.z = az_mss - gravityBody.z;

    return linearAccel;
}

/**
 * @brief 使用四元数旋转向量 (世界坐标 -> 传感器坐标)
 */
Vector3 IMU_Fusion::quaternionRotateVectorInverse(const Vector3& v) {
    // 使用共轭四元数 q* = (w, -x, -y, -z)
    float qw = q.w, qx = -q.x, qy = -q.y, qz = -q.z;

    // 四元数乘法: q* * v * q
    float ix = qw * v.x + qy * v.z - qz * v.y;
    float iy = qw * v.y + qz * v.x - qx * v.z;
    float iz = qw * v.z + qx * v.y - qy * v.x;
    float iw = -qx * v.x - qy * v.y - qz * v.z;

    Vector3 result;
    result.x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
    result.y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
    result.z = iz * qw + iw * -qz + ix * -qy - iy * -qx;

    return result;
}

/**
 * @brief 使用四元数旋转向量 (传感器坐标 -> 世界坐标)
 */
Vector3 IMU_Fusion::quaternionRotateVector(const Vector3& v) {
    // 四元数乘法: q * v * q*
    float ix = q.w * v.x + q.y * v.z - q.z * v.y;
    float iy = q.w * v.y + q.z * v.x - q.x * v.z;
    float iz = q.w * v.z + q.x * v.y - q.y * v.x;
    float iw = -q.x * v.x - q.y * v.y - q.z * v.z;

    Vector3 result;
    result.x = ix * q.w + iw * -q.x + iy * -q.z - iz * -q.y;
    result.y = iy * q.w + iw * -q.y + iz * -q.x - ix * -q.z;
    result.z = iz * q.w + iw * -q.z + ix * -q.y - iy * -q.x;

    return result;
}

/******************************************************************************
 *                            速度解算
 ******************************************************************************/

/**
 * @brief 更新速度估计
 *
 * @param ax_mss,ay_mss,az_mss 去重力后的加速度 (m/s²)
 * @param dt 时间步长 (秒)
 */
void IMU_Fusion::updateVelocity(float ax_mss, float ay_mss, float az_mss, float dt) {
    // 减去校准偏置
    if (isCalibrated) {
        ax_mss -= accelBias.x;
        ay_mss -= accelBias.y;
        az_mss -= accelBias.z;
    }

    // 低通滤波
    lpfAccel.x = lpfAlpha * ax_mss + (1.0f - lpfAlpha) * lpfAccel.x;
    lpfAccel.y = lpfAlpha * ay_mss + (1.0f - lpfAlpha) * lpfAccel.y;
    lpfAccel.z = lpfAlpha * az_mss + (1.0f - lpfAlpha) * lpfAccel.z;

    // 死区处理 (应对噪声) - 摩托车优化
    const float DEADZONE = 0.08f;  // 0.08 m/s² (降低死区，保留匀速时的微小加速度)
    float ax = (fabs(lpfAccel.x) < DEADZONE) ? 0.0f : lpfAccel.x;
    float ay = (fabs(lpfAccel.y) < DEADZONE) ? 0.0f : lpfAccel.y;
    float az = (fabs(lpfAccel.z) < DEADZONE) ? 0.0f : lpfAccel.z;

    // 速度积分
    velocity.x += ax * dt;
    velocity.y += ay * dt;
    velocity.z += az * dt;

    // 位置积分 (可选)
    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
    position.z += velocity.z * dt;

    // 速度衰减 (对抗漂移) - 摩托车优化
    float speed = getSpeed();

    // 智能衰减：根据速度大小选择衰减系数
    if (speed < 1.0f) {  // 极低速/怠速 (<3.6 km/h)
        velocity.x *= decayIdle;   // 0.998 = 每秒衰减约 2%
        velocity.y *= decayIdle;
        velocity.z *= decayIdle;
    } else {  // 正常行驶 (>3.6 km/h)
        velocity.x *= decayMoving;  // 0.99995 = 每秒衰减约 0.5%
        velocity.y *= decayMoving;
        velocity.z *= decayMoving;
    }

    // 速度限幅
    const float MAX_SPEED = 60.0f;  // 60 m/s = 216 km/h
    velocity.x = constrain(velocity.x, -MAX_SPEED, MAX_SPEED);
    velocity.y = constrain(velocity.y, -MAX_SPEED, MAX_SPEED);
    velocity.z = constrain(velocity.z, -MAX_SPEED, MAX_SPEED);
}

/**
 * @brief 零速修正 (ZUPT)
 */
void IMU_Fusion::applyZeroVelocityUpdate() {
    float speed = getSpeed();
    float accelMag = sqrt(lpfAccel.x * lpfAccel.x +
                         lpfAccel.y * lpfAccel.y +
                         lpfAccel.z * lpfAccel.z);

    // 检测静止状态
    bool isStationary = (accelMag < stationaryThresholdAcc) &&
                       (speed < stationaryThresholdVel);

    if (isStationary) {
        stationaryCount++;
        movingCount = 0;

        // 持续静止 1 秒 (100 * 10ms @ 100Hz)
        if (stationaryCount >= 100) {
            resetVelocity();
            stationaryCount = 100;  // 防止溢出
        }
    } else {
        stationaryCount = 0;
        movingCount++;
    }
}

/******************************************************************************
 *                            姿态输出
 ******************************************************************************/

/**
 * @brief 欧拉角转四元数（ZYX 顺序，航空惯例）
 *
 * @param roll_deg 横滚角 (度)
 * @param pitch_deg 俯仰角 (度)
 * @param yaw_deg 航向角 (度)
 * @return Quaternion 四元数
 */
Quaternion IMU_Fusion::eulerToQuaternion(float roll_deg, float pitch_deg, float yaw_deg) {
    // 转换为弧度
    float roll = roll_deg * DEG_TO_RAD;
    float pitch = pitch_deg * DEG_TO_RAD;
    float yaw = yaw_deg * DEG_TO_RAD;

    // 计算半角
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    // ZYX 旋转顺序（航空惯例）
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

/**
 * @brief 从欧拉角设置四元数姿态
 *
 * @param roll_deg 横滚角 (度)
 * @param pitch_deg 俯仰角 (度)
 * @param yaw_deg 航向角 (度)
 */
void IMU_Fusion::setEulerAngles(float roll_deg, float pitch_deg, float yaw_deg) {
    q = eulerToQuaternion(roll_deg, pitch_deg, yaw_deg);
    quaternionNormalize();
}

/**
 * @brief 四元数转欧拉角
 */
EulerAngles IMU_Fusion::getEulerAngles() {
    EulerAngles angles;

    // Roll (X轴旋转)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch (Y轴旋转)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1.0f) {
        angles.pitch = copysign(90.0f, sinp);  // 万向节锁
    } else {
        angles.pitch = asin(sinp) * RAD_TO_DEG;
    }

    // Yaw (Z轴旋转)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;

    return angles;
}

/**
 * @brief 四元数转旋转矩阵
 */
void IMU_Fusion::getRotationMatrix(float R[3][3]) {
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;

    R[0][0] = 1.0f - 2.0f * (q2*q2 + q3*q3);
    R[0][1] = 2.0f * (q1*q2 - q0*q3);
    R[0][2] = 2.0f * (q1*q3 + q0*q2);

    R[1][0] = 2.0f * (q1*q2 + q0*q3);
    R[1][1] = 1.0f - 2.0f * (q1*q1 + q3*q3);
    R[1][2] = 2.0f * (q2*q3 - q0*q1);

    R[2][0] = 2.0f * (q1*q3 - q0*q2);
    R[2][1] = 2.0f * (q2*q3 + q0*q1);
    R[2][2] = 1.0f - 2.0f * (q1*q1 + q2*q2);
}

/******************************************************************************
 *                            速度输出
 ******************************************************************************/

float IMU_Fusion::getSpeed() {
    return sqrt(velocity.x * velocity.x +
                velocity.y * velocity.y +
                velocity.z * velocity.z);
}

float IMU_Fusion::getSpeedKmh() {
    return getSpeed() * 3.6f;  // m/s -> km/h
}

void IMU_Fusion::resetVelocity() {
    velocity.x = velocity.y = velocity.z = 0.0f;
}

void IMU_Fusion::resetPosition() {
    position.x = position.y = position.z = 0.0f;
}

/******************************************************************************
 *                            全局实例
 ******************************************************************************/

// 创建全局实例 (50Hz 采样, beta=0.05)
IMU_Fusion imuFusion(50.0f, 0.05f);
