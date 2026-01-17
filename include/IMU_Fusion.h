/******************************************************************************
 *                   IMU 传感器融合库 - 基于 Madgwick AHRS
 *                   适用于摩托车运动追踪
 ******************************************************************************/

#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include <Arduino.h>
#include <math.h>

/******************************************************************************
 *                            四元数结构体
 ******************************************************************************/
typedef struct {
    float w;  // 标量部分
    float x;  // 向量部分 i
    float y;  // 向量部分 j
    float z;  // 向量部分 k
} Quaternion;

/******************************************************************************
 *                            欧拉角结构体
 ******************************************************************************/
typedef struct {
    float roll;   // 横滚角 (度)
    float pitch;  // 俯仰角 (度)
    float yaw;    // 航向角 (度)
} EulerAngles;

/******************************************************************************
 *                            三轴向量结构体
 ******************************************************************************/
typedef struct {
    float x;
    float y;
    float z;
} Vector3;

/******************************************************************************
 *                            IMU 融合类
 ******************************************************************************/
class IMU_Fusion {
private:
    // 四元数状态 (姿态)
    Quaternion q;

    // 滤波器参数
    float beta;           // Madgwick 算法增益 (推荐: 0.033 ~ 0.1)
    float sampleFreq;     // 采样频率 Hz
    float invSampleFreq;  // 采样周期 s

    // 速度和位置状态
    Vector3 velocity;     // 线速度 m/s
    Vector3 position;     // 位置 m

    // 加速度校准偏置
    Vector3 accelBias;
    bool isCalibrated;

    // 低通滤波器状态
    Vector3 lpfAccel;
    float lpfAlpha;       // 低通滤波系数 (0.0 ~ 1.0)

    // 速度衰减参数
    float decayIdle;      // 怠速衰减系数
    float decayMoving;    // 行驶衰减系数

    // 零速检测
    uint32_t stationaryCount;
    uint32_t movingCount;
    float stationaryThresholdAcc;  // 静止判定加速度阈值
    float stationaryThresholdVel;  // 静止判定速度阈值

    // 辅助函数
    void quaternionNormalize();
    void madgwickUpdateIMU(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float dt);
    Vector3 quaternionRotateVector(const Vector3& v);
    Vector3 quaternionRotateVectorInverse(const Vector3& v);
    float invSqrt(float x);

public:
    // 构造函数
    IMU_Fusion(float sampleFrequency = 50.0f, float beta = 0.05f);

    // 初始化
    void begin();
    void reset();

    // 设置参数
    void setBeta(float beta);
    void setSampleFrequency(float freq);
    void setLowPassAlpha(float alpha);
    void setDecayFactors(float idle, float moving);
    void setStationaryThresholds(float accThreshold, float velThreshold);

    // 校准
    void startCalibration();
    void addCalibrationSample(float ax, float ay, float az);
    bool finishCalibration(uint16_t numSamples = 100);
    bool getCalibrationStatus() { return isCalibrated; }
    Vector3 getAccelBias() { return accelBias; }

    // 更新姿态 (核心算法)
    void update(float gx_dps, float gy_dps, float gz_dps,
                float ax_mss, float ay_mss, float az_mss,
                float dt);

    // 速度解算
    void updateVelocity(float ax_mss, float ay_mss, float az_mss, float dt);
    void applyZeroVelocityUpdate();  // ZUPT 零速修正

    // 获取姿态
    Quaternion getQuaternion() { return q; }
    EulerAngles getEulerAngles();
    void getRotationMatrix(float R[3][3]);

    // 从欧拉角设置姿态（用于直接使用 JY901 输出的角度）
    void setEulerAngles(float roll_deg, float pitch_deg, float yaw_deg);
    static Quaternion eulerToQuaternion(float roll_deg, float pitch_deg, float yaw_deg);

    // 获取运动状态
    Vector3 getVelocity() { return velocity; }
    Vector3 getPosition() { return position; }
    float getSpeed();  // 速度模长 m/s
    float getSpeedKmh();  // 速度 km/h

    // 重力补偿 (使用四元数旋转)
    Vector3 removeGravity(float ax_mss, float ay_mss, float az_mss);

    // 重置速度和位置
    void resetVelocity();
    void resetPosition();
};

/******************************************************************************
 *                            全局实例 (可选)
 ******************************************************************************/
extern IMU_Fusion imuFusion;

#endif // IMU_FUSION_H
