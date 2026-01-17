#include "DataProcessor.h"
#include "BLE.h"                  // 用于通过 BLE 输出调试信息
#include "SD_card.h"              // 用于 SD 卡操作
#include <SD_MMC.h>                // SD/MMC 文件系统库（非 SPI）

extern char name[50];
// 简单数组缓冲：固定 10 个样本
static const size_t BATCH_N = 10;
static const size_t BATCH_X = 100;
static float wind[BATCH_N];//风速
static unsigned int degree[BATCH_N];//风向
static float hum[BATCH_N];//湿度
static float temp[BATCH_N];//温度
static float pressure[BATCH_N];//气压
static float Lux[BATCH_N];//光照
static float rain[BATCH_N];//雨量
static unsigned int AS_Lux[BATCH_N]; //AS模块光照
static unsigned int AS_Red[BATCH_N]; //AS模块红外
static size_t g_count = 0; // 已填充样本数量（按索引推进）
//SD卡写入量
// 第2部分：全局变量（添加到DataProcessor.cpp顶部）
// ============================================================================

float jy901_vx = 0.0f; // x方向线速度
float jy901_vy = 0.0f; // y方向线速度
float jy901_vz = 0.0f; // z方向线速度

// 校准相关
static float acc_bias_x = 0.0f;
static float acc_bias_y = 0.0f;
static float acc_bias_z = 0.0f;
static bool is_calibrated = false;
static uint32_t calib_count = 0;
static bool calib_requested = false;  // 手动触发校准标志

// 低通滤波器状态
static float lpf_ax = 0.0f;
static float lpf_ay = 0.0f;
static float lpf_az = 0.0f;

// 零速检测
static uint32_t stationary_count = 0;
static uint32_t moving_count = 0;

// 振动检测
static float vibration_history[10] = {0};  // 最近10次加速度模长
static uint8_t vibration_idx = 0;

// 速度平滑
static float speed_history[5] = {0};  // 最近5次速度
static uint8_t speed_idx = 0;

float SD_windSpeed;
unsigned int SD_winddeg;
float SD_hum;
float SD_temp;
float SD_kPa;
float SD_hundredLux;
float SD_rain;
//手套接收
const size_t GLOVE_LEN = 95;
uint8_t Glove_buf[GLOVE_LEN];
uint32_t lastReceiveTime = 0;
uint8_t bufferIndex = 0;
static bool search_flag = true;
//串口屏接收
const size_t CKP_LEN = 50;//R1:123,456/RHKJ
uint8_t CKP_buf[CKP_LEN];
uint32_t lastReceiveTime_CKP = 0;
uint8_t bufferIndex_CKP = 0;
static bool search_CKP_flag = true;
//手套数据储存
GLOVE_DATA Glove_data;
GLOVE_DATA CKP_data;


// ============================================================================
// 第3部分：辅助函数
// ============================================================================

/**
 * @brief 计算数组平均值
 */
static float array_mean(float *arr, uint8_t len) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < len; i++) {
        sum += arr[i];
    }
    return sum / len;
}

/**
 * @brief 计算数组标准差（检测振动）
 */
static float array_std(float *arr, uint8_t len) {
    float mean = array_mean(arr, len);
    float sum_sq = 0.0f;
    for (uint8_t i = 0; i < len; i++) {
        float diff = arr[i] - mean;
        sum_sq += diff * diff;
    }
    return sqrtf(sum_sq / len);
}

/**
 * @brief 检测是否在振动（发动机运转）
 * @return true=在振动（怠速/行驶），false=真正静止（熄火）
 */
static bool is_vibrating() {
    float std_dev = array_std(vibration_history, 10);
    
    // 标准差大于0.2说明在振动（发动机运转或行驶）
    return (std_dev > 0.2f);
}

/**
 * @brief 获取平滑后的速度（推荐用于显示）
 */
float Motorcycle_GetSmoothSpeed() {
    return array_mean(speed_history, 5);
}

// ============================================================================
// 第4部分：摩托车专用速度解算函数
// ============================================================================

/**
 * @brief 摩托车专用IMU速度解算
 * 
 * @param ax_mss X轴加速度（已去除重力），单位 m/s²
 * @param ay_mss Y轴加速度（已去除重力），单位 m/s²
 * @param az_mss Z轴加速度（已去除重力），单位 m/s²
 * @param pitch 俯仰角，单位度
 * @param roll 横滚角，单位度
 * 
 * 特点：
 * - 大死区（0.5 m/s²）应对振动
 * - 低通滤波平滑数据
 * - 智能零速检测（区分停车和怠速）
 * - 弱速度衰减（不影响正常行驶）
 */
void DP_jy901_Motorcycle(float ax_mss, float ay_mss, float az_mss, float pitch, float roll)
{   
    static uint32_t last_time_us = 0;
    uint32_t current_time_us = micros();
    
    // ========== 时间差计算 ==========
    if (last_time_us == 0) {
        last_time_us = current_time_us;
        return;
    }
    
    float dt;
    if (current_time_us >= last_time_us) {
        dt = (current_time_us - last_time_us) / 1000000.0f;
    } else {
        dt = ((0xFFFFFFFF - last_time_us) + current_time_us + 1) / 1000000.0f;
    }
    last_time_us = current_time_us;
    
    // 跳过异常时间间隔
    if (dt < 0.005f || dt > 0.1f) {
        return;
    }

    // ========== 振动历史记录 ==========
    float acc_magnitude = sqrtf(ax_mss * ax_mss + ay_mss * ay_mss + az_mss * az_mss);
    vibration_history[vibration_idx] = acc_magnitude;
    vibration_idx = (vibration_idx + 1) % 10;

    // ========== 智能校准（只在熄火静止时校准）==========
    if (calib_requested && !is_calibrated) {
        // 检查是否真的静止（无振动）
        if (!is_vibrating() && acc_magnitude < 0.3f) {
            if (calib_count < 100) {  // 100次 * 20ms = 2秒
                acc_bias_x += ax_mss;
                acc_bias_y += ay_mss;
                acc_bias_z += az_mss;
                calib_count++;
                
                if (calib_count == 100) {
                    acc_bias_x /= 100.0f;
                    acc_bias_y /= 100.0f;
                    acc_bias_z /= 100.0f;
                    is_calibrated = true;
                    calib_requested = false;
                    
                    Serial.printf("[摩托车IMU] 校准完成: 偏置=(%.4f, %.4f, %.4f)\n", 
                                 acc_bias_x, acc_bias_y, acc_bias_z);
                }
                return;
            }
        } else {
            // 如果检测到振动，重置校准
            if (calib_count > 0) {
                Serial.println("[摩托车IMU] 校准中断（检测到振动），请在熄火静止时校准");
                calib_count = 0;
                acc_bias_x = 0.0f;
                acc_bias_y = 0.0f;
                acc_bias_z = 0.0f;
            }
        }
    }
    
    // 减去偏置（如果已校准）
    if (is_calibrated) {
        ax_mss -= acc_bias_x;
        ay_mss -= acc_bias_y;
        az_mss -= acc_bias_z;
    }

    // ========== 低通滤波（平滑振动）==========
    // 截止频率约10Hz，可以过滤发动机振动（通常30-50Hz）
    const float LPF_ALPHA = 0.3f;  // alpha越小，滤波越强
    
    lpf_ax = LPF_ALPHA * ax_mss + (1.0f - LPF_ALPHA) * lpf_ax;
    lpf_ay = LPF_ALPHA * ay_mss + (1.0f - LPF_ALPHA) * lpf_ay;
    lpf_az = LPF_ALPHA * az_mss + (1.0f - LPF_ALPHA) * lpf_az;
    
    // 使用滤波后的加速度
    ax_mss = lpf_ax;
    ay_mss = lpf_ay;
    az_mss = lpf_az;

    // ========== 大死区（应对振动噪声）==========
    const float DEADZONE = 0.5f;  // 0.5 m/s²，应对发动机振动
    
    if (fabsf(ax_mss) < DEADZONE) ax_mss = 0.0f;
    if (fabsf(ay_mss) < DEADZONE) ay_mss = 0.0f;
    if (fabsf(az_mss) < DEADZONE) az_mss = 0.0f;

    // ========== 智能零速检测（区分停车和怠速）==========
    float vel_magnitude = sqrtf(jy901_vx*jy901_vx + jy901_vy*jy901_vy + jy901_vz*jy901_vz);
    
    // 方法：同时检测加速度小、速度小、无振动
    const float STATIONARY_ACC_THRESHOLD = 0.3f;
    const float STATIONARY_VEL_THRESHOLD = 0.1f;   // 0.36 km/h
    const uint32_t STATIONARY_COUNT_LIMIT = 100;   // 100次 * 20ms = 2秒
    
    // 真正静止的条件：加速度小 + 速度小 + 无振动
    bool truly_stationary = (acc_magnitude < STATIONARY_ACC_THRESHOLD) && 
                           (vel_magnitude < STATIONARY_VEL_THRESHOLD) &&
                           (!is_vibrating());
    
    if (truly_stationary) {
        stationary_count++;
        moving_count = 0;
        
        if (stationary_count >= STATIONARY_COUNT_LIMIT) {
            // 真正静止（熄火停车），重置速度
            jy901_vx = 0.0f;
            jy901_vy = 0.0f;
            jy901_vz = 0.0f;
            stationary_count = STATIONARY_COUNT_LIMIT;
            return;
        }
    } else {
        // 在移动或怠速
        stationary_count = 0;
        moving_count++;
    }

    // ========== 速度积分 ==========
    jy901_vx += ax_mss * dt;
    jy901_vy += ay_mss * dt;
    jy901_vz += az_mss * dt;

    // ========== 弱速度衰减（不影响正常行驶）==========
    // 只在怠速状态（有振动但速度小）时才衰减
    bool is_idling = is_vibrating() && (vel_magnitude < 2.0f);  // 速度<7.2km/h且振动
    
    if (is_idling) {
        // 怠速时强衰减（对抗累积漂移）
        const float DECAY_IDLE = 0.990f;  // 每次衰减1%
        jy901_vx *= DECAY_IDLE;
        jy901_vy *= DECAY_IDLE;
        jy901_vz *= DECAY_IDLE;
    } else if (moving_count > 50) {  // 确认在移动（1秒以上）
        // 行驶时极弱衰减（几乎不衰减）
        const float DECAY_MOVING = 0.9995f;  // 每次衰减0.05%
        jy901_vx *= DECAY_MOVING;
        jy901_vy *= DECAY_MOVING;
        jy901_vz *= DECAY_MOVING;
    }

    // ========== 速度平滑（移动平均）==========
    speed_history[speed_idx] = vel_magnitude;
    speed_idx = (speed_idx + 1) % 5;

    // ========== 速度限幅（摩托车最高速）==========
    const float MAX_SPEED = 60.0f;  // 60 m/s = 216 km/h（留余量）
    
    if (fabsf(jy901_vx) > MAX_SPEED) jy901_vx = (jy901_vx > 0) ? MAX_SPEED : -MAX_SPEED;
    if (fabsf(jy901_vy) > MAX_SPEED) jy901_vy = (jy901_vy > 0) ? MAX_SPEED : -MAX_SPEED;
    if (fabsf(jy901_vz) > MAX_SPEED) jy901_vz = (jy901_vz > 0) ? MAX_SPEED : -MAX_SPEED;
}

void DP_AS7341(unsigned int AS_Lux_ch,unsigned int AS_Red_ch)
{
    AS_Lux[g_count] = AS_Lux_ch;
    AS_Red[g_count] = AS_Red_ch;
    g_count++;
    if (g_count >= BATCH_N) g_count = 0;
}

void receiveAsString() {
    while(Serial1.available()>0)
    {
      bool dataReceived = false;
      bool parsingError = false;
      uint8_t byte = Serial1.read();
      lastReceiveTime = millis();
      // 如果正在搜索帧头
      if (search_flag) {
          // 检查是否是帧头 'R'
          if (byte == 'R'||byte == 'L') {
              Glove_buf[0] = byte;
              bufferIndex = 1;
              search_flag = false;
          }
      } 
      //如果已经开始接收数据
      else {
        // 将字节存入缓冲区
        if (bufferIndex < GLOVE_LEN) {
            Glove_buf[bufferIndex++] = byte;
        } else {
            // 缓冲区溢出
            Serial.println("MAX ->0");
            bufferIndex = 0;
            search_flag = true;
            parsingError = true;
            dataReceived = true;
            return;
        }
      if (bufferIndex == 95) {
          // 完整数据包已接收
          if (Analyze()) {
              dataReceived = true;
          } else {
              parsingError = true;
              dataReceived = true;
          }
          
          // 重置接收状态
          bufferIndex = 0;
          search_flag = true;
          break;  // 退出循环，处理当前数据
        }
      }
    }
}

bool Analyze() {//数据解析
    if(Glove_buf[0]!='R' && Glove_buf[0]!='L')
    {
        BLE_sendf("错误:帧头不是R");
        return false;
    }
    Glove_data.RL = Glove_buf[0];

    if(Glove_data.RL == 'R')
    {
        
        Glove_data.mode_R = Glove_buf[1]-'0';
        Glove_data.temp_R = 10*(Glove_buf[2]-'0') + (Glove_buf[3]-'0');
        Glove_data.temp_now_R[0] = 1000*(Glove_buf[7]-'0')+100*(Glove_buf[8]-'0')+10*(Glove_buf[9]-'0')+(Glove_buf[10]-'0');
        Glove_data.P_R[0] = 100*(Glove_buf[11+13*0]-'0') + 10*(Glove_buf[12+13*0]-'0') + (Glove_buf[13+13*0]-'0');
        Glove_data.MAX_R[0] = 100*(Glove_buf[14+13*0]-'0') + 10*(Glove_buf[15+13*0]-'0') + (Glove_buf[16+13*0]-'0');

        Glove_data.temp_now_R[1] = 1000*(Glove_buf[7+13*1]-'0')+100*(Glove_buf[8+13*1]-'0')+10*(Glove_buf[9+13*1]-'0')+(Glove_buf[10+13*1]-'0');
        Glove_data.P_R[1] = 100*(Glove_buf[11+13*1]-'0') + 10*(Glove_buf[12+13*1]-'0') + (Glove_buf[13+13*1]-'0');
        Glove_data.MAX_R[1] = 100*(Glove_buf[14+13*1]-'0') + 10*(Glove_buf[15+13*1]-'0') + (Glove_buf[16+13*1]-'0');
       
        Glove_data.temp_now_R[2] = 1000*(Glove_buf[7+13*2]-'0')+100*(Glove_buf[8+13*2]-'0')+10*(Glove_buf[9+13*2]-'0')+(Glove_buf[10+13*2]-'0');
        Glove_data.P_R[2] = 100*(Glove_buf[11+13*2]-'0') + 10*(Glove_buf[12+13*2]-'0') + (Glove_buf[13+13*2]-'0');
        Glove_data.MAX_R[2] = 100*(Glove_buf[14+13*2]-'0') + 10*(Glove_buf[15+13*2]-'0') + (Glove_buf[16+13*2]-'0');

        Glove_data.temp_now_R[3] = 1000*(Glove_buf[7+13*3]-'0')+100*(Glove_buf[8+13*3]-'0')+10*(Glove_buf[9+13*3]-'0')+(Glove_buf[10+13*3]-'0');
        Glove_data.P_R[3] = 100*(Glove_buf[11+13*3]-'0') + 10*(Glove_buf[12+13*3]-'0') + (Glove_buf[13+13*3]-'0');
        Glove_data.MAX_R[3] = 100*(Glove_buf[14+13*3]-'0') + 10*(Glove_buf[15+13*3]-'0') + (Glove_buf[16+13*3]-'0');

        Glove_data.temp_now_R[4] = 1000*(Glove_buf[7+13*4]-'0')+100*(Glove_buf[8+13*4]-'0')+10*(Glove_buf[9+13*4]-'0')+(Glove_buf[10+13*4]-'0');
        Glove_data.P_R[4] = 100*(Glove_buf[11+13*4]-'0') + 10*(Glove_buf[12+13*4]-'0') + (Glove_buf[13+13*4]-'0');
        Glove_data.MAX_R[4] = 100*(Glove_buf[14+13*4]-'0') + 10*(Glove_buf[15+13*4]-'0') + (Glove_buf[16+13*4]-'0');

        Glove_data.temp_now_R[5] = 1000*(Glove_buf[7+13*5]-'0')+100*(Glove_buf[8+13*5]-'0')+10*(Glove_buf[9+13*5]-'0')+(Glove_buf[10+13*5]-'0');
        Glove_data.P_R[5] = 100*(Glove_buf[11+13*5]-'0') + 10*(Glove_buf[12+13*5]-'0') + (Glove_buf[13+13*51]-'0');
        Glove_data.MAX_R[5] = 100*(Glove_buf[14+13*5]-'0') + 10*(Glove_buf[15+13*5]-'0') + (Glove_buf[16+13*5]-'0');

        Glove_data.temp_now_R[6] = 1000*(Glove_buf[7+13*6]-'0')+100*(Glove_buf[8+13*6]-'0')+10*(Glove_buf[9+13*6]-'0')+(Glove_buf[10+13*6]-'0');
        Glove_data.P_R[6] = 100*(Glove_buf[11+13*6]-'0') + 10*(Glove_buf[12+13*6]-'0') + (Glove_buf[13+13*6]-'0');
        Glove_data.MAX_R[6] = 100*(Glove_buf[14+13*6]-'0') + 10*(Glove_buf[15+13*6]-'0') + (Glove_buf[16+13*6]-'0');
    }
    else {
        
        Glove_data.mode_L = Glove_buf[1]-'0';
        Glove_data.temp_L = 10*(Glove_buf[2]-'0') + (Glove_buf[3]-'0');
        Glove_data.temp_now_L[0] = 1000*(Glove_buf[7]-'0')+100*(Glove_buf[8]-'0')+10*(Glove_buf[9]-'0')+(Glove_buf[10]-'0');
        Glove_data.P_L[0] = 100*(Glove_buf[11+13*0]-'0') + 10*(Glove_buf[12+13*0]-'0') + (Glove_buf[13+13*0]-'0');
        Glove_data.MAX_L[0] = 100*(Glove_buf[14+13*0]-'0') + 10*(Glove_buf[15+13*0]-'0') + (Glove_buf[16+13*0]-'0');

        Glove_data.temp_now_L[1] = 1000*(Glove_buf[7+13*1]-'0')+100*(Glove_buf[8+13*1]-'0')+10*(Glove_buf[9+13*1]-'0')+(Glove_buf[10+13*1]-'0');
        Glove_data.P_L[1] = 100*(Glove_buf[11+13*1]-'0') + 10*(Glove_buf[12+13*1]-'0') + (Glove_buf[13+13*1]-'0');
        Glove_data.MAX_L[1] = 100*(Glove_buf[14+13*1]-'0') + 10*(Glove_buf[15+13*1]-'0') + (Glove_buf[16+13*1]-'0');
       
        Glove_data.temp_now_L[2] = 1000*(Glove_buf[7+13*2]-'0')+100*(Glove_buf[8+13*2]-'0')+10*(Glove_buf[9+13*2]-'0')+(Glove_buf[10+13*2]-'0');
        Glove_data.P_L[2] = 100*(Glove_buf[11+13*2]-'0') + 10*(Glove_buf[12+13*2]-'0') + (Glove_buf[13+13*2]-'0');
        Glove_data.MAX_L[2] = 100*(Glove_buf[14+13*2]-'0') + 10*(Glove_buf[15+13*2]-'0') + (Glove_buf[16+13*2]-'0');

        Glove_data.temp_now_L[3] = 1000*(Glove_buf[7+13*3]-'0')+100*(Glove_buf[8+13*3]-'0')+10*(Glove_buf[9+13*3]-'0')+(Glove_buf[10+13*3]-'0');
        Glove_data.P_L[3] = 100*(Glove_buf[11+13*3]-'0') + 10*(Glove_buf[12+13*3]-'0') + (Glove_buf[13+13*3]-'0');
        Glove_data.MAX_L[3] = 100*(Glove_buf[14+13*3]-'0') + 10*(Glove_buf[15+13*3]-'0') + (Glove_buf[16+13*3]-'0');

        Glove_data.temp_now_L[4] = 1000*(Glove_buf[7+13*4]-'0')+100*(Glove_buf[8+13*4]-'0')+10*(Glove_buf[9+13*4]-'0')+(Glove_buf[10+13*4]-'0');
        Glove_data.P_L[4] = 100*(Glove_buf[11+13*4]-'0') + 10*(Glove_buf[12+13*4]-'0') + (Glove_buf[13+13*4]-'0');
        Glove_data.MAX_L[4] = 100*(Glove_buf[14+13*4]-'0') + 10*(Glove_buf[15+13*4]-'0') + (Glove_buf[16+13*4]-'0');

        Glove_data.temp_now_L[5] = 1000*(Glove_buf[7+13*5]-'0')+100*(Glove_buf[8+13*5]-'0')+10*(Glove_buf[9+13*5]-'0')+(Glove_buf[10+13*5]-'0');
        Glove_data.P_L[5] = 100*(Glove_buf[11+13*5]-'0') + 10*(Glove_buf[12+13*5]-'0') + (Glove_buf[13+13*51]-'0');
        Glove_data.MAX_L[5] = 100*(Glove_buf[14+13*5]-'0') + 10*(Glove_buf[15+13*5]-'0') + (Glove_buf[16+13*5]-'0');

        Glove_data.temp_now_L[6] = 1000*(Glove_buf[7+13*6]-'0')+100*(Glove_buf[8+13*6]-'0')+10*(Glove_buf[9+13*6]-'0')+(Glove_buf[10+13*6]-'0');
        Glove_data.P_L[6] = 100*(Glove_buf[11+13*6]-'0') + 10*(Glove_buf[12+13*6]-'0') + (Glove_buf[13+13*6]-'0');
        Glove_data.MAX_L[6] = 100*(Glove_buf[14+13*6]-'0') + 10*(Glove_buf[15+13*6]-'0') + (Glove_buf[16+13*6]-'0');
    }

    return true;
}

void CKP_receive() {
    while(Serial2.available()>0)
    {
      bool dataReceived = false;
      bool parsingError = false;
      uint8_t byte = Serial2.read();
      lastReceiveTime_CKP = millis();
      // 如果正在搜索帧头
      if (search_CKP_flag) {
          // 检查是否是帧头 'R'
          if (byte == 'R'||byte == 'L') {
              CKP_buf[0] = byte;
              bufferIndex_CKP = 1;
              search_CKP_flag = false;
          }
      } 
      //如果已经开始接收数据
      else {
        // 将字节存入缓冲区
        if (byte != '|') {
            CKP_buf[bufferIndex_CKP++] = byte;
        } else if (byte == '|') {
          // 完整数据包已接收
          if (CKP_Analyze()) {
            bufferIndex_CKP = 0;
              dataReceived = true;
          } else {
              parsingError = true;
              dataReceived = true;
          }
          
          // 重置接收状态
          bufferIndex = 0;
          search_CKP_flag = true;
          break;  // 退出循环，处理当前数据
        }
      }
    }
}

bool CKP_Analyze()
{
    if(CKP_buf[1]== 'H') {//标记
        String p(name);
        File f = SD_MMC.open(p.c_str(), FILE_APPEND);   // 以追加方式打开
        f.printf("\r\n-------------------------------------------------------\r\n");//SD内标记方式
        f.close();
    }
    if(CKP_buf[2]== ':') {//手套数据解析
        CKP_data.RL = CKP_buf[0];
        if(CKP_data.RL == 'R')
        {
            digitalWrite(16, HIGH);
            CKP_data.Channel_R = CKP_buf[1]-'0';
            CKP_data.P_R[CKP_data.Channel_R] = 100*(CKP_buf[3]-'0') + 10*(CKP_buf[4]-'0') + 1*(CKP_buf[5]-'0');
            CKP_data.MAX_R[CKP_data.Channel_R] = 100*(CKP_buf[7]-'0') + 10*(CKP_buf[8]-'0') + 1*(CKP_buf[9]-'0');
            Serial1.printf("%01c%01d:%03d,%03d",CKP_data.RL,CKP_data.Channel_R,CKP_data.P_R[CKP_data.Channel_R],CKP_data.MAX_R[CKP_data.Channel_R]);
        }else if(CKP_data.RL == 'L') {
            digitalWrite(16, LOW);
            CKP_data.Channel_L = CKP_buf[1]-'0';
            CKP_data.P_L[CKP_data.Channel_L] = 100*(CKP_buf[3]-'0') + 10*(CKP_buf[4]-'0') + 1*(CKP_buf[5]-'0');
            CKP_data.MAX_L[CKP_data.Channel_L] = 100*(CKP_buf[7]-'0') + 10*(CKP_buf[8]-'0') + 1*(CKP_buf[9]-'0');
            Serial1.printf("%01c%01d:%03d,%03d",CKP_data.RL,CKP_data.Channel_L,CKP_data.P_L[CKP_data.Channel_L],CKP_data.MAX_L[CKP_data.Channel_L]);
        }  
    }
    // else {//错误
        
    // }
    return true;
}