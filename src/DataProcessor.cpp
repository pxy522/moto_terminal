#include "DataProcessor.h"
#include "BLE.h"                  // 用于通过 BLE 输出调试信息
#include "SD_card.h"              // 用于 SD 卡操作
#include <SD_MMC.h>                // SD/MMC 文件系统库（非 SPI）
#include <math.h>

extern char name[50];
// 简单数组缓冲：固定 10 个样本
static const size_t BATCH_N = 10;
static const size_t BATCH_X = 100;
static float jy901_x[BATCH_X];//加速度
static float jy901_y[BATCH_X];
static float jy901_z[BATCH_X];
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

float jy901_vx = 0.0f; float jy901_vy = 0.0f; float jy901_vz = 0.0f; // xyz方向线速度
// 摩托车专用速度计算状态变量
static float prev_forward_accel = 0.0f;           // 上次的前向加速度
static float integrated_velocity = 0.0f;          // 积分得到的速度 (m/s)
static float filtered_velocity = 0.0f;            // 滤波后的速度 (m/s)
static float lowpass_acc[3] = {0.0f, 0.0f, 0.0f}; // 低通滤波加速度
static unsigned long prev_time_ms = 0;            // 上次计算时间
static bool first_sample = true;                  // 是否是第一个样本
static bool have_prev_acc = false;                // 是否有前一个加速度数据

// 摩托车参数（可根据实际情况调整）
#define MAX_ACCEL 2.0f        // 最大合理加速度 (g) - 防止异常值
#define VELOCITY_DAMPING 0.995f // 速度阻尼系数，防止积分漂移
#define MAX_SPEED 45.0f       // 最大合理速度 (m/s) ~180 km/h

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

void DP_jy901(float ax_g, float ay_g, float az_g)
{   
    // 1. 计算实际时间间隔（使用实际时间，更精确）
    unsigned long current_time_ms = millis();
    float dt;
    
    if (first_sample) {
        dt = 0.02f;  // 默认采样周期
        first_sample = false;
    } else if (prev_time_ms > 0) {
        dt = (current_time_ms - prev_time_ms) / 1000.0f; // 转换为秒
        // 限制dt在合理范围内
        if (dt > 0.1f) dt = 0.02f;    // 如果间隔太大，使用默认值
        if (dt < 0.001f) dt = 0.001f; // 防止过小
    } else {
        dt = 0.02f;
    }
    
    // 2. 摩托车场景优化：重点关注前向加速度（X轴）
    // 假设传感器安装时X轴指向摩托车前进方向
    float forward_accel = ax_g;
    float lateral_accel = ay_g;
    float vertical_accel = az_g;
    
    // 3. 限制加速度范围（防止异常值）
    if (fabs(forward_accel) > MAX_ACCEL) {
        forward_accel = (forward_accel > 0) ? MAX_ACCEL : -MAX_ACCEL;
    }
    
    // 4. 使用高通滤波分离重力分量和运动加速度
    // 这对于摩托车很重要，因为摩托车会倾斜，重力会分解到各个轴
    const float alpha = 0.05f; // 低通滤波系数，用于估计重力分量
    
    // 更新低通滤波值（估计重力/低频分量）
    lowpass_acc[0] = alpha * forward_accel + (1.0f - alpha) * lowpass_acc[0];
    lowpass_acc[1] = alpha * lateral_accel + (1.0f - alpha) * lowpass_acc[1];
    lowpass_acc[2] = alpha * vertical_accel + (1.0f - alpha) * lowpass_acc[2];
    
    // 高通滤波：原始值减去低频分量得到运动加速度
    float motion_forward_accel = forward_accel - lowpass_acc[0];
    
    // 5. 积分得到速度（使用梯形法提高精度）
    if (have_prev_acc && dt > 0) {
        // 梯形积分：v += (a_prev + a_current) * 0.5 * dt
        integrated_velocity += (prev_forward_accel + motion_forward_accel) * 0.5f * 9.8f * dt;
        
        // 6. 防止积分漂移：添加速度阻尼
        // 模拟空气阻力，防止速度无限增长
        integrated_velocity *= VELOCITY_DAMPING;
        
        // 7. 限制最大速度（防止误差累积过大）
        if (fabs(integrated_velocity) > MAX_SPEED) {
            integrated_velocity = (integrated_velocity > 0) ? MAX_SPEED : -MAX_SPEED;
        }
        
        // 8. 摩托车专用零速检测
        // 当速度和加速度都很小时，认为摩托车静止
        float speed_magnitude = fabs(integrated_velocity);
        float accel_magnitude = fabs(motion_forward_accel * 9.8f);
        
        if (speed_magnitude < 0.1f && accel_magnitude < 0.1f) {
            // 缓慢归零，避免突变
            integrated_velocity *= 0.8f;
            if (fabs(integrated_velocity) < 0.01f) {
                integrated_velocity = 0.0f;
            }
        }
    }
    
    // 9. 保存当前加速度供下次使用
    prev_forward_accel = motion_forward_accel;
    have_prev_acc = true;
    
    // 10. 更新全局速度变量
    // 对于摩托车，我们主要关心前进方向的速度
    jy901_vx = integrated_velocity;  // 前进方向速度 (m/s)
    jy901_vy = 0.0f;  // 侧向速度通常很小，可忽略或单独处理
    jy901_vz = 0.0f;  // 垂直速度通常很小，可忽略
    
    // 11. 更新时间戳
    prev_time_ms = current_time_ms;
}


void DP_weather(float windSpeed_m_s,unsigned int windDir_deg,float humidity_pct,
                 float temperature_C,float pressure_kPa,float lux20_hundredLux,
                 float rain_mm)
{
    wind[g_count] = windSpeed_m_s;
    degree[g_count] = windDir_deg;
    hum[g_count] = humidity_pct;
    temp[g_count] = temperature_C;
    pressure[g_count] = pressure_kPa;
    Lux[g_count] = lux20_hundredLux;
    rain[g_count] = rain_mm;

    g_count++;
    if (g_count >= BATCH_N) 
    {
        g_count = 0;
    }
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

        Glovvoide_data.temp_now_L[5] = 1000*(Glove_buf[7+13*5]-'0')+100*(Glove_buf[8+13*5]-'0')+10*(Glove_buf[9+13*5]-'0')+(Glove_buf[10+13*5]-'0');
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