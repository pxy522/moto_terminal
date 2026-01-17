#ifndef DATA_PROCESSOR_H_
#define DATA_PROCESSOR_H_

#include <Arduino.h>

typedef struct  
{
    uint8_t RL;

    unsigned int mode_R;
    unsigned int temp_R;
    unsigned int Channel_R;
    int temp_now_R[7];
    unsigned int P_R[7];
    unsigned int MAX_R[7];

    unsigned int mode_L;
    unsigned int temp_L;
    unsigned int Channel_L;
    int temp_now_L[7];
    unsigned int P_L[7];
    unsigned int MAX_L[7];
}GLOVE_DATA;
extern GLOVE_DATA Glove_data;

void removeGravity(float pitch, float roll);
void DP_jy901(float ax_g,float ay_g,float az_g, float pitch, float roll);// 处理 JY901 加速度数据
void DP_jy901_Motorcycle(float ax_mss, float ay_mss, float az_mss, float pitch, float roll); // 摩托车专用IMU处理
float Motorcycle_GetSmoothSpeed();//线速度数据平滑显示

void DP_weather(float windSpeed_m_s,unsigned int windDir_deg,float humidity_pct,
                 float temperature_C,float pressure_kPa,float lux20_hundredLux,
                 float rain_mm);// 处理气象数据
void DP_AS7341(unsigned int AS_Lux_ch,unsigned int AS_Red_ch);// 处理 AS7341 光谱数据
void receiveAsString();//手套长格式数据接收
bool Analyze();
void CKP_receive();
bool CKP_Analyze();

#endif // DATA_PROCESSOR_H_
