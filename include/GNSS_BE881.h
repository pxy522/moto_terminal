/******************************************************************************
 *                   BE-881 GNSS 模块驱动 - 头文件
 *
 *  功能:
 *  - 软串口接收 NMEA 0183 数据
 *  - 解析 GPS/北斗/GLONASS/Galileo 定位数据
 *  - 提供速度、位置、时间信息
 *  - 与 IMU 融合提供高精度定位
 ******************************************************************************/

#ifndef GNSS_BE881_H
#define GNSS_BE881_H

#include <Arduino.h>

/******************************************************************************
 *                            配置参数
 ******************************************************************************/
#define GNSS_RX_PIN         10      // GPIO10 接收 GNSS TX (ESP32的RX)
#define GNSS_TX_PIN         12      // GPIO12 发送到 GNSS RX (ESP32的TX)
#define GNSS_BAUD_RATE      115200  // 默认波特率
#define GNSS_BUFFER_SIZE    256     // NMEA 句子缓冲区

/******************************************************************************
 *                            数据结构
 ******************************************************************************/

/**
 * @brief GNSS 定位数据结构
 */
struct GNSS_Data {
    // 时间信息
    uint8_t  hour;          // UTC 小时 (0-23)
    uint8_t  minute;        // UTC 分钟 (0-59)
    uint8_t  second;        // UTC 秒 (0-59)
    uint16_t millisecond;   // 毫秒 (0-999)

    uint8_t  year;          // 年份 (2000-2099)
    uint8_t  month;         // 月份 (1-12)
    uint8_t  day;           // 日期 (1-31)

    // 位置信息
    double   latitude;      // 纬度 (度, -90 to +90)
    double   longitude;     // 经度 (度, -180 to +180)
    float    altitude;      // 海拔高度 (米)

    // 速度信息
    float    speed_kmh;     // 地速 (km/h)
    float    speed_mps;     // 地速 (m/s)
    float    speed_knots;   // 地速 (节)

    // 方向信息
    float    course;        // 航向角 (度, 0-360)

    // 定位质量
    uint8_t  fix_quality;   // 0=未定位, 1=GPS, 2=DGPS, 4=RTK固定, 5=RTK浮动
    uint8_t  satellites;    // 使用的卫星数量
    float    hdop;          // 水平精度因子
    float    vdop;          // 垂直精度因子
    float    pdop;          // 位置精度因子

    // 状态标志
    bool     is_valid;      // 定位数据是否有效
    uint32_t last_update;   // 最后更新时间戳 (毫秒)
};

/******************************************************************************
 *                            类定义
 ******************************************************************************/

class GNSS_BE881 {
public:
    GNSS_BE881();

    // 初始化
    void begin(int rx_pin = GNSS_RX_PIN, int tx_pin = GNSS_TX_PIN,
               uint32_t baud = GNSS_BAUD_RATE);

    // 数据处理
    void update();                    // 更新 GNSS 数据 (在主循环或任务中调用)
    bool available();                 // 是否有新数据

    // 数据访问
    GNSS_Data getData();              // 获取完整数据结构
    bool isValid();                   // 定位是否有效
    uint8_t getFixQuality();          // 获取定位质量
    uint8_t getSatellites();          // 获取卫星数量

    // 位置信息
    double getLatitude();             // 纬度 (度)
    double getLongitude();            // 经度 (度)
    float getAltitude();              // 海拔 (米)

    // 速度信息
    float getSpeedKmh();              // 速度 (km/h)
    float getSpeedMps();              // 速度 (m/s)
    float getCourse();                // 航向 (度)

    // 时间信息
    void getTime(uint8_t &h, uint8_t &m, uint8_t &s);
    void getDate(uint8_t &y, uint8_t &m, uint8_t &d);

    // 精度信息
    float getHDOP();                  // 水平精度因子

    // 诊断
    void printDiagnostics();          // 打印诊断信息
    uint32_t getMessageCount();       // 获取接收的消息数量

private:
    GNSS_Data data;

    char nmea_buffer[GNSS_BUFFER_SIZE];
    uint16_t buffer_index;

    bool new_data;
    uint32_t message_count;
    uint32_t parse_error_count;

    // NMEA 解析
    void processNMEA(const char* sentence);
    bool parseGGA(const char* sentence);  // 位置、精度、卫星数
    bool parseRMC(const char* sentence);  // 位置、速度、时间、日期
    bool parseVTG(const char* sentence);  // 速度、航向
    bool parseGSA(const char* sentence);  // 精度因子 (PDOP, HDOP, VDOP)

    // 辅助函数
    bool validateChecksum(const char* sentence);
    double parseLatitude(const char* lat, const char* ns);
    double parseLongitude(const char* lon, const char* ew);
    float parseFloat(const char* str);
    int parseInt(const char* str);
    void extractField(const char* sentence, int field_num, char* buffer, int buf_size);
    int countFields(const char* sentence);
};

// 全局 GNSS 对象
extern GNSS_BE881 gnss;

#endif // GNSS_BE881_H
