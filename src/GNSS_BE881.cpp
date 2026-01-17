/******************************************************************************
 *                   BE-881 GNSS 模块驱动 - 实现
 ******************************************************************************/

#include "GNSS_BE881.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

// 使用 ESP32 硬件串口 (由于软串口在 115200 bps 可能不稳定)
// 我们使用 GPIO RX 映射功能
HardwareSerial GNSSSerial(1); // 使用 Serial1 (UART1)

/******************************************************************************
 *                            构造函数
 ******************************************************************************/

GNSS_BE881::GNSS_BE881() {
    memset(&data, 0, sizeof(GNSS_Data));
    buffer_index = 0;
    new_data = false;
    message_count = 0;
    parse_error_count = 0;
}

/******************************************************************************
 *                            初始化
 ******************************************************************************/

void GNSS_BE881::begin(int rx_pin, int tx_pin, uint32_t baud) {
    // 使用硬件串口 UART1,映射到自定义 GPIO
    // ESP32-S3 支持任意 GPIO 映射
    GNSSSerial.begin(baud, SERIAL_8N1, rx_pin, tx_pin);

    Serial.println("\n========== GNSS BE-881 初始化 ==========");
    Serial.printf("[GNSS] ESP32 配置: RX=GPIO%d, TX=GPIO%d, 波特率=%d\n", rx_pin, tx_pin, baud);
    Serial.println("\n⚠️  请确认物理接线:");
    Serial.printf("  GNSS TX (PIN3) → ESP32 GPIO%d (RX接收)\n", rx_pin);
    Serial.printf("  GNSS RX (PIN4) → ESP32 GPIO%d (TX发送)\n", tx_pin);
    Serial.println("  GNSS VCC (PIN5) → ESP32 5V");
    Serial.println("  GNSS GND (PIN2) → ESP32 GND");
    Serial.println("\n💡 GNSS模块上电后:");
    Serial.println("  - 蓝灯持续闪烁 = TX正在发送数据 (正常)");
    Serial.println("  - 蓝灯闪10秒后停止 = TX线可能接错或悬空");
    Serial.println("  - 红灯每秒闪1次 = 已定位成功 (PPS信号)");
    Serial.println("\n[GNSS] 开始监听串口数据...");
    Serial.println("========================================\n");

    data.is_valid = false;
    data.last_update = 0;
}

/******************************************************************************
 *                            数据更新
 ******************************************************************************/

void GNSS_BE881::update() {
    static uint32_t byte_count = 0;
    static uint32_t last_debug_print = 0;
    static bool debug_complete = false;

    // 只在收到前1000字节时打印调试信息
    if (!debug_complete && millis() - last_debug_print >= 5000) {
        last_debug_print = millis();
        Serial.printf("[GNSS DEBUG] 已接收字节数: %u, 消息数: %u\n", byte_count, message_count);

        if (byte_count > 1000) {
            Serial.println("[GNSS DEBUG] ✅ 串口通信正常,已停止调试输出");
            debug_complete = true;
        }
    }

    while (GNSSSerial.available()) {
        char c = GNSSSerial.read();
        byte_count++;

        // NMEA 句子以 '$' 开始, '\n' 结束
        if (c == '$') {
            buffer_index = 0;
            nmea_buffer[buffer_index++] = c;
        }
        else if (buffer_index > 0) {
            if (c == '\r') {
                // 忽略回车符
                continue;
            }
            else if (c == '\n') {
                // 句子结束
                nmea_buffer[buffer_index] = '\0';
                processNMEA(nmea_buffer);
                buffer_index = 0;
            }
            else if (buffer_index < GNSS_BUFFER_SIZE - 1) {
                nmea_buffer[buffer_index++] = c;
            }
            else {
                // 缓冲区溢出,重置
                buffer_index = 0;
            }
        }
    }
}

bool GNSS_BE881::available() {
    bool ret = new_data;
    new_data = false;
    return ret;
}

/******************************************************************************
 *                            NMEA 解析
 ******************************************************************************/

void GNSS_BE881::processNMEA(const char* sentence) {
    // 验证校验和
    if (!validateChecksum(sentence)) {
        parse_error_count++;
        return;
    }

    message_count++;

    // 识别 NMEA 句子类型 (跳过 Talker ID, 只看消息类型)
    const char* msg_type = sentence + 3;  // 跳过 "$GNRMC" 的 "$GN"

    if (strncmp(msg_type, "GGA", 3) == 0) {
        parseGGA(sentence);
    }
    else if (strncmp(msg_type, "RMC", 3) == 0) {
        parseRMC(sentence);
        new_data = true;  // RMC 包含最全面的信息,作为新数据标志
    }
    else if (strncmp(msg_type, "VTG", 3) == 0) {
        parseVTG(sentence);
    }
    else if (strncmp(msg_type, "GSA", 3) == 0) {
        parseGSA(sentence);
    }
}

/**
 * @brief 解析 GGA - 全球定位系统固定数据
 * $GNGGA,time,lat,NS,lon,EW,quality,numSV,HDOP,alt,altUnit,sep,sepUnit,diffAge,diffStation*cs
 */
bool GNSS_BE881::parseGGA(const char* sentence) {
    char field[32];

    // 字段 1: 时间 (hhmmss.ss)
    extractField(sentence, 1, field, sizeof(field));
    if (strlen(field) >= 6) {
        data.hour = (field[0] - '0') * 10 + (field[1] - '0');
        data.minute = (field[2] - '0') * 10 + (field[3] - '0');
        data.second = (field[4] - '0') * 10 + (field[5] - '0');
        if (strlen(field) > 7) {
            data.millisecond = parseFloat(field + 7) * 1000;
        }
    }

    // 字段 2-3: 纬度
    extractField(sentence, 2, field, sizeof(field));
    char ns[2];
    extractField(sentence, 3, ns, sizeof(ns));
    if (strlen(field) > 0) {
        data.latitude = parseLatitude(field, ns);
    }

    // 字段 4-5: 经度
    extractField(sentence, 4, field, sizeof(field));
    char ew[2];
    extractField(sentence, 5, ew, sizeof(ew));
    if (strlen(field) > 0) {
        data.longitude = parseLongitude(field, ew);
    }

    // 字段 6: 定位质量
    extractField(sentence, 6, field, sizeof(field));
    data.fix_quality = parseInt(field);
    data.is_valid = (data.fix_quality > 0);

    // 字段 7: 卫星数量
    extractField(sentence, 7, field, sizeof(field));
    data.satellites = parseInt(field);

    // 字段 8: HDOP
    extractField(sentence, 8, field, sizeof(field));
    data.hdop = parseFloat(field);

    // 字段 9: 海拔高度
    extractField(sentence, 9, field, sizeof(field));
    data.altitude = parseFloat(field);

    data.last_update = millis();
    return true;
}

/**
 * @brief 解析 RMC - 推荐最小定位信息
 * $GNRMC,time,status,lat,NS,lon,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs
 */
bool GNSS_BE881::parseRMC(const char* sentence) {
    char field[32];

    // 字段 1: 时间
    extractField(sentence, 1, field, sizeof(field));
    if (strlen(field) >= 6) {
        data.hour = (field[0] - '0') * 10 + (field[1] - '0');
        data.minute = (field[2] - '0') * 10 + (field[3] - '0');
        data.second = (field[4] - '0') * 10 + (field[5] - '0');
        if (strlen(field) > 7) {
            data.millisecond = parseFloat(field + 7) * 1000;
        }
    }

    // 字段 2: 状态 (A=有效, V=无效)
    extractField(sentence, 2, field, sizeof(field));
    bool status_valid = (field[0] == 'A');

    // 字段 3-4: 纬度
    extractField(sentence, 3, field, sizeof(field));
    char ns[2];
    extractField(sentence, 4, ns, sizeof(ns));
    if (strlen(field) > 0 && status_valid) {
        data.latitude = parseLatitude(field, ns);
    }

    // 字段 5-6: 经度
    extractField(sentence, 5, field, sizeof(field));
    char ew[2];
    extractField(sentence, 6, ew, sizeof(ew));
    if (strlen(field) > 0 && status_valid) {
        data.longitude = parseLongitude(field, ew);
    }

    // 字段 7: 速度 (节)
    extractField(sentence, 7, field, sizeof(field));
    if (strlen(field) > 0) {
        data.speed_knots = parseFloat(field);
        data.speed_kmh = data.speed_knots * 1.852f;  // 节 -> km/h
        data.speed_mps = data.speed_kmh / 3.6f;      // km/h -> m/s
    }

    // 字段 8: 航向
    extractField(sentence, 8, field, sizeof(field));
    if (strlen(field) > 0) {
        data.course = parseFloat(field);
    }

    // 字段 9: 日期 (ddmmyy)
    extractField(sentence, 9, field, sizeof(field));
    if (strlen(field) >= 6) {
        data.day = (field[0] - '0') * 10 + (field[1] - '0');
        data.month = (field[2] - '0') * 10 + (field[3] - '0');
        data.year = (field[4] - '0') * 10 + (field[5] - '0') + 2000;
    }

    data.is_valid = status_valid;
    data.last_update = millis();
    return true;
}

/**
 * @brief 解析 VTG - 地面速度信息
 * $GNVTG,cogt,cogtUnit,cogm,cogmUnit,sogn,sognUnit,sogk,sogkUnit,posMode*cs
 */
bool GNSS_BE881::parseVTG(const char* sentence) {
    char field[32];

    // 字段 1: 真北航向
    extractField(sentence, 1, field, sizeof(field));
    if (strlen(field) > 0) {
        data.course = parseFloat(field);
    }

    // 字段 5: 速度 (节)
    extractField(sentence, 5, field, sizeof(field));
    if (strlen(field) > 0) {
        data.speed_knots = parseFloat(field);
        data.speed_kmh = data.speed_knots * 1.852f;
        data.speed_mps = data.speed_kmh / 3.6f;
    }

    return true;
}

/**
 * @brief 解析 GSA - GNSS DOP 和有效卫星
 * $GNGSA,opMode,navMode{,svid},PDOP,HDOP,VDOP,systemId*cs
 */
bool GNSS_BE881::parseGSA(const char* sentence) {
    char field[32];

    // 字段 15: PDOP
    extractField(sentence, 15, field, sizeof(field));
    if (strlen(field) > 0) {
        data.pdop = parseFloat(field);
    }

    // 字段 16: HDOP
    extractField(sentence, 16, field, sizeof(field));
    if (strlen(field) > 0) {
        data.hdop = parseFloat(field);
    }

    // 字段 17: VDOP
    extractField(sentence, 17, field, sizeof(field));
    if (strlen(field) > 0) {
        data.vdop = parseFloat(field);
    }

    return true;
}

/******************************************************************************
 *                            辅助函数
 ******************************************************************************/

/**
 * @brief 验证 NMEA 校验和
 */
bool GNSS_BE881::validateChecksum(const char* sentence) {
    if (sentence[0] != '$') return false;

    // 查找 '*'
    const char* star = strchr(sentence, '*');
    if (!star) return false;

    // 计算校验和 (从 '$' 之后到 '*' 之前)
    uint8_t checksum = 0;
    for (const char* p = sentence + 1; p < star; p++) {
        checksum ^= *p;
    }

    // 解析 16 进制校验和
    uint8_t expected = strtoul(star + 1, NULL, 16);

    return (checksum == expected);
}

/**
 * @brief 解析纬度 (ddmm.mmmmm -> 度)
 */
double GNSS_BE881::parseLatitude(const char* lat, const char* ns) {
    if (strlen(lat) < 4) return 0.0;

    // ddmm.mmmmm 格式
    char degrees[3] = {lat[0], lat[1], '\0'};
    double deg = atof(degrees);
    double min = atof(lat + 2);

    double latitude = deg + min / 60.0;

    if (ns[0] == 'S') {
        latitude = -latitude;
    }

    return latitude;
}

/**
 * @brief 解析经度 (dddmm.mmmmm -> 度)
 */
double GNSS_BE881::parseLongitude(const char* lon, const char* ew) {
    if (strlen(lon) < 5) return 0.0;

    // dddmm.mmmmm 格式
    char degrees[4] = {lon[0], lon[1], lon[2], '\0'};
    double deg = atof(degrees);
    double min = atof(lon + 3);

    double longitude = deg + min / 60.0;

    if (ew[0] == 'W') {
        longitude = -longitude;
    }

    return longitude;
}

float GNSS_BE881::parseFloat(const char* str) {
    return atof(str);
}

int GNSS_BE881::parseInt(const char* str) {
    return atoi(str);
}

/**
 * @brief 提取 NMEA 句子中的字段
 */
void GNSS_BE881::extractField(const char* sentence, int field_num, char* buffer, int buf_size) {
    buffer[0] = '\0';

    int current_field = 0;
    const char* p = sentence;
    const char* field_start = NULL;

    while (*p && current_field <= field_num) {
        if (*p == ',' || *p == '*') {
            if (current_field == field_num && field_start) {
                int len = p - field_start;
                if (len >= buf_size) len = buf_size - 1;
                strncpy(buffer, field_start, len);
                buffer[len] = '\0';
                return;
            }
            current_field++;
            field_start = p + 1;
        }
        p++;
    }
}

int GNSS_BE881::countFields(const char* sentence) {
    int count = 1;
    for (const char* p = sentence; *p; p++) {
        if (*p == ',') count++;
    }
    return count;
}

/******************************************************************************
 *                            数据访问
 ******************************************************************************/

GNSS_Data GNSS_BE881::getData() {
    return data;
}

bool GNSS_BE881::isValid() {
    return data.is_valid;
}

uint8_t GNSS_BE881::getFixQuality() {
    return data.fix_quality;
}

uint8_t GNSS_BE881::getSatellites() {
    return data.satellites;
}

double GNSS_BE881::getLatitude() {
    return data.latitude;
}

double GNSS_BE881::getLongitude() {
    return data.longitude;
}

float GNSS_BE881::getAltitude() {
    return data.altitude;
}

float GNSS_BE881::getSpeedKmh() {
    return data.speed_kmh;
}

float GNSS_BE881::getSpeedMps() {
    return data.speed_mps;
}

float GNSS_BE881::getCourse() {
    return data.course;
}

void GNSS_BE881::getTime(uint8_t &h, uint8_t &m, uint8_t &s) {
    h = data.hour;
    m = data.minute;
    s = data.second;
}

void GNSS_BE881::getDate(uint8_t &y, uint8_t &m, uint8_t &d) {
    y = data.year - 2000;
    m = data.month;
    d = data.day;
}

float GNSS_BE881::getHDOP() {
    return data.hdop;
}

/******************************************************************************
 *                            诊断
 ******************************************************************************/

void GNSS_BE881::printDiagnostics() {
    Serial.println("\n========== GNSS 诊断信息 ==========");
    Serial.printf("消息计数: %u (错误: %u)\n", message_count, parse_error_count);
    Serial.printf("定位状态: %s\n", data.is_valid ? "有效" : "无效");
    Serial.printf("定位质量: %d (0=无, 1=GPS, 2=DGPS, 4=RTK)\n", data.fix_quality);
    Serial.printf("卫星数量: %d\n", data.satellites);
    Serial.printf("精度因子: HDOP=%.2f, PDOP=%.2f, VDOP=%.2f\n",
                  data.hdop, data.pdop, data.vdop);

    Serial.println("\n【位置】:");
    Serial.printf("  纬度: %.6f°\n", data.latitude);
    Serial.printf("  经度: %.6f°\n", data.longitude);
    Serial.printf("  海拔: %.1f m\n", data.altitude);

    Serial.println("\n【速度】:");
    Serial.printf("  速度: %.2f km/h (%.2f m/s)\n", data.speed_kmh, data.speed_mps);
    Serial.printf("  航向: %.1f°\n", data.course);

    Serial.println("\n【时间】:");
    Serial.printf("  UTC: %04d-%02d-%02d %02d:%02d:%02d.%03d\n",
                  data.year, data.month, data.day,
                  data.hour, data.minute, data.second, data.millisecond);

    Serial.printf("\n最后更新: %lu ms 前\n", millis() - data.last_update);
    Serial.println("====================================\n");
}

uint32_t GNSS_BE881::getMessageCount() {
    return message_count;
}

// 全局对象
GNSS_BE881 gnss;
