#include "SD_card.h"              // 头文件声明接口
#include <Arduino.h>               // Arduino 基础支持（String、millis 等）
#include <SD_MMC.h>                // SD/MMC 文件系统库（非 SPI）
#include "BLE.h"                  // 用于通过 BLE 输出调试信息

static bool _sd_mounted = false;   // 保存当前 SD 卡是否挂载成功

// 挂载 SD 卡：使用 1-bit SDMMC 模式（只接 DAT0），自定义引脚
// DAT0 -> IO21, CLK -> IO47, CMD -> IO48
// begin("/sdcard", true) 中第一个参数是挂载点名称，第二个参数 true 表示 1-bit 模式
void SD_card_init() {
  BLE_sendf("SD: mounting...");            // 通过 BLE 输出挂载开始日志
  SD_MMC.setPins(47, 48, 21, -1, -1, -1);   // 设置引脚：CLK=47, CMD=48, DAT0=21，其余未使用传 -1
  if (SD_MMC.begin("/sdcard", true)) {     // 尝试挂载，返回 true 表示成功
    _sd_mounted = true;                     // 记录挂载状态
    uint64_t sectors = SD_MMC.cardSize();   // cardSize() 返回扇区数（可能为 0 表示未知）
    uint64_t bytes = (sectors > 0 ? sectors * 512ULL : 0ULL); // 传统 SD 卡一个扇区 512 字节
    BLE_sendf("SD mounted, size: %llu bytes", bytes); // 输出容量信息
  } else {
    _sd_mounted = false;                    // 挂载失败
    BLE_sendf("SD mount failed");           // 输出失败日志
  }
}

// 返回是否已挂载，用于主循环判断是否可进行文件操作
bool SD_card_is_mounted() {
  return _sd_mounted;                       // 直接返回标志位
}

// 若未挂载则再尝试一次挂载；避免在 ISR 内直接调用挂载
void SD_card_try_mount_once() {
  if (_sd_mounted) return;                  // 已挂载不重复操作
  SD_card_init();                           // 重试挂载
}

// 创建或覆盖一个文件：传入绝对路径（建议以 / 开头，如 "/newfile.txt"）
// 使用 FILE_WRITE 模式会在存在时覆盖文件，不存在时新建
bool SD_card_create_file(const char *path) {
  if (!_sd_mounted) {                       // 未挂载则直接失败
    BLE_sendf("SD not mounted");
    return false;
  }
  if (!path || !*path) {                    // 路径为空则失败
    BLE_sendf("Invalid path");
    return false;
  }
  String p(path);                           // 将 C 字符串包装为 Arduino String
  if (!p.startsWith("/")) p = String("/") + p; // 若不以 '/' 开头则补一个前缀
  File f = SD_MMC.open(p.c_str(), FILE_APPEND);   // 打开文件用于写入（覆盖/创建）
  if (!f) {                                 // 打开失败
    BLE_sendf("Create FAIL:%s", p.c_str());
    return false;
  }
  f.print("");                             // 写入空字符串确保文件结构建立
  f.close();                                // 关闭文件释放句柄
  BLE_sendf("Create OK:%s", p.c_str());     // 成功日志
  return true;                              // 返回成功
}

// 追加时间戳 (HH:MM:SS) 行到文件末尾；seconds 为从 0 开始累计的总秒数
bool SD_card_append_timestamp(const char *path, int seconds) {
  if (!_sd_mounted) return false;                 // 未挂载直接失败
  if (!path || !*path) return false;              // 无效路径
  String p(path);
  if (!p.startsWith("/")) p = String("/") + p;  // 规范化前缀
  int sec = seconds % 86400;                 // 24h 循环
  int hh = sec / 3600;
  int mm = (sec % 3600) / 60;
  int ss = sec % 60;
  char line[16];
  snprintf(line, sizeof(line), "%02u:%02u:%02u\r\n", hh, mm, ss); // 格式化时间戳
  File f = SD_MMC.open(p.c_str(), FILE_APPEND);   // 以追加方式打开
  if (!f) {                                       // 如果文件不存在，尝试创建再追加
    f = SD_MMC.open(p.c_str(), FILE_WRITE);
    if (!f) return false;
    f.close();
    f = SD_MMC.open(p.c_str(), FILE_APPEND);
    if (!f) return false;
  }
  f.print(seconds/2);
  f.print(".");
  size_t w = f.print(line);                       // 写入时间戳行
  f.close();
  return w == strlen(line);
}

