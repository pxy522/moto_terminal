#ifndef __SD_CARD_H_
#define __SD_CARD_H_

// 初始化并尝试挂载 SD 卡（使用 SDMMC，用户指定 DAT0=IO21, CLK=IO47, CMD=IO48）
void SD_card_init();

// 返回 SD 是否已挂载
bool SD_card_is_mounted();

// 如果未挂载则尝试挂载一次（适合在主循环调用，避免在 ISR 中执行）
void SD_card_try_mount_once();

// 创建(或覆盖)一个文本文件，成功返回 true，失败返回 false
bool SD_card_create_file(const char *path);

// 追加一个格式化时间戳 (HH:MM:SS) 到指定文件；seconds 为自增秒数
bool SD_card_append_timestamp(const char *path, int seconds);

#endif // SD_CARD_H_