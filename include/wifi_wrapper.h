#ifndef WIFI_WRAPPER_H_
#define WIFI_WRAPPER_H_

#include <Arduino.h>
#include <WiFi.h>

// 初始化（可选）
void WiFi_init();

// 连接到指定 SSID/Password（阻塞直到连接或超时）
void WiFi_Connect(const char *ssid, const char *password);

// 断开连接
void WiFi_Disconnect();

// 是否已连接
bool WiFi_isConnected();

#endif // WIFI_WRAPPER_H_