#include "Task.h"
#include <WiFi.h>//WiFi
#include <HTTPClient.h>//HTTP
#include <BLE.h>
#include <Wire.h>// I2C 
#include <TCA9548A.h>//I2C 多路复用器
#include <JY901.h>//IMU传感器
#include "DFRobot_AS7341.h"
#include "485SN.h" // RS485 Modbus 传感器
#include "wifi_wrapper.h"//WiFi和API
#include "SD_card.h"

extern volatile bool send485_flag;

void Task1()
{

}