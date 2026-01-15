#include "wifi_wrapper.h"
#include "BLE.h"

void WiFi_init() {
  // 占位：如需设定 static IP、回调等在这里添加
}

void WiFi_Connect(const char *ssid, const char *password) {
  WiFi.begin(ssid, password);
  BLE_sendf("Connecting to %s", ssid);
  unsigned long start = millis();
  const unsigned long TIMEOUT = 5000; // 5s超时时间
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < TIMEOUT) {
    delay(200);
    BLE_sendf(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    BLE_sendf("WiFi connected");
    BLE_sendf("IP:%s", WiFi.localIP().toString().c_str());
  } else {
    BLE_sendf("WiFi connect timeout");
  }
}

void WiFi_Disconnect() {
  WiFi.disconnect(true);
}

bool WiFi_isConnected() {
  return WiFi.status() == WL_CONNECTED;
}
