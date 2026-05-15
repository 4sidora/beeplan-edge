/**
 * BeePlan edge — скелет прошивки (ESP32 + ESP-NOW).
 * Реализация датчиков и FFT переносится по мере готовности железа.
 */
#include <cstring>
#include <esp_now.h>
#include <WiFi.h>

#include "config.h"

namespace {

constexpr uint32_t kMagic = 0x00BEEF01;  // wire: little-endian 01 EF BE 00
constexpr uint8_t kProtoVersion = 1;

struct __attribute__((packed)) Envelope {
  uint32_t magic;
  uint8_t proto_version;
  char device_id[32];
  uint32_t unix_ts;
  uint8_t metric;  // 0 temp C x100, 1 rh % x100, 2 audio placeholder
  int16_t i16_a;
  int16_t i16_b;
};

void on_sent(const uint8_t* mac, esp_now_send_status_t status) {
  Serial.printf("ESP-NOW send status=%d\n", static_cast<int>(status));
}

bool espnow_init() {
  if (esp_now_init() != ESP_OK) {
    return false;
  }
  esp_now_register_send_cb(on_sent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, GATEWAY_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    return false;
  }
  return true;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("BeePlan edge starting");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);

  if (!espnow_init()) {
    Serial.println("esp_now init failed");
    abort();
  }
}

void loop() {
  Envelope msg{};
  msg.magic = kMagic;
  msg.proto_version = kProtoVersion;
  memset(msg.device_id, 0, sizeof(msg.device_id));
  strncpy(msg.device_id, DEVICE_PUBLIC_ID, sizeof(msg.device_id) - 1);
  msg.unix_ts = static_cast<uint32_t>(time(nullptr));
  if (msg.unix_ts < 1700000000) {
    msg.unix_ts = static_cast<uint32_t>(millis() / 1000);  // fallback until NTP
  }

  msg.metric = 0;
  msg.i16_a = static_cast<int16_t>(22.5f * 100);  // placeholder temperature
  msg.i16_b = 0;
  esp_now_send(GATEWAY_MAC, reinterpret_cast<uint8_t*>(&msg), sizeof(msg));

  msg.metric = 1;
  msg.i16_a = static_cast<int16_t>(55.0f * 100);  // placeholder RH
  msg.i16_b = 0;
  esp_now_send(GATEWAY_MAC, reinterpret_cast<uint8_t*>(&msg), sizeof(msg));

  delay(static_cast<uint32_t>(WAKE_INTERVAL_SEC) * 1000U);
}
