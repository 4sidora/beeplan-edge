/**
 * BeePlan edge — скелет прошивки (ESP32 + ESP-NOW).
 */
#include <Arduino.h>
#include <cstring>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include "beeplan_io.h"
#include "beeplan_espnow.h"
#include "config.h"

namespace {

constexpr uint32_t kMagic = 0x00BEEF01;
constexpr uint8_t kProtoVersion = 1;

struct __attribute__((packed)) Envelope {
  uint32_t magic;
  uint8_t proto_version;
  char device_id[32];
  uint32_t unix_ts;
  uint8_t metric;
  int16_t i16_a;
  int16_t i16_b;
};

volatile esp_now_send_status_t g_last_send_status = ESP_NOW_SEND_FAIL;
volatile bool g_send_done = false;
uint8_t g_wifi_channel = 0;

float g_demo_temp_c = 22.5f;
float g_demo_humidity_pct = 55.0f;

void randomize_demo_metrics() {
  g_demo_temp_c += static_cast<float>(random(-20, 21)) / 10.0f;
  g_demo_humidity_pct += static_cast<float>(random(-25, 26)) / 10.0f;
  if (g_demo_temp_c < 15.0f) {
    g_demo_temp_c = 15.0f;
  }
  if (g_demo_temp_c > 32.0f) {
    g_demo_temp_c = 32.0f;
  }
  if (g_demo_humidity_pct < 35.0f) {
    g_demo_humidity_pct = 35.0f;
  }
  if (g_demo_humidity_pct > 85.0f) {
    g_demo_humidity_pct = 85.0f;
  }
}

void on_sent_legacy(const uint8_t* mac, esp_now_send_status_t status) {
  (void)mac;
  g_last_send_status = status;
  g_send_done = true;
}

#if BEEPLAN_ESPNOW_V3
void on_sent_v3(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  (void)info;
  g_last_send_status = status;
  g_send_done = true;
}
#endif

void log_esp_err(const char* what, esp_err_t err) {
  BEE_SERIAL.printf("%s: %s (0x%x)\n", what, esp_err_to_name(err), static_cast<unsigned>(err));
}

bool ensure_peer_on_channel(uint8_t channel) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, GATEWAY_MAC, 6);
  peer.channel = channel;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_err_t err;
  if (esp_now_is_peer_exist(GATEWAY_MAC)) {
    err = esp_now_mod_peer(&peer);
  } else {
    err = esp_now_add_peer(&peer);
  }
  if (err != ESP_OK) {
    log_esp_err("esp_now peer", err);
  }
  return err == ESP_OK;
}

bool wait_send_done(uint32_t timeout_ms) {
  const uint32_t start = millis();
  while (!g_send_done && (millis() - start) < timeout_ms) {
    delay(1);
  }
  return g_send_done;
}

bool send_envelope(Envelope& msg) {
  g_send_done = false;
  g_last_send_status = ESP_NOW_SEND_FAIL;
  const esp_err_t err =
      esp_now_send(GATEWAY_MAC, reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
  if (err != ESP_OK) {
    log_esp_err("esp_now_send() rejected", err);
    return false;
  }
  if (!wait_send_done(300)) {
    BEE_SERIAL.println("ESP-NOW send timeout (no callback)");
    return false;
  }
  BEE_SERIAL.printf("ESP-NOW send status=%d (0=ok)\n", static_cast<int>(g_last_send_status));
  return g_last_send_status == ESP_NOW_SEND_SUCCESS;
}

bool discover_gateway_channel() {
  Envelope probe{};
  probe.magic = kMagic;
  probe.proto_version = kProtoVersion;
  memset(probe.device_id, 0, sizeof(probe.device_id));
  strncpy(probe.device_id, DEVICE_PUBLIC_ID, sizeof(probe.device_id) - 1);
  probe.unix_ts = 0;
  probe.metric = 0;
  probe.i16_a = 0;
  probe.i16_b = 0;

  BEE_SERIAL.println("ESP-NOW: scanning WiFi channels 1-13...");
  for (uint8_t ch = 1; ch <= 13; ++ch) {
    if (esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
      continue;
    }
    if (!ensure_peer_on_channel(ch)) {
      continue;
    }
    if (send_envelope(probe)) {
      g_wifi_channel = ch;
      BEE_SERIAL.printf("Gateway reachable on WiFi channel %u\n", static_cast<unsigned>(ch));
      return true;
    }
  }
  return false;
}

bool espnow_init() {
  WiFi.mode(WIFI_STA);
  // wifioff=false: Wi-Fi стек должен остаться включённым для ESP-NOW (disconnect(true,*) гасит радио).
  WiFi.disconnect(false, true);
  WiFi.setSleep(false);
  delay(100);

  if (esp_now_init() != ESP_OK) {
    return false;
  }
#if BEEPLAN_ESPNOW_V3
  if (!beeplan_register_send_cb(on_sent_v3)) {
    return false;
  }
#else
  if (!beeplan_register_send_cb(on_sent_legacy)) {
    return false;
  }
#endif

#if GATEWAY_WIFI_CHANNEL > 0
  g_wifi_channel = static_cast<uint8_t>(GATEWAY_WIFI_CHANNEL);
  if (!ensure_peer_on_channel(g_wifi_channel)) {
    return false;
  }
  esp_wifi_set_channel(g_wifi_channel, WIFI_SECOND_CHAN_NONE);
#else
  if (!ensure_peer_on_channel(1)) {
    return false;
  }
#endif
  return true;
}

}  // namespace

void setup() {
  beeplan_led_init();
  beeplan_serial_begin();
  BEE_SERIAL.printf("BeePlan edge starting %s\n", FIRMWARE_SERIAL_TAG);
#if BEEPLAN_ESPNOW_V3
  BEE_SERIAL.println("ESP-NOW callbacks: IDF5/v3");
#else
  BEE_SERIAL.println("ESP-NOW callbacks: legacy");
#endif
  BEE_SERIAL.printf("GATEWAY_MAC=%02X:%02X:%02X:%02X:%02X:%02X\n", GATEWAY_MAC[0], GATEWAY_MAC[1],
                    GATEWAY_MAC[2], GATEWAY_MAC[3], GATEWAY_MAC[4], GATEWAY_MAC[5]);
  BEE_SERIAL.printf("sizeof(Envelope)=%u\n", static_cast<unsigned>(sizeof(Envelope)));
  randomSeed(esp_random());
  BEE_SERIAL.flush();

  if (!espnow_init()) {
    BEE_SERIAL.println("FATAL: esp_now init failed");
    BEE_SERIAL.flush();
    while (true) {
      beeplan_led_toggle();
      delay(150);
    }
  }

#if GATEWAY_WIFI_CHANNEL == 0
  if (!discover_gateway_channel()) {
    BEE_SERIAL.println("WARN: auto channel failed; retrying each wake cycle");
  }
#endif

  BEE_SERIAL.printf("DEVICE_PUBLIC_ID=%s WAKE_INTERVAL_SEC=%u channel=%u\n", DEVICE_PUBLIC_ID,
                    static_cast<unsigned>(WAKE_INTERVAL_SEC), static_cast<unsigned>(g_wifi_channel));
  BEE_SERIAL.flush();
}

void loop() {
  beeplan_led_toggle();

  BEE_SERIAL.printf("edge wake cycle (interval %us)\n", static_cast<unsigned>(WAKE_INTERVAL_SEC));
  BEE_SERIAL.flush();

#if GATEWAY_WIFI_CHANNEL == 0
  if (g_wifi_channel == 0) {
    discover_gateway_channel();
  }
#endif
  if (g_wifi_channel > 0) {
    esp_wifi_set_channel(g_wifi_channel, WIFI_SECOND_CHAN_NONE);
    ensure_peer_on_channel(g_wifi_channel);
  }

  Envelope msg{};
  msg.magic = kMagic;
  msg.proto_version = kProtoVersion;
  memset(msg.device_id, 0, sizeof(msg.device_id));
  strncpy(msg.device_id, DEVICE_PUBLIC_ID, sizeof(msg.device_id) - 1);
  msg.unix_ts = static_cast<uint32_t>(time(nullptr));
  if (msg.unix_ts < 1700000000) {
    msg.unix_ts = static_cast<uint32_t>(millis() / 1000);
  }

  if (g_wifi_channel == 0) {
    BEE_SERIAL.println("skip send: gateway channel unknown (fix WiFi/ESP-NOW or set GATEWAY_WIFI_CHANNEL)");
  } else {
    randomize_demo_metrics();
    msg.metric = 0;
    msg.i16_a = static_cast<int16_t>(g_demo_temp_c * 100.0f);
    msg.i16_b = 0;
    send_envelope(msg);

    msg.metric = 1;
    msg.i16_a = static_cast<int16_t>(g_demo_humidity_pct * 100.0f);
    msg.i16_b = 0;
    send_envelope(msg);
    BEE_SERIAL.printf("demo metrics: %.1f C, %.1f %%RH\n", static_cast<double>(g_demo_temp_c),
                      static_cast<double>(g_demo_humidity_pct));
  }
  BEE_SERIAL.flush();

  delay(static_cast<uint32_t>(WAKE_INTERVAL_SEC) * 1000U);
}
