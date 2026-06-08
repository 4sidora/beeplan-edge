/**
 * BeePlan edge — ESP-NOW v2, deep sleep, TDMA slot.
 */
#include <Arduino.h>
#include <cstring>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <time.h>
#include <WiFi.h>

#include "beeplan_io.h"
#include "beeplan_espnow.h"
#include "config.h"
#include "envelope_v2.h"

namespace {

#if GATEWAY_WIFI_CHANNEL < 1 || GATEWAY_WIFI_CHANNEL > 13
#error "GATEWAY_WIFI_CHANNEL must be 1–13 in ESP-NOW v2"
#endif

volatile esp_now_send_status_t g_last_send_status = ESP_NOW_SEND_FAIL;
volatile bool g_send_done = false;
volatile bool g_ack_received = false;
uint8_t g_tx_seq = 0;

float g_demo_temp_c = 22.5f;
float g_demo_humidity_pct = 55.0f;
int16_t g_demo_signal_dbm = -68;
float g_demo_battery_pct = 88.0f;

void pack_firmware_version(uint16_t& major_minor, uint16_t& patch) {
  int maj = 0;
  int min = 0;
  int pat = 0;
  sscanf(FIRMWARE_VERSION, "%d.%d.%d", &maj, &min, &pat);
  major_minor = static_cast<uint16_t>((maj << 8) | (min & 0xff));
  patch = static_cast<uint16_t>(pat);
}

void randomize_device_status() {
  g_demo_signal_dbm = static_cast<int16_t>(random(-88, -48));
  g_demo_battery_pct += static_cast<float>(random(-15, 6)) / 10.0f;
  if (g_demo_battery_pct < 5.0f) {
    g_demo_battery_pct = 95.0f;
  }
  if (g_demo_battery_pct > 100.0f) {
    g_demo_battery_pct = 100.0f;
  }
}

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

void on_recv_legacy(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
  if (len != static_cast<int>(sizeof(AckFrameV2))) {
    return;
  }
  AckFrameV2 ack{};
  memcpy(&ack, data, sizeof(ack));
  if (ack.magic != kBeeplanMagicAck || ack.proto_version != kBeeplanProtoV2) {
    return;
  }
  if (strncmp(ack.device_id, DEVICE_PUBLIC_ID, sizeof(ack.device_id)) != 0) {
    return;
  }
  if (ack.ack_seq == g_tx_seq) {
    g_ack_received = true;
  }
}

#if BEEPLAN_ESPNOW_V3
void on_recv_v3(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  (void)info;
  on_recv_legacy(nullptr, data, len);
}
#endif

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
  return err == ESP_OK;
}

bool wait_send_done(uint32_t timeout_ms) {
  const uint32_t start = millis();
  while (!g_send_done && (millis() - start) < timeout_ms) {
    delay(1);
  }
  return g_send_done;
}

bool send_report(ReportFrameV2& msg) {
  g_send_done = false;
  g_ack_received = false;
  g_last_send_status = ESP_NOW_SEND_FAIL;
  const esp_err_t err =
      esp_now_send(GATEWAY_MAC, reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
  if (err != ESP_OK) {
    return false;
  }
  if (!wait_send_done(300)) {
    return false;
  }
  if (g_last_send_status != ESP_NOW_SEND_SUCCESS) {
    return false;
  }
  const uint32_t ack_start = millis();
  while ((millis() - ack_start) < 50) {
    if (g_ack_received) {
      return true;
    }
    delay(1);
  }
  return false;
}

bool espnow_init() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  WiFi.setSleep(false);
  delay(100);

  const uint8_t channel = static_cast<uint8_t>(GATEWAY_WIFI_CHANNEL);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    return false;
  }
#if BEEPLAN_ESPNOW_V3
  if (!beeplan_register_send_cb(on_sent_v3) || !beeplan_register_recv_cb(on_recv_v3)) {
    return false;
  }
#else
  if (!beeplan_register_send_cb(on_sent_legacy) || !beeplan_register_recv_cb(on_recv_legacy)) {
    return false;
  }
#endif
  return ensure_peer_on_channel(channel);
}

uint32_t current_epoch() {
  time_t now = time(nullptr);
  if (now > 1700000000) {
    return static_cast<uint32_t>(now);
  }
  return static_cast<uint32_t>(millis() / 1000);
}

uint32_t seconds_until_next_slot() {
  const uint32_t epoch = current_epoch();
  const uint32_t hour_start = epoch - (epoch % WAKE_INTERVAL_SEC);
  uint32_t slot_ts = hour_start + static_cast<uint32_t>(TELEMETRY_SLOT_SEC);
  if (slot_ts <= epoch) {
    slot_ts += WAKE_INTERVAL_SEC;
  }
  return slot_ts - epoch;
}

void sleep_until_next_slot() {
  const uint32_t delay_sec = seconds_until_next_slot();
  BEE_SERIAL.printf("deep sleep %us until slot %u\n", delay_sec,
                    static_cast<unsigned>(TELEMETRY_SLOT_SEC));
  BEE_SERIAL.flush();
  esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(delay_sec) * 1000000ULL);
  esp_deep_sleep_start();
}

ReportFrameV2 build_report() {
  randomize_demo_metrics();
  randomize_device_status();

  ReportFrameV2 msg{};
  msg.magic = kBeeplanMagicV2;
  msg.proto_version = kBeeplanProtoV2;
  msg.flags = kFlagScheduled;
  msg.seq = ++g_tx_seq;
  msg.device_type = static_cast<uint8_t>(DEVICE_TYPE);
  msg.metrics_present = kMetricTemp | kMetricRh | kMetricSignal | kMetricBattery | kMetricFirmware;
  memset(msg.device_id, 0, sizeof(msg.device_id));
  strncpy(msg.device_id, DEVICE_PUBLIC_ID, sizeof(msg.device_id) - 1);
  msg.unix_ts = current_epoch();
  msg.temp_c_x100 = static_cast<int16_t>(g_demo_temp_c * 100.0f);
  msg.rh_x100 = static_cast<int16_t>(g_demo_humidity_pct * 100.0f);
  msg.signal_dbm = g_demo_signal_dbm;
  msg.battery_x100 = static_cast<int16_t>(g_demo_battery_pct * 100.0f);
  pack_firmware_version(msg.fw_major_minor, msg.fw_patch);
  msg.audio_rms_x1000 = 0;
  msg.audio_peak_hz = 0;
  return msg;
}

void transmit_with_retry() {
  ReportFrameV2 msg = build_report();
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (send_report(msg)) {
      BEE_SERIAL.printf("ReportFrameV2 sent seq=%u ack ok\n", static_cast<unsigned>(msg.seq));
      return;
    }
    BEE_SERIAL.printf("ReportFrameV2 retry %d seq=%u\n", attempt + 1, static_cast<unsigned>(msg.seq));
    delay(50 + attempt * 50);
  }
  beeplan_led_toggle();
  BEE_SERIAL.println("ReportFrameV2 failed after retries");
}

}  // namespace

void setup() {
  beeplan_led_init();
  beeplan_serial_begin();
  BEE_SERIAL.printf("BeePlan edge %s\n", FIRMWARE_SERIAL_TAG);
  BEE_SERIAL.printf("sizeof(ReportFrameV2)=%u slot=%u channel=%u\n",
                    static_cast<unsigned>(sizeof(ReportFrameV2)),
                    static_cast<unsigned>(TELEMETRY_SLOT_SEC),
                    static_cast<unsigned>(GATEWAY_WIFI_CHANNEL));
  randomSeed(esp_random());

  if (!espnow_init()) {
    BEE_SERIAL.println("FATAL: esp_now init failed");
    while (true) {
      beeplan_led_toggle();
      delay(150);
    }
  }

  transmit_with_retry();
  sleep_until_next_slot();
}

void loop() {
  // unreachable — deep sleep in setup()
}
