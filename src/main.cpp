/**
 * BeePlan edge — ESP-NOW v2, deep sleep, TDMA slot.
 */
#include <Arduino.h>
#include <Preferences.h>
#include <cstring>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <time.h>
#include <WiFi.h>

#include "beeplan_io.h"
#include "beeplan_espnow.h"
#include "beeplan_sensors.h"
#include "config.h"
#include "envelope_v2.h"

namespace {

#if GATEWAY_WIFI_CHANNEL < 1 || GATEWAY_WIFI_CHANNEL > 13
#error "GATEWAY_WIFI_CHANNEL must be 1–13 in ESP-NOW v2"
#endif

constexpr char kPrefsNamespace[] = "beeplan";
constexpr char kPrefsReportSeq[] = "report_seq";

volatile esp_now_send_status_t g_last_send_status = ESP_NOW_SEND_FAIL;
volatile bool g_send_done = false;
volatile bool g_ack_received = false;
uint8_t g_tx_seq = 0;

/** Счётчик успешных отправок — сохраняется между deep sleep. */
RTC_DATA_ATTR uint32_t g_send_iteration = 0;

Preferences g_prefs;

uint32_t load_report_seq() {
  g_prefs.begin(kPrefsNamespace, true);
  const uint32_t seq = g_prefs.getUInt(kPrefsReportSeq, 0);
  g_prefs.end();
  return seq;
}

void save_report_seq(uint32_t seq) {
  g_prefs.begin(kPrefsNamespace, false);
  g_prefs.putUInt(kPrefsReportSeq, seq);
  g_prefs.end();
}

bool wall_clock_valid() {
  return time(nullptr) > static_cast<time_t>(kMinValidUnixTs);
}

void pack_firmware_version(uint16_t& major_minor, uint16_t& patch) {
  int maj = 0;
  int min = 0;
  int pat = 0;
  sscanf(FIRMWARE_VERSION, "%d.%d.%d", &maj, &min, &pat);
  major_minor = static_cast<uint16_t>((maj << 8) | (min & 0xff));
  patch = static_cast<uint16_t>(pat);
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
  if (info != nullptr && info->rx_ctrl != nullptr) {
    beeplan_sensors_on_ack_rssi(info->rx_ctrl->rssi);
  }
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
  while ((millis() - ack_start) < 300) {
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
  if (wall_clock_valid()) {
    return static_cast<uint32_t>(time(nullptr));
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

ReportFrameV2 build_report(uint32_t iteration, uint32_t report_seq) {
  int16_t signal_dbm = -127;
  float battery_pct = 0.0f;
  beeplan_sensors_read(signal_dbm, battery_pct);

  ReportFrameV2 msg{};
  msg.magic = kBeeplanMagicV2;
  msg.proto_version = kBeeplanProtoV2;
  msg.flags = kFlagScheduled;
  msg.seq = static_cast<uint8_t>(report_seq & 0xFFU);
  g_tx_seq = msg.seq;
  msg.device_type = static_cast<uint8_t>(DEVICE_TYPE);
  msg.metrics_present = kMetricTemp | kMetricRh | kMetricSignal | kMetricBattery | kMetricFirmware;
  memset(msg.device_id, 0, sizeof(msg.device_id));
  strncpy(msg.device_id, DEVICE_PUBLIC_ID, sizeof(msg.device_id) - 1);
  if (wall_clock_valid()) {
    msg.unix_ts = static_cast<uint32_t>(time(nullptr));
  } else {
    // Без NTP: unix_ts = монотонный NVS-счётчик (gateway → report_id edge-e2f56588:nvs:N).
    msg.unix_ts = report_seq;
  }
  // Тестовый режим: номер успешной итерации как t°C и RH%.
  msg.temp_c_x100 = static_cast<int16_t>(iteration * 100);
  msg.rh_x100 = static_cast<int16_t>(iteration * 100);
  msg.signal_dbm = signal_dbm;
  msg.battery_x100 = static_cast<int16_t>(battery_pct * 100.0f);
  uint16_t fw_major_minor = 0;
  uint16_t fw_patch = 0;
  pack_firmware_version(fw_major_minor, fw_patch);
  msg.fw_major_minor = fw_major_minor;
  msg.fw_patch = fw_patch;
  msg.audio_rms_x1000 = 0;
  msg.audio_peak_hz = 0;
  return msg;
}

void transmit_with_retry() {
  const uint32_t next_iteration = g_send_iteration + 1;
  const uint32_t report_seq = load_report_seq() + 1;
  ReportFrameV2 msg = build_report(next_iteration, report_seq);
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (send_report(msg)) {
      g_send_iteration = next_iteration;
      save_report_seq(report_seq);
      BEE_SERIAL.printf(
          "ReportFrameV2 sent seq=%u iter=%lu report=%lu bat=%.0f%% rssi=%d ack ok\n",
          static_cast<unsigned>(msg.seq), static_cast<unsigned long>(g_send_iteration),
          static_cast<unsigned long>(report_seq), msg.battery_x100 / 100.0f,
          static_cast<int>(g_link_rssi_dbm));
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
  beeplan_sensors_init();
  beeplan_serial_begin();
  BEE_SERIAL.printf("BeePlan edge %s\n", FIRMWARE_SERIAL_TAG);
  BEE_SERIAL.printf("sizeof(ReportFrameV2)=%u slot=%u channel=%u\n",
                    static_cast<unsigned>(sizeof(ReportFrameV2)),
                    static_cast<unsigned>(TELEMETRY_SLOT_SEC),
                    static_cast<unsigned>(GATEWAY_WIFI_CHANNEL));
  BEE_SERIAL.printf("report_seq nvs=%lu clock=%s\n",
                    static_cast<unsigned long>(load_report_seq()),
                    wall_clock_valid() ? "ntp" : "none");
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
