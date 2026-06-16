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
#include <sys/time.h>
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
/** Интервал замера из ACK gateway; 0 — использовать WAKE_INTERVAL_SEC из прошивки. */
RTC_DATA_ATTR uint32_t g_runtime_wake_interval_sec = 0;
/** Последний WAKE_INTERVAL_SEC из прошивки (для сброса runtime после перепрошивки). */
RTC_DATA_ATTR uint32_t g_rtc_compiled_wake_sec = 0;
/** Unix-время последней отправки метрики firmware_version. */
RTC_DATA_ATTR uint32_t g_last_firmware_report_ts = 0;

Preferences g_prefs;

constexpr uint32_t kFirmwareReportMinIntervalSec = 86400U;

void reconcile_runtime_wake_interval() {
  const uint32_t compiled = static_cast<uint32_t>(WAKE_INTERVAL_SEC);
  if (g_rtc_compiled_wake_sec != compiled) {
    g_runtime_wake_interval_sec = 0;
    g_rtc_compiled_wake_sec = compiled;
    BEEPLAN_LOG("wake_interval reset (compiled=%u s)\n", static_cast<unsigned>(compiled));
  }
}

uint32_t effective_wake_interval_sec() {
  if (g_runtime_wake_interval_sec >= 10U && g_runtime_wake_interval_sec <= 86400U) {
    return g_runtime_wake_interval_sec;
  }
  return static_cast<uint32_t>(WAKE_INTERVAL_SEC);
}

void apply_wake_interval_from_ack(uint16_t wake_sec) {
  if (wake_sec < 10U || wake_sec > 86400U) {
    return;
  }
  if (g_runtime_wake_interval_sec != static_cast<uint32_t>(wake_sec)) {
    g_runtime_wake_interval_sec = wake_sec;
    BEEPLAN_LOG("wake_interval set to %u s from gateway\n", static_cast<unsigned>(wake_sec));
  }
}

bool wall_clock_valid() {
  return time(nullptr) > static_cast<time_t>(kMinValidUnixTs);
}

bool should_include_firmware_metric() {
  const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
    return true;
  }
  if (g_last_firmware_report_ts == 0) {
    return true;
  }
  if (!wall_clock_valid()) {
    return false;
  }
  return time(nullptr) - static_cast<time_t>(g_last_firmware_report_ts) >=
         static_cast<time_t>(kFirmwareReportMinIntervalSec);
}

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

void apply_gateway_time(uint32_t gateway_unix_ts) {
  if (gateway_unix_ts < kMinValidUnixTs) {
    return;
  }
  const struct timeval tv = {
      .tv_sec = static_cast<time_t>(gateway_unix_ts),
      .tv_usec = 0,
  };
  settimeofday(&tv, nullptr);
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
  if (len < static_cast<int>(kAckFrameV2LegacyLen)) {
    return;
  }
  AckFrameV2 ack{};
  memset(&ack, 0, sizeof(ack));
  memcpy(&ack, data, static_cast<size_t>(len < static_cast<int>(sizeof(ack)) ? len : sizeof(ack)));
  if (ack.magic != kBeeplanMagicAck) {
    return;
  }
  if (ack.proto_version != kBeeplanProtoV2 && ack.proto_version != kBeeplanProtoAckV3) {
    return;
  }
  if (strncmp(ack.device_id, DEVICE_PUBLIC_ID, sizeof(ack.device_id)) != 0) {
    return;
  }
  apply_gateway_time(ack.gateway_unix_ts);
  if (ack.proto_version >= kBeeplanProtoAckV3 &&
      len >= static_cast<int>(sizeof(AckFrameV2))) {
    apply_wake_interval_from_ack(ack.wake_interval_sec);
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

  if (!beeplan_espnow_enable_lr()) {
    BEEPLAN_LOGLN("WARN: ESP-NOW LR mode not enabled");
  }

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
  const uint32_t wake_sec = effective_wake_interval_sec();
  if (wake_sec < kTdmaMinWakeIntervalSec) {
    return wake_sec;
  }
  const uint32_t epoch = current_epoch();
  const uint32_t cycle_start = epoch - (epoch % wake_sec);
  uint32_t slot_ts = cycle_start + static_cast<uint32_t>(TELEMETRY_SLOT_SEC);
  if (slot_ts <= epoch) {
    slot_ts += wake_sec;
  }
  return slot_ts - epoch;
}

void sleep_until_next_slot() {
  const uint32_t wake_sec = effective_wake_interval_sec();
  const uint32_t delay_sec = seconds_until_next_slot();
  if (wake_sec < kTdmaMinWakeIntervalSec) {
    BEEPLAN_LOG("deep sleep %us (wake %us, TDMA off)\n", delay_sec, static_cast<unsigned>(wake_sec));
  } else {
    BEEPLAN_LOG("deep sleep %us until slot %u\n", delay_sec,
                static_cast<unsigned>(TELEMETRY_SLOT_SEC));
  }
#if BEEPLAN_DEBUG
  BEE_SERIAL.flush();
#endif
  esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(delay_sec) * 1000000ULL);
  esp_deep_sleep_start();
}

ReportFrameV2 build_report(uint32_t iteration, uint32_t report_seq) {
  int16_t signal_dbm = -127;
  float battery_volts = 0.0f;
  beeplan_sensors_read(signal_dbm, battery_volts);

  ReportFrameV2 msg{};
  msg.magic = kBeeplanMagicV2;
  msg.proto_version = kBeeplanProtoV2;
  msg.flags = kFlagScheduled;
  msg.seq = static_cast<uint8_t>(report_seq & 0xFFU);
  g_tx_seq = msg.seq;
  msg.device_type = static_cast<uint8_t>(DEVICE_TYPE);
  uint8_t metrics = kMetricTemp | kMetricRh | kMetricSignal | kMetricBattery;
  if (should_include_firmware_metric()) {
    metrics |= kMetricFirmware;
  }
  msg.metrics_present = metrics;
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
  msg.battery_x100 = static_cast<int16_t>(battery_volts * 100.0f);
  if ((metrics & kMetricFirmware) != 0) {
    uint16_t fw_major_minor = 0;
    uint16_t fw_patch = 0;
    pack_firmware_version(fw_major_minor, fw_patch);
    msg.fw_major_minor = fw_major_minor;
    msg.fw_patch = fw_patch;
  }
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
      if ((msg.metrics_present & kMetricFirmware) != 0) {
        if (wall_clock_valid()) {
          g_last_firmware_report_ts = static_cast<uint32_t>(time(nullptr));
        } else {
          g_last_firmware_report_ts = 1;
        }
      }
      BEEPLAN_LOG(
          "ReportFrameV2 sent seq=%u iter=%lu report=%lu bat=%.2fV rssi=%d wake=%u ack ok\n",
          static_cast<unsigned>(msg.seq), static_cast<unsigned long>(g_send_iteration),
          static_cast<unsigned long>(report_seq), msg.battery_x100 / 100.0f,
          static_cast<int>(g_link_rssi_dbm), static_cast<unsigned>(effective_wake_interval_sec()));
      return;
    }
    BEEPLAN_LOG("ReportFrameV2 retry %d seq=%u\n", attempt + 1, static_cast<unsigned>(msg.seq));
    delay(50 + attempt * 50);
  }
  beeplan_led_toggle();
  BEEPLAN_LOGLN("ReportFrameV2 failed after retries");
}

/** GPIO0 (BOOT/PRG): удержание при старте — режим USB-прошивки без deep sleep. */
bool boot_button_held() {
  pinMode(0, INPUT_PULLUP);
  delay(20);
  return digitalRead(0) == LOW;
}

void install_mode_loop() {
  BEE_SERIAL.println("INSTALL mode (BOOT held) — deep sleep disabled, ready for USB flash");
  while (true) {
    delay(5000);
    BEE_SERIAL.println("INSTALL mode: waiting for WebSerial...");
  }
}

}  // namespace

void setup() {
  beeplan_led_init();
  beeplan_sensors_init();
  beeplan_serial_begin();
  reconcile_runtime_wake_interval();
  if (boot_button_held()) {
    install_mode_loop();
  }
  BEEPLAN_LOG("BeePlan edge %s\n", FIRMWARE_SERIAL_TAG);
  BEEPLAN_LOG("sizeof(ReportFrameV2)=%u slot=%u channel=%u wake=%u\n",
              static_cast<unsigned>(sizeof(ReportFrameV2)),
              static_cast<unsigned>(TELEMETRY_SLOT_SEC),
              static_cast<unsigned>(GATEWAY_WIFI_CHANNEL),
              static_cast<unsigned>(effective_wake_interval_sec()));
  BEEPLAN_LOG("report_seq nvs=%lu clock=%s\n", static_cast<unsigned long>(load_report_seq()),
              wall_clock_valid() ? "ntp" : "none");
  randomSeed(esp_random());

  if (!espnow_init()) {
    BEEPLAN_LOGLN("FATAL: esp_now init failed");
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
