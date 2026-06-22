#include "beeplan_scales.h"

#include <Arduino.h>
#include <HX711.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <time.h>

#include "beeplan_io.h"
#include "beeplan_sensors.h"
#include "config.h"

namespace {

#if DEVICE_TYPE == 1

constexpr char kScalePrefs[] = "beeplan_scale";
constexpr int kSampleCount = 11;
constexpr float kDefaultScaleFactor = 4200.0f;
constexpr float kDefaultTempCoeffKgPerC = 0.0015f;
constexpr float kStabilitySpreadKg = 0.35f;

HX711 g_hx711;
OneWire g_one_wire(DS18B20_PIN);
DallasTemperature g_ds18b20(&g_one_wire);
Preferences g_scale_prefs;

bool g_scale_ready = false;
float g_scale_factor = kDefaultScaleFactor;
long g_scale_offset = 0;
float g_temp_ref_c = 20.0f;
float g_temp_coeff = kDefaultTempCoeffKgPerC;
float g_last_stable_kg = 0.0f;

void load_calibration() {
  g_scale_prefs.begin(kScalePrefs, true);
  g_scale_factor = g_scale_prefs.getFloat("factor", kDefaultScaleFactor);
  g_scale_offset = g_scale_prefs.getLong("offset", 0);
  g_temp_ref_c = g_scale_prefs.getFloat("temp_ref", 20.0f);
  g_temp_coeff = g_scale_prefs.getFloat("temp_coeff", kDefaultTempCoeffKgPerC);
  g_last_stable_kg = g_scale_prefs.getFloat("last_kg", 0.0f);
  g_scale_prefs.end();
}

float read_temperature_c() {
  g_ds18b20.requestTemperatures();
  const float t = g_ds18b20.getTempCByIndex(0);
  if (t > -55.0f && t < 125.0f) {
    return t;
  }
  return NAN;
}

bool read_median_kg(float& out_kg) {
  if (!g_scale_ready) {
    return false;
  }
  float samples[kSampleCount];
  int valid = 0;
  for (int i = 0; i < kSampleCount; ++i) {
    if (!g_hx711.wait_ready_timeout(1200)) {
      break;
    }
    const float kg = g_hx711.get_units(1);
    if (!std::isfinite(kg)) {
      continue;
    }
    samples[valid++] = kg;
    delay(40);
  }
  if (valid < 5) {
    return false;
  }
  std::sort(samples, samples + valid);
  const float median = samples[valid / 2];
  const float spread = samples[valid - 1] - samples[0];
  if (spread > kStabilitySpreadKg) {
    BEEPLAN_LOG("scales: unstable spread=%.2f kg, keep last\n", spread);
    out_kg = g_last_stable_kg;
    return g_last_stable_kg > 0.0f;
  }
  out_kg = median;
  return true;
}

float apply_temp_compensation(float kg, float temp_c) {
  if (!std::isfinite(temp_c)) {
    return kg;
  }
  return kg - g_temp_coeff * (temp_c - g_temp_ref_c);
}

float apply_weight_mode(float kg) {
#if WEIGHT_MODE_HALF
  return kg * 2.0f;
#else
  return kg;
#endif
}

void pack_firmware_version(uint16_t& major_minor, uint16_t& patch) {
  int maj = 0;
  int min = 0;
  int pat = 0;
  sscanf(FIRMWARE_VERSION, "%d.%d.%d", &maj, &min, &pat);
  major_minor = static_cast<uint16_t>((maj << 8) | (min & 0xff));
  patch = static_cast<uint16_t>(pat);
}

#endif  // DEVICE_TYPE == 1

}  // namespace

void beeplan_scales_init() {
#if DEVICE_TYPE == 1
  load_calibration();
  pinMode(HX711_SCK_PIN, OUTPUT);
  digitalWrite(HX711_SCK_PIN, LOW);
  g_hx711.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  g_hx711.set_scale(g_scale_factor);
  g_hx711.set_offset(g_scale_offset);
  g_scale_ready = g_hx711.wait_ready_timeout(2000);
  g_ds18b20.begin();
  BEEPLAN_LOG("scales init dout=%d sck=%d factor=%.1f offset=%ld half=%d ready=%d\n",
              HX711_DOUT_PIN, HX711_SCK_PIN, g_scale_factor, g_scale_offset,
#if WEIGHT_MODE_HALF
              1,
#else
              0,
#endif
              g_scale_ready ? 1 : 0);
#endif
}

void beeplan_scales_power_down() {
#if DEVICE_TYPE == 1
  if (g_scale_ready) {
    g_hx711.power_down();
  }
#endif
}

ReportFrameV2 beeplan_scales_build_report(uint32_t report_seq, bool include_firmware) {
#if DEVICE_TYPE != 1
  (void)report_seq;
  (void)include_firmware;
  return ReportFrameV2{};
#else
  int16_t signal_dbm = -127;
  float battery_volts = 0.0f;
  beeplan_sensors_read(signal_dbm, battery_volts);

  ReportFrameV2 msg{};
  msg.magic = kBeeplanMagicV2;
  msg.proto_version = kBeeplanProtoV2;
  msg.flags = kFlagScheduled;
  msg.seq = static_cast<uint8_t>(report_seq & 0xFFU);
  msg.device_type = 1;

  uint8_t metrics = kMetricSignal | kMetricBattery | kMetricAudio;
  const float temp_c = read_temperature_c();
  if (std::isfinite(temp_c)) {
    metrics |= kMetricTemp;
  }
  if (include_firmware) {
    metrics |= kMetricFirmware;
  }
  msg.metrics_present = metrics;

  memset(msg.device_id, 0, sizeof(msg.device_id));
  strncpy(msg.device_id, DEVICE_PUBLIC_ID, sizeof(msg.device_id) - 1);

  if (time(nullptr) > static_cast<time_t>(kMinValidUnixTs)) {
    msg.unix_ts = static_cast<uint32_t>(time(nullptr));
  } else {
    msg.unix_ts = report_seq;
  }

  float kg = 0.0f;
  if (read_median_kg(kg)) {
    kg = apply_temp_compensation(kg, temp_c);
    kg = apply_weight_mode(kg);
    if (kg < 0.0f) {
      kg = 0.0f;
    }
    if (kg > 320.0f) {
      kg = 320.0f;
    }
    g_last_stable_kg = kg;
    g_scale_prefs.begin(kScalePrefs, false);
    g_scale_prefs.putFloat("last_kg", kg);
    g_scale_prefs.end();
  } else if (g_last_stable_kg > 0.0f) {
    kg = g_last_stable_kg;
  }

  if (std::isfinite(temp_c)) {
    msg.temp_c_x100 = static_cast<int16_t>(lroundf(temp_c * 100.0f));
  }
  msg.rh_x100 = 0;
  msg.signal_dbm = signal_dbm;
  msg.battery_x100 = static_cast<int16_t>(battery_volts * 100.0f);
  msg.audio_rms_x1000 = static_cast<int16_t>(lroundf(kg * 100.0f));
  msg.audio_peak_hz = 0;

  if ((metrics & kMetricFirmware) != 0) {
    uint16_t fw_major_minor = 0;
    uint16_t fw_patch = 0;
    pack_firmware_version(fw_major_minor, fw_patch);
    msg.fw_major_minor = fw_major_minor;
    msg.fw_patch = fw_patch;
  }

  BEEPLAN_LOG("scales report kg=%.2f temp=%.1fC bat=%.2fV\n", kg,
              std::isfinite(temp_c) ? temp_c : -999.0f, battery_volts);
  return msg;
#endif
}
