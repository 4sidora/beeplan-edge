#include "beeplan_sensors.h"

#include <Arduino.h>

int16_t g_link_rssi_dbm = -127;

namespace {

#if defined(BEEPLAN_BOARD_T_ENERGY) && BEEPLAN_BOARD_T_ENERGY

constexpr int kBatAdcPin = 35;
constexpr int kBatSampleCount = 16;
/** Делитель 100k/100k на T-Energy T18 V3.0. */
constexpr float kBatDividerRatio = 2.0f;
constexpr float kLiIonEmptyV = 3.30f;
constexpr float kLiIonFullV = 4.20f;

float read_battery_voltage() {
  uint32_t sum_mv = 0;
  for (int i = 0; i < kBatSampleCount; ++i) {
    sum_mv += static_cast<uint32_t>(analogReadMilliVolts(kBatAdcPin));
    delay(2);
  }
  const float adc_v = static_cast<float>(sum_mv) / static_cast<float>(kBatSampleCount) / 1000.0f;
  return adc_v * kBatDividerRatio;
}

float voltage_to_percent(float voltage) {
  if (voltage >= kLiIonFullV) {
    return 100.0f;
  }
  if (voltage <= kLiIonEmptyV) {
    return 0.0f;
  }
  return (voltage - kLiIonEmptyV) / (kLiIonFullV - kLiIonEmptyV) * 100.0f;
}

#endif

}  // namespace

void beeplan_sensors_init() {
#if defined(BEEPLAN_BOARD_T_ENERGY) && BEEPLAN_BOARD_T_ENERGY
  analogSetPinAttenuation(kBatAdcPin, ADC_11db);
  pinMode(kBatAdcPin, INPUT);
#endif
}

void beeplan_sensors_on_ack_rssi(int16_t rssi_dbm) {
  g_link_rssi_dbm = rssi_dbm;
}

void beeplan_sensors_read(int16_t& signal_dbm, float& battery_percent) {
#if defined(BEEPLAN_BOARD_T_ENERGY) && BEEPLAN_BOARD_T_ENERGY
  signal_dbm = g_link_rssi_dbm;
  battery_percent = voltage_to_percent(read_battery_voltage());
#else
  signal_dbm = static_cast<int16_t>(random(-88, -48));
  battery_percent = 88.0f;
  battery_percent += static_cast<float>(random(-15, 6)) / 10.0f;
  if (battery_percent < 5.0f) {
    battery_percent = 95.0f;
  }
  if (battery_percent > 100.0f) {
    battery_percent = 100.0f;
  }
#endif
}
