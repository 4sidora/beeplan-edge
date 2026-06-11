#include "beeplan_sensors.h"

#include <Arduino.h>

int16_t g_link_rssi_dbm = -127;
/** RSSI последнего ACK — сохраняется между deep sleep. */
RTC_DATA_ATTR int16_t g_last_ack_rssi_dbm = -127;

namespace {

#if defined(BEEPLAN_BOARD_T_ENERGY) && BEEPLAN_BOARD_T_ENERGY

constexpr int kBatAdcPin = 35;
constexpr int kBatSampleCount = 16;
/** Делитель 100k/100k на T-Energy T18 V3.0. */
constexpr float kBatDividerRatio = 2.0f;

float read_battery_voltage() {
  uint32_t sum_mv = 0;
  for (int i = 0; i < kBatSampleCount; ++i) {
    sum_mv += static_cast<uint32_t>(analogReadMilliVolts(kBatAdcPin));
    delay(2);
  }
  const float adc_v = static_cast<float>(sum_mv) / static_cast<float>(kBatSampleCount) / 1000.0f;
  return adc_v * kBatDividerRatio;
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
  g_last_ack_rssi_dbm = rssi_dbm;
}

void beeplan_sensors_read(int16_t& signal_dbm, float& battery_volts) {
#if defined(BEEPLAN_BOARD_T_ENERGY) && BEEPLAN_BOARD_T_ENERGY
  signal_dbm = g_last_ack_rssi_dbm;
  battery_volts = read_battery_voltage();
#else
  signal_dbm = static_cast<int16_t>(random(-88, -48));
  battery_volts = 3.9f;
  battery_volts += static_cast<float>(random(-15, 6)) / 100.0f;
  if (battery_volts < 3.5f) {
    battery_volts = 4.05f;
  }
  if (battery_volts > 4.2f) {
    battery_volts = 4.2f;
  }
#endif
}
