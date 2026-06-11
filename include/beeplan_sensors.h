#pragma once

#include <stdint.h>

/** RSSI последнего ACK gateway (dBm), обновляется в ESP-NOW recv. */
extern int16_t g_link_rssi_dbm;

void beeplan_sensors_init();
void beeplan_sensors_on_ack_rssi(int16_t rssi_dbm);
void beeplan_sensors_read(int16_t& signal_dbm, float& battery_volts);
