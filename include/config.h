#pragma once

/** MAC концентратора ESP-NOW (6 байт). Замените на свой. */
static const uint8_t GATEWAY_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define DEVICE_PUBLIC_ID "dev-edge-1"
#define WAKE_INTERVAL_SEC 60
#define GATEWAY_WIFI_CHANNEL 0
#define FIRMWARE_VERSION "0.0.0-dev"
#define FIRMWARE_SERIAL_TAG "beeplan-Edge-0.0.0-dev"
