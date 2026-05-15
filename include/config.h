#pragma once

/** MAC концентратора ESP-NOW (6 байт). Замените на свой. */
static const uint8_t GATEWAY_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/** Публичный идентификатор устройства (как в BeePlan API). */
#define DEVICE_PUBLIC_ID "dev-edge-1"

/** Интервал между циклами измерения (секунды) для отладки без deep sleep. */
#define WAKE_INTERVAL_SEC 60
