#pragma once

#include "envelope_v2.h"

/** Инициализация HX711 / DS18B20, загрузка калибровки из NVS. */
void beeplan_scales_init();

/** Отключить HX711 перед deep sleep (экономия ~1.5 mA). */
void beeplan_scales_power_down();

/**
 * Собрать ReportFrameV2 для пасечных весов.
 * Вес кодируется в audio_rms_x1000 (кг×100), см. RADIO_PROTOCOL / gateway decode.
 */
ReportFrameV2 beeplan_scales_build_report(uint32_t report_seq, bool include_firmware);
