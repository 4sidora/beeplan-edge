# beeplan-edge

**Сборка платы, пины датчиков и связь с концентратором:** [HARDWARE.md](https://github.com/4sidora/beeplan-docs/blob/main/HARDWARE.md) в репозитории **beeplan-docs** (обновляйте при изменениях прошивки).

Прошивка **BeePlan** для ESP32 в улье: опрос датчиков, короткий захват звука, вычисление компактных признаков, отправка на концентратор по **ESP‑NOW**, затем deep sleep.

**Рекомендуемая установка:** веб-мастер в [beeplan-web](https://github.com/4sidora/beeplan-web) (`/install/edge`). Конфигурация генерируется сервисом **beeplan-builder** из `include/config.h.in`.

## Сборка (разработчикам)

1. Скопируйте `include/config.h.example` → `include/config.h`.
2. Укажите `GATEWAY_MAC` и `DEVICE_PUBLIC_ID`.
3. `pio run -t upload`

## Протокол ESP‑NOW (черновик v1)

Бинарный кадр (little-endian), поле `magic = 0xBEEF01`:

| Поле | Размер |
|------|--------|
| magic | u32 |
| proto_version | u8 |
| device_public_id | 32 байта, zero-padded ASCII |
| unix_ts_utc | u32 |
| metric | u8 enum (0=temp, 1=rh, 2=audio) |
| payload | см. реализацию |

Концентратор транслирует это в JSON для `POST /v1/telemetry/batch`.

## Лицензия

MIT (добавьте файл LICENSE при публикации).
