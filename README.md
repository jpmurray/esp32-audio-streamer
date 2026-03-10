# ESP32 Audio Streamer

Small ESP32 (Arduino) project that streams mono 16‑bit PCM from an I2S microphone over HTTP.

## HTTP API

- `GET /`
  - Simple index page with an HTML5 audio player pointing to `/stream`.
- `GET /stream`
  - Audio stream of mono 16‑bit PCM at `SAMPLE_RATE_HZ`.
  - Content type depends on build flag `STREAM_WAV_ENABLE`:
    - `STREAM_WAV_ENABLE=0` → `audio/L16; rate=<Hz>; channels=1` (raw PCM)
    - `STREAM_WAV_ENABLE=1` → `audio/x-wav` (WAV header followed by PCM)
- `GET /uptime`
  - Returns uptime information only as JSON.
  - Fields: `uptime`, `uptime_human`, `days`, `hours`, `minutes`, `seconds`
- `GET /status`
  - Returns device status and schedule as JSON (non-uptime fields moved from `/uptime`).
  - Includes both UTC and local timezone fields (local TZ set via `LOCAL_TZ`).
  - Fields:
    - Device/location: `boot_count`, `location` { `lat`, `lon` }, `schedule_basis`
    - Time: `now_utc`, `last_ntp_check_utc`; local variants: `now_local`, `last_ntp_check_local`
    - Today/tomorrow: `today` { `civil_dawn_utc`, `civil_dusk_utc` }, `today_local` { `civil_dawn_local`, `civil_dusk_local` }, and same for `tomorrow`/`tomorrow_local`
    - Mode/event: `mode` (day|night), `next_event` { `type`, `at_utc`, `seconds_until` }, plus `next_event_local` { `type`, `at_local` }
    - History: `last_three_wakes_utc`, `next_three_sleeps_utc`, and local arrays `last_three_wakes_local`, `next_three_sleeps_local`
    - Timezone info: `timezone` { `posix` }

## Power Scheduling

- Awake from civil dawn → civil dusk; deep sleep at night (civil dusk → next dawn).
- Computes dawn/dusk daily using a lightweight NOAA/Meeus method (sun altitude −6°), UTC only.
- Syncs NTP at boot and at least every 24h while awake. If time is invalid, retries every 60s and defers sleep decisions until valid.
- Before deep sleep, shuts down I2S producer, ring buffer, I2S driver and Wi‑Fi.

## Persistence

- RTC (retained across deep sleep): boot counter, cached dawn/dusk (today/tomorrow), last compute day (UTC YMD), last NTP sync time.
- NVS (Preferences namespace `sched`):
  - `last_wakes` (CSV of up to 3 epoch seconds) → exposed as `last_three_wakes_utc`
  - `next_three_sleeps` (CSV of up to 3 epoch seconds) → exposed as `next_three_sleeps_utc`

## Build Flags (PlatformIO)

Add these in `local_env.ini` via `build_flags` (see `local_env.ini.example`). Defaults shown in parentheses.

- Wi‑Fi
  - `-D WIFI_SSID="..."` — Wi‑Fi SSID (required; default placeholder `YOUR_SSID`).
  - `-D WIFI_PASS="..."` — Wi‑Fi password (required; default placeholder `YOUR_PASSWORD`).
  - `-D WIFI_TX_POWER_DBM=15` — target TX power in dBm (default 15). Mapped to closest supported step: −1, 2, 5, 7, 8.5, 11, 13, 15, 17, 18.5, 19, 19.5.
- Audio
  - `-D SAMPLE_RATE_HZ=48000` — sample rate in Hz (default 48000).
  - `-D CONVERT_SHIFT=11` — right shift converting 32‑bit mic to 16‑bit PCM; lower is louder, watch clipping (default 11).
  - `-D HPF_ENABLE=1` — enable high‑pass/DC‑block filter (default 1=on).
  - `-D HPF_CUTOFF_HZ=100` — HPF cutoff in Hz (default 100).
  - `-D USE_RIGHT_CHANNEL=1` — 1: use RIGHT channel; 0: LEFT (default 1).
- I2S pins
  - `-D PIN_I2S_WS=25` — WS/LRCLK pin (default 25).
  - `-D PIN_I2S_SCK=33` — BCLK/SCK pin (default 33).
  - `-D PIN_I2S_SD=32` — SD/DOUT pin (default 32).
  - `-D I2S_PORT_NUM=0` — I2S port index (default 0).
- Stream format
  - `-D STREAM_WAV_ENABLE=0` — 0: raw `audio/L16`, 1: `audio/x-wav` header (default 0).
- Location
  - `-D LAT=51.4630911` — latitude in degrees (positive north). If omitted, defaults to 51.4630911.
  - `-D LON=-3.1678763` — longitude in degrees (negative west). If omitted, defaults to -3.1678763.
- Buffering
  - `-D CHUNK_FRAMES=1024` — producer chunk size in frames (default 1024).
  - `-D RB_CAPACITY_BYTES=65536` — ring buffer size in bytes (default 65536).
  - `-D DMA_BUF_COUNT_CFG=4` — I2S DMA buffer count (default 4).
- Server & logging
  - `-D SERVER_PORT=80` — HTTP port (default 80).
  - `-D LOG_LEVEL=2` — 1: errors, 2: info/warn/error, 3: debug (default 2).
- Local timezone
  - `-D LOCAL_TZ="UTC0"` — POSIX TZ string for local time (default `UTC0`). Examples: `EST5EDT,M3.2.0/2,M11.1.0/2`, `CET-1CEST,M3.5.0/2,M10.5.0/3`.
- Power / stability
  - `-D ENABLE_POWER_SCHEDULING=1` - 1: put the board into deep sleep at night, 0: run continuously (default 1).
  - `-D ENABLE_BROWNOUT_DISABLE=1` — 1: disable brownout detector (workaround), 0: leave enabled (default 1).

## Notes
- Set `WIFI_SSID`/`WIFI_PASS` in `local_env.ini`. The project logs a warning if placeholders are used.
- The stream is mono 16‑bit PCM. WAV mode adds a streaming header with unknown size fields.
