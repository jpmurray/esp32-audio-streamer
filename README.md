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
  - Returns device uptime as JSON.
  - Example:
    ```json
    {
      "uptime": 93784,
      "uptime_human": "1d 2h 3m 4s",
      "days": 1,
      "hours": 2,
      "minutes": 3,
      "seconds": 4
    }
    ```

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
- Buffering
  - `-D CHUNK_FRAMES=1024` — producer chunk size in frames (default 1024).
  - `-D RB_CAPACITY_BYTES=65536` — ring buffer size in bytes (default 65536).
  - `-D DMA_BUF_COUNT_CFG=4` — I2S DMA buffer count (default 4).
- Server & logging
  - `-D SERVER_PORT=80` — HTTP port (default 80).
  - `-D LOG_LEVEL=2` — 1: errors, 2: info/warn/error, 3: debug (default 2).
- Power / stability
  - `-D ENABLE_BROWNOUT_DISABLE=1` — 1: disable brownout detector (workaround), 0: leave enabled (default 1).

## Notes
- Set `WIFI_SSID`/`WIFI_PASS` in `local_env.ini`. The project logs a warning if placeholders are used.
- The stream is mono 16‑bit PCM. WAV mode adds a streaming header with unknown size fields.
