# ESP32 Audio Streamer (Arduino + PlatformIO)

Streams live audio from an I²S MEMS microphone (INMP44/INMP441–style) on an ESP32 over HTTP as raw 16‑bit PCM. Built with the Arduino framework on PlatformIO for the `wemos_d1_mini32` board.

- Wi‑Fi STA with blocking connect and diagnostics
- I²S mic capture at 48 kHz, mono, 32‑bit in → 16‑bit out
- Pipeline: I²S producer task → ring buffer → optional HPF → HTTP streamer
- Endpoint `/stream`: continuous audio as raw PCM (`audio/L16; rate=48000; channels=1`) or WAV (`audio/x-wav`) when enabled via build flag
- Simple index page at `/` with a basic player

## Hardware

- Board: ESP32 (WEMOS D1 MINI ESP32)
- Microphone: INMP44/INMP441‑style I²S MEMS (mono)
- Pin mapping (from user constraints, configurable via `audio_settings.ini`):
  - `WS / LRCLK` → `GPIO25`  (`-D PIN_I2S_WS=25`)
  - `BCLK / SCK` → `GPIO33`  (`-D PIN_I2S_SCK=33`)
  - `SD / DOUT (mic → ESP32)` → `GPIO32`  (`-D PIN_I2S_SD=32`)
- Channel selection: mic L/R strap determines which channel carries audio. Default is RIGHT. Set with `-D USE_RIGHT_CHANNEL=1` or `0`.

Power tip: USB ports/cables with weak 5V can cause brownouts when Wi‑Fi transmits. This firmware disables the brownout detector and reduces stalls; for reliability, use a good cable/port or a powered hub.

## Project Layout

- `src/main.cpp` — firmware (Wi‑Fi, I²S init, ring buffer, HPF, HTTP server)
- `platformio.ini` — board config and external config includes
- `wifi_secrets.ini.example` — example secrets config
- `audio_settings.ini.example` — example audio/tuning config

## Configure

This project uses a single external, git‑ignored config file so you don’t commit secrets or personal tuning.

- Copy example and edit:
  ```bash
  cp local_env.ini.example local_env.ini
  ```
- Put your Wi‑Fi credentials and any tuning flags in that file. Example contents:
  ```ini
  [env:wemos_d1_mini32]
  build_flags =
    '-D WIFI_SSID="Your SSID"'
    '-D WIFI_PASS="Your Password"'
    -D SAMPLE_RATE_HZ=48000
    -D CONVERT_SHIFT=11
    -D HPF_ENABLE=1
    -D HPF_CUTOFF_HZ=100
    -D CHUNK_FRAMES=1024
    -D RB_CAPACITY_BYTES=65536
    -D USE_RIGHT_CHANNEL=1
    -D PIN_I2S_WS=25
    -D PIN_I2S_SCK=33
    -D PIN_I2S_SD=32
    -D I2S_PORT_NUM=0
    -D DMA_BUF_COUNT_CFG=4
    -D SERVER_PORT=80
    ; Optional: stream as WAV instead of raw PCM
    ; -D STREAM_WAV_ENABLE=1
  ```

Notes
- PlatformIO loads this file via `extra_configs` in `platformio.ini`:
  ```ini
  [platformio]
  extra_configs = local_env.ini
  ```
- `.gitignore` excludes `local_env.ini`.

## Build and Flash

- Build + Upload:
  ```bash
  pio run -t upload
  ```
- Serial Monitor (115200):
  ```bash
  pio device monitor
  ```
- On boot you should see the IP address, I²S init OK, and ring buffer creation.

## Use

- Index page: `http://<esp-ip>/` (basic HTML page)
- Stream: `http://<esp-ip>/stream` — raw 16‑bit PCM (default) or WAV with header, mono, 48 kHz

Players
- sox (quick check):
  ```bash
  # Raw PCM (default)
  curl http://<esp-ip>/stream | play -t s16 -r 48000 -c 1 -
  # If WAV header is enabled
  curl http://<esp-ip>/stream | play -t wav -
  ```
- VLC:
  ```bash
  vlc --demux=rawaud --rawaud-fourcc=s16l \
      --rawaud-channels=1 --rawaud-samplerate=48000 \
      --network-caching=200 http://<esp-ip>/stream
  ```
- mpv / IINA:
  ```bash
  mpv --demuxer=rawaudio --demuxer-rawaudio-format=s16le \
      --demuxer-rawaudio-channels=1 --demuxer-rawaudio-samplerate=48000 \
      http://<esp-ip>/stream
  ```

## How It Works

- I²S configured as master RX, 32‑bit samples. Mic output is 24‑bit in a 32‑bit frame.
- Producer task reads I²S into a local buffer, converts to 16‑bit with a fixed shift (no AGC), and pushes into a FreeRTOS ring buffer.
- HTTP handler pulls int16 chunks from the ring buffer, optionally applies a 1st‑order high‑pass filter (DC blocker), and writes to the client.
- Designed for one client; the handler blocks while streaming.

## Tuning

Build‑time flags (set in `audio_settings.ini`):
- `SAMPLE_RATE_HZ` — default `48000`. You can try `32000` if needed.
- `CONVERT_SHIFT` — scale from 32‑bit to 16‑bit. Lower value = louder; typical 10–11. Avoid clipping.
- `HPF_ENABLE` / `HPF_CUTOFF_HZ` — 1st‑order DC blocker; 80–120 Hz is a common range.
- `CHUNK_FRAMES` — producer chunk size in frames. `1024` aligns with DMA; `2048` reduces overhead with ~21 ms extra latency at 48 kHz.
- `RB_CAPACITY_BYTES` — total ring buffer size. `65536` (64 KB) is a good default; increase for more headroom.
- `USE_RIGHT_CHANNEL` — choose RIGHT(1) or LEFT(0) depending on mic strap.

## Troubleshooting

- Brownout / resets
  - Use a better USB cable/port or powered hub. Firmware already disables the brownout detector and manages TX power.
- Silence on stream
  - Switch channel with `-D USE_RIGHT_CHANNEL=0/1`. Verify pin mapping and mic power/ground.
- Choppy audio
  - Keep HPF enabled but ensure the ring buffer is large (`RB_CAPACITY_BYTES>=65536`).
  - Use `CHUNK_FRAMES=1024` to align with DMA for lower jitter.
  - Network hiccups over Wi‑Fi can cause gaps; closer AP helps.
- I²S driver install error about DMA length
  - `DMA_BUF_LEN` is fixed to `1024` due to ESP‑IDF limits.

## Notes / Limits

- Single client supported (blocking handler).
- Stream is raw PCM; most browsers won’t play `audio/L16` directly, but tools like sox/VLC/mpv handle it.
- No authentication; for LAN/testing use.
