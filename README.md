# esp32-audio-streamer

An ESP32-based I2S microphone audio streamer with a browser UI and HTTP control API. It continuously captures audio from an I2S MEMS microphone, serves a raw PCM (or WAV) stream over HTTP, and optionally schedules deep sleep around civil dawn/dusk for low-power field deployments.

This has been tested on a Wemos D1 Mini32 board, and should thoerically work on other ESP32 boards as well.

---

## What it does

- Captures mono audio from an I2S microphone at a configurable sample rate (default 48 kHz, 32-bit input downshifted to PCM-16)
- Streams audio as chunked HTTP on port 81 (`audio/L16` or `audio/x-wav`)
- Serves a web UI and REST control API on port 80
- Optionally deep-sleeps at night based on computed civil twilight (latitude/longitude + NTP time)
- Persists a small set of runtime settings to NVS (non-volatile storage) so they survive reboots

---

## Hardware

Default pin mapping (override via `local_env.ini`):

| Signal | GPIO |
|--------|------|
| WS / LRCLK | 25 |
| BCK / SCK  | 33 |
| SD / DOUT  | 32 |

Channel selection defaults to RIGHT (`USE_RIGHT_CHANNEL=1`). Brownout detection is disabled by default (`ENABLE_BROWNOUT_DISABLE=1`).

---

## Architecture

```
┌─────────────┐   I2S DMA    ┌──────────────────┐  ring buffer  ┌─────────────┐
│  I2S mic    │ ──────────▶  │  AudioPipeline   │ ────────────▶ │ StreamServer│ ──▶ HTTP :81/stream
│  (32-bit)   │              │  (producer task) │               │ (stream task│
└─────────────┘              └──────────────────┘               └─────────────┘
                                     │ HPF applied at read time (StreamServer)
                                     ▼
                              RuntimeSettings (NVS)
                              AppState / Scheduler
                                     │
                              ┌──────┴──────┐
                              │ HttpControl │ ──▶ HTTP :80  (UI + /api/*)
                              └─────────────┘
```

**Key modules:**

| Module | Responsibility |
|--------|---------------|
| `AudioPipeline` | I2S init, DMA capture, 32→16-bit shift, ring buffer writes |
| `StreamServer` | Dedicated FreeRTOS task, HTTP chunked stream on port 81, HPF application |
| `HttpControl` | Control-plane routes on port 80, CSRF protection |
| `RuntimeSettings` | NVS-backed settings struct, hot-apply where safe |
| `AppState` / `Scheduler` | Dawn/dusk computation, NTP, deep sleep scheduling |
| `LogBuffer` | In-memory circular log, exposed via `/api/logs` |

---

## Ports

| Port | Purpose |
|------|---------|
| **80** | Web UI (`/`) and REST API (`/api/*`) |
| **81** | Audio stream (`/stream`) |

To play the stream:
```
http://<device-ip>:81/stream
```
The web UI (port 80) embeds an `<audio>` player pointed at this URL automatically.

---

## Compile-time configuration (`local_env.ini`)

Copy `local_env.ini.example` to `local_env.ini` and set values before building. All settings become C preprocessor defines via PlatformIO's `build_flags`.

**Wi-Fi**

| Key | Default | Description |
|-----|---------|-------------|
| `WIFI_SSID` | `"Your SSID"` | Network name |
| `WIFI_PASS` | `"Your Password"` | Network password |
| `WIFI_TX_POWER_DBM` | `15` | TX power in dBm (−1 … 19.5) |

**Audio**

| Key | Default | Description |
|-----|---------|-------------|
| `SAMPLE_RATE_HZ` | `48000` | I2S sample rate |
| `CONVERT_SHIFT` | `11` | 32→16-bit right-shift; lower = louder |
| `HPF_ENABLE` | `1` | Enable high-pass filter |
| `HPF_CUTOFF_HZ` | `100` | HPF cutoff frequency (Hz) |

**Buffering / I2S**

| Key | Default | Description |
|-----|---------|-------------|
| `CHUNK_FRAMES` | `1024` | Producer chunk size in frames |
| `RB_CAPACITY_BYTES` | `65536` | Ring buffer size (bytes) |
| `DMA_BUF_COUNT_CFG` | `4` | I2S DMA buffer count |
| `USE_RIGHT_CHANNEL` | `1` | 1 = RIGHT channel, 0 = LEFT |
| `PIN_I2S_WS` | `25` | WS / LRCLK pin |
| `PIN_I2S_SCK` | `33` | BCK / SCK pin |
| `PIN_I2S_SD` | `32` | SD / DOUT pin |
| `I2S_PORT_NUM` | `0` | I2S peripheral index |

**Location & time**

| Key | Default | Description |
|-----|---------|-------------|
| `LAT` | `51.4630911` | Latitude (positive = north) |
| `LON` | `-3.1678763` | Longitude (negative = west) |
| `LOCAL_TZ` | `"UTC0"` | POSIX TZ string |

**Server**

| Key | Default | Description |
|-----|---------|-------------|
| `SERVER_PORT` | `80` | Control-plane HTTP port |
| `STREAM_WAV_ENABLE` | `0` | 0 = raw PCM L16, 1 = WAV header |

**Power / debug**

| Key | Default | Description |
|-----|---------|-------------|
| `ENABLE_BROWNOUT_DISABLE` | `1` | 1 = disable brownout detector |
| `ENABLE_DEEP_SLEEP` | `1` | 0 = disable deep sleep (stay awake at night; useful for debugging) |
| `LOG_LEVEL` | `2` | 1 = errors only, 2 = info/warn/error, 3 = debug |

---

## Runtime settings (persisted in NVS)

These settings can be changed through the web UI or `/api/set` without reflashing. They are stored in the `runtime` NVS namespace and restored at boot.

| Setting | Range | Hot-apply? | Notes |
|---------|-------|-----------|-------|
| `wifi_tx_power_dbm` | −1 … 20 | Yes | Applied immediately |
| `hpf_enabled` | bool | Yes | Applied immediately |
| `hpf_cutoff_hz` | 20 … 8000 | Yes | Applied immediately |
| `convert_shift` | 1 … 31 | **No** | Requires audio restart (see below) |

### `convert_shift` — configured vs active

`convert_shift` has two values at runtime:

- **Configured value** — what is stored in NVS and editable via `/api/set`. Survives reboots.
- **Active value** — captured once when the audio pipeline initialises (`audioPipeline_init()`). The I2S producer task uses this fixed snapshot.

After changing `convert_shift` via the Settings panel, click **Restart Audio** (or POST `/api/action/restart-audio`) to stop and reinitialise the pipeline. The new configured value becomes the active value. The web UI "Audio" card always shows the active value and the Settings panel notes "(restart req.)" next to the field.

---

## Web UI and API

### Web UI (`http://<device-ip>/`)

The single-page UI polls the device every 5 s (logs every 15 s) and shows:

| Card | Contents |
|------|----------|
| **Stream** | Status badge, connection count, stream URL with copy button, embedded audio player |
| **Status** | Day/night mode, uptime, boot count, civil dawn/dusk times, next scheduled event |
| **Audio** | I2S health, sample rate, WAV/PCM mode, HPF config, active convert_shift, peak level, peak hold, clip count, error/drop counts |
| **Performance** | Heap free/min, CPU MHz, Wi-Fi RSSI |
| **Settings** | Wi-Fi TX power, HPF enabled, HPF cutoff, convert shift — "Apply" POSTs to `/api/set` |
| **Actions** | Restart Audio, Time Sync, Reboot |
| **Logs** | Colour-coded live log buffer (auto-scroll) |

### REST API (`/api/*` on port 80)

All POST routes require the CSRF header: `X-ESP32MIC-CSRF: 1`

**GET endpoints**

| Route | Description |
|-------|-------------|
| `/api/status` | System status: mode, uptime, boot count, location, dawn/dusk, next event, stream URL, runtime settings |
| `/api/audio_status` | Audio pipeline state: I2S health, sample rate, active convert_shift, HPF config, peak/clip/drop metrics |
| `/api/perf_status` | Heap, stack high-water marks, CPU MHz, Wi-Fi RSSI |
| `/api/logs` | `{"logs": [...]}` — recent log lines from in-memory buffer |

**POST endpoints**

| Route | Description |
|-------|-------------|
| `/api/set` | Update runtime settings (`wifi_tx_power_dbm`, `hpf_enabled`, `hpf_cutoff_hz`, `convert_shift`). Returns `{ok, restart_audio_required, settings}`. Persists to NVS. |
| `/api/action/restart-audio` | Stop and reinitialise audio pipeline (required after `convert_shift` change) |
| `/api/action/reset-peak-hold` | Reset the peak-hold accumulator |
| `/api/action/time-sync` | Force NTP synchronisation |
| `/api/action/reboot` | Reboot device (`ESP.restart()`) |

---

## Deep sleep scheduling

When `ENABLE_DEEP_SLEEP=1` (default), the device:

1. Syncs time via NTP at boot (retries every 60 s until valid)
2. Computes civil dawn and dusk for the configured `LAT`/`LON` (−6° twilight angle)
3. Sleeps from dusk until the next dawn; wakes, re-syncs NTP, and repeats
4. Caches schedule in RTC memory and NVS so it survives cold boots

Set `ENABLE_DEEP_SLEEP=0` in `local_env.ini` to keep the device awake around the clock. This is useful during development or in always-on deployments.

---

## Build and usage

### Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- ESP32 board (tested on Wemos D1 Mini32 / equivalent)

### Steps

```bash
# 1. Clone and enter the project
git clone <repo-url>
cd esp32-audio-streamer

# 2. Create your local config
cp local_env.ini.example local_env.ini
# Edit local_env.ini — set WIFI_SSID, WIFI_PASS, pins, location, etc.

# 3. Build and upload
pio run --target upload

# 4. Monitor serial output (115200 baud)
pio device monitor

# 5. Open the web UI
# The device prints its IP on the serial console at boot.
# Navigate to: http://<device-ip>/

# 6. Stream audio
# In VLC, ffplay, or any HTTP audio client:
# http://<device-ip>:81/stream
```

---

## Limitations / notes

- **One stream client at a time.** The stream task blocks until the current client disconnects before accepting a new one.
- **convert_shift requires audio restart.** Changing the shift value via the UI takes effect only after clicking "Restart Audio" — the active value shown in the Audio card is what the hardware currently uses.
- **STREAM_WAV_ENABLE is compile-time only.** Switching between raw PCM and WAV mid-stream is unsafe and is not a runtime setting.
- **LOG_LEVEL is compile-time only.** Runtime log-level changes are not currently supported.
- **NTP dependency.** If NTP fails at boot, deep sleep scheduling is deferred until time becomes valid. The device keeps retrying every 60 s.
- **Single I2S input.** Only one microphone / channel is captured; stereo mics must be configured for the correct channel via `USE_RIGHT_CHANNEL`.

---

## Acknowledgements

The recent control-plane and embedded Web UI improvements in this project were inspired in part by two BirdNET-Go community projects:

- [**birdnetgo-esp32-rtsp-mic**](https://github.com/Sukecz/birdnetgo-esp32-rtsp-mic)
- [**birdnetgo-m5stack-atom-echo-rtsp-mic**](https://github.com/stedrow/birdnetgo-m5stack-atom-echo-rtsp-mic)

In particular, they helped shape the direction of the cleaner runtime control surface, browser-based configuration flow, and device-status UI used in the latest iterations of this firmware. Everyone is encouraged to check on their works, they support different stack and boards.
