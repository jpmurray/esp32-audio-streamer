# esp32-audio-streamer

An ESP32-based I2S microphone audio streamer with a browser UI and HTTP control API. It continuously captures audio from an I2S MEMS microphone, serves a raw PCM (or WAV) stream over HTTP, and optionally schedules deep sleep around civil dawn/dusk for low-power field deployments.

This has been tested on a Wemos D1 Mini32 board, and should thoerically work on other ESP32 boards as well.

---

## What it does

- Captures mono audio from an I2S microphone at a configurable sample rate (default 48 kHz, 32-bit input downshifted to PCM-16)
- Streams audio as chunked HTTP on port 81 (`audio/L16` or `audio/x-wav`)
- Serves a web UI and REST control API on port 80
- Provides first-boot Wi-Fi onboarding through a setup AP and `http://192.168.4.1/`
- Optionally deep-sleeps at night based on computed civil twilight (latitude/longitude + NTP time)
- Persists Wi-Fi credentials and runtime settings to NVS (non-volatile storage) so they survive reboots

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
| `NetworkManager` | NVS-backed Wi-Fi credentials, bounded STA reconnect, setup AP onboarding |
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

Wi-Fi credentials are normally configured at runtime. On first boot with no saved credentials, the firmware starts a setup AP named `ESP32-Audio-Setup-XXXXXX` (suffix derived from MAC by default); connect to it and open `http://192.168.4.1/`.

| Key | Default | Description |
|-----|---------|-------------|
| `WIFI_SSID` | `"YOUR_SSID"` | Optional legacy seed SSID. If non-placeholder and no NVS credentials exist, the firmware tries it once and persists it on success. |
| `WIFI_PASS` | `"YOUR_PASSWORD"` | Optional legacy seed password. Never displayed by the API/UI. |
| `WIFI_TX_POWER_DBM` | `15` | TX power in dBm (−1 … 20) |
| `WIFI_SETUP_AP_SSID` | `"ESP32-Audio-Setup"` | Base setup AP SSID |
| `WIFI_SETUP_AP_UNIQUE_SUFFIX` | `1` | Append a MAC-derived suffix to the AP SSID |
| `WIFI_SETUP_AP_PASS` | `""` | Setup AP password; empty means open, 8+ chars enables WPA2 |
| `WIFI_CONNECT_TIMEOUT_MS` | `20000` | Bounded STA connection attempt before AP fallback |

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

These settings can be changed through the web UI or `/api/set` without reflashing. They are stored in the `runtime` NVS namespace and restored at boot. Wi-Fi credentials are managed separately by `NetworkManager` in the `wifi` NVS namespace and are changed through the Wi-Fi setup/management UI or `/api/wifi/*` endpoints.

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

After changing `convert_shift` via the `/audio` settings panel, click **Restart Audio** (or POST `/api/action/restart-audio`) to stop and reinitialise the pipeline. The audio page shows both values if they differ.

---

## Web UI and API

### Web UI (`http://<device-ip>/`)

The normal UI is split into lightweight pages with a shared static navigation menu:

| Route | Contents |
|-------|----------|
| `/` | Dashboard: stream status/player, quick actions, compact Wi-Fi status, links to detail pages |
| `/wifi` | Full Wi-Fi management: status, credentials, scan, reconnect/forget, Wi-Fi TX power |
| `/audio` | Audio pipeline status/settings: HPF, convert_shift, restart audio, reset peak, stream/audio counters |
| `/system` | Time/scheduler/location, performance diagnostics, logs, time sync, reboot |

The dashboard and detail pages poll status every 5 s; logs poll every 15 s on `/system`.

### Wi-Fi onboarding

If the device has no saved credentials, or cannot connect with saved/seed credentials within the bounded timeout, it starts a setup AP. Connect to `ESP32-Audio-Setup-XXXXXX` (or your configured `WIFI_SETUP_AP_SSID`) and open:

```text
http://192.168.4.1/
```

The onboarding page can scan, save credentials, and shows the assigned STA IP after a successful connection. It is not a full DNS captive portal; open the address above explicitly. The setup AP is open by default unless `WIFI_SETUP_AP_PASS` is configured.

### REST API (`/api/*` on port 80)

All POST routes require the CSRF header: `X-ESP32MIC-CSRF: 1`

**GET endpoints**

| Route | Description |
|-------|-------------|
| `/api/status` | System status: mode, uptime, boot count, location, dawn/dusk, next event, stream URL, runtime settings, compact Wi-Fi status |
| `/api/audio_status` | Audio pipeline state: I2S health, sample rate, active convert_shift, HPF config, peak/clip/drop metrics |
| `/api/perf_status` | Heap, stack high-water marks, CPU MHz, Wi-Fi RSSI (`null` when STA is disconnected) |
| `/api/logs` | `{"logs": [...]}` — recent log lines from in-memory buffer |
| `/api/wifi_status` | Wi-Fi status: saved SSID, connected SSID, STA/AP IP, RSSI, setup AP state, last error. Never returns passwords. |
| `/api/wifi_scan` | Manual Wi-Fi scan returning SSID, RSSI, encryption flag, channel |

**POST endpoints**

| Route | Description |
|-------|-------------|
| `/api/set` | Update runtime settings (`wifi_tx_power_dbm`, `hpf_enabled`, `hpf_cutoff_hz`, `convert_shift`). Returns `{ok, restart_audio_required, settings}`. Persists to NVS. |
| `/api/wifi/config` | Save Wi-Fi credentials and schedule reconnect. Accepts `ssid`, `password_action=set|keep|clear`, and optional `password`. |
| `/api/wifi/reconnect` | Schedule reconnect using saved credentials |
| `/api/wifi/forget` | Erase saved credentials, start setup AP, optionally disconnect STA |
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
# Edit local_env.ini for pins, location, audio, etc.
# Wi-Fi SSID/PASS are optional legacy seed credentials, not required.

# 3. Build and upload
pio run --target upload

# 4. Monitor serial output (115200 baud)
pio device monitor

# 5. Configure Wi-Fi on first boot
# If no saved credentials exist, connect your phone/laptop to the setup AP:
#   ESP32-Audio-Setup-XXXXXX
# Then open:
#   http://192.168.4.1/
# Save credentials and note the assigned STA IP shown by the page/serial log.

# 6. Open the normal web UI
# Navigate to: http://<device-ip>/

# 7. Stream audio
# In VLC, ffplay, or any HTTP audio client:
# http://<device-ip>:81/stream
```

---

## Limitations / notes

- **One stream client at a time.** The stream task blocks until the current client disconnects before accepting a new one.
- **convert_shift requires audio restart.** Changing the shift value via the UI takes effect only after clicking "Restart Audio" — the active value shown in the Audio card is what the hardware currently uses.
- **STREAM_WAV_ENABLE is compile-time only.** Switching between raw PCM and WAV mid-stream is unsafe and is not a runtime setting.
- **LOG_LEVEL is compile-time only.** Runtime log-level changes are not currently supported.
- **NTP dependency.** If NTP fails at boot, deep sleep scheduling is deferred until time becomes valid. The device keeps retrying every 60 s. Deep sleep decisions are skipped while the device is in setup AP onboarding mode.
- **Setup AP is not a full captive portal.** Connect to the AP and explicitly open `http://192.168.4.1/`. The AP is open by default unless `WIFI_SETUP_AP_PASS` is configured.
- **Wi-Fi passwords are write-only.** Saved passwords are never shown in the UI, logs, or API responses.
- **Single I2S input.** Only one microphone / channel is captured; stereo mics must be configured for the correct channel via `USE_RIGHT_CHANNEL`.

---

## Acknowledgements

The recent control-plane and embedded Web UI improvements in this project were inspired in part by two BirdNET-Go community projects:

- [**birdnetgo-esp32-rtsp-mic**](https://github.com/Sukecz/birdnetgo-esp32-rtsp-mic)
- [**birdnetgo-m5stack-atom-echo-rtsp-mic**](https://github.com/stedrow/birdnetgo-m5stack-atom-echo-rtsp-mic)

In particular, they helped shape the direction of the cleaner runtime control surface, browser-based configuration flow, and device-status UI used in the latest iterations of this firmware. Everyone is encouraged to check on their works, they support different stack and boards.
