# Configuration reference

Most configuration starts in `local_env.ini`.

Create it from the example:

```bash
cp local_env.ini.example local_env.ini
```

Settings in `local_env.ini` become PlatformIO build flags. Runtime settings are stored in NVS and can be changed from the web UI.

## Common settings

### I2S pins

| Flag | Default | Description |
|---|---:|---|
| `PIN_I2S_WS` | 25 | WS / LRCLK |
| `PIN_I2S_SCK` | 33 | BCK / SCK |
| `PIN_I2S_SD` | 32 | SD / DOUT |
| `USE_RIGHT_CHANNEL` | 1 | `1` = right channel, `0` = left channel |
| `I2S_PORT_NUM` | 0 | ESP32 I2S peripheral |

### Audio

| Flag / setting | Default | Description |
|---|---:|---|
| `SAMPLE_RATE_HZ` | 48000 | Compile-time seed; runtime audio profile normally controls active rate |
| `CONVERT_SHIFT` | 11 | 32-bit to 16-bit right shift; lower is louder |
| `HPF_ENABLE` | 1 | Enable high-pass filter |
| `HPF_CUTOFF_HZ` | 100 | High-pass cutoff in Hz |
| `CHUNK_FRAMES` | 1024 | Producer chunk size |
| `RB_CAPACITY_BYTES` | 262144 | Preferred ring-buffer size; firmware falls back if allocation fails |
| `DMA_BUF_COUNT_CFG` | 4 | I2S DMA buffer count |

### I2S producer task placement

| Flag | Default | Description |
|---|---:|---|
| `I2S_TASK_STACK_WORDS` | 6144 | Stack size in words for the I2S producer FreeRTOS task |
| `I2S_TASK_PRIORITY` | 4 | FreeRTOS priority of the I2S producer task |
| `I2S_TASK_CORE` | 1 | CPU core to pin the I2S producer task to (0 or 1) |

The default places the producer on core 1, away from Wi-Fi/lwIP internals which run on core 0. Set `I2S_TASK_CORE=0` to restore the legacy behaviour if needed.

### Runtime audio profiles

| Profile | Sample rate | Use when |
|---|---:|---|
| `quality_48k` | 48 kHz | You want maximum detail and Wi-Fi is stable |
| `stability_24k` | 24 kHz | You want lower bandwidth and fewer stream issues |

The profile is changed from the **Audio** page or `/api/set`. Click **Restart Audio** after changing it.

### Wi-Fi

Wi-Fi credentials are normally configured from the setup UI.

Optional build flags:

| Flag | Default | Description |
|---|---:|---|
| `WIFI_TX_POWER_DBM` | 15 | Wi-Fi transmit power target |
| `WIFI_SETUP_AP_SSID` | `ESP32-Audio-Setup` | Setup AP name prefix |
| `WIFI_SETUP_AP_PASS` | empty | Setup AP password; empty means open |
| `WIFI_SETUP_AP_UNIQUE_SUFFIX` | 1 | Append MAC suffix to AP name |
| `WIFI_CONNECT_TIMEOUT_MS` | 20000 | STA connect timeout before setup AP fallback |

### RSSI quality thresholds

The firmware classifies the current STA RSSI into a quality tier that is exposed in
`/api/wifi_status`, health logs, and the web UI.

| Flag | Default | Description |
|---|---:|---|
| `WIFI_RSSI_WARN_DBM` | -70 | RSSI at or below this is classified as `weak` |
| `WIFI_RSSI_UNSTABLE_DBM` | -75 | RSSI at or below this is classified as `unstable` |
| `WIFI_RSSI_WARN_LOG_INTERVAL_MS` | 60000 | Minimum ms between repeated weak/unstable RSSI log warnings |

Classification:

| `rssi_quality` | Condition | Meaning |
|---|---|---|
| `good` | RSSI > `WIFI_RSSI_WARN_DBM` | Signal is acceptable for streaming |
| `weak` | RSSI ≤ `WIFI_RSSI_WARN_DBM` | Signal may cause occasional stream issues |
| `unstable` | RSSI ≤ `WIFI_RSSI_UNSTABLE_DBM` | Signal is likely to cause stream interruptions |
| `unknown` | STA disconnected | Quality cannot be determined |

A rate-limited warning is logged when the connection enters `weak` or `unstable` and again at most once per `WIFI_RSSI_WARN_LOG_INTERVAL_MS` while it stays degraded. The warning timer resets automatically when signal improves.

Wi-Fi passwords are write-only: they are not shown in the UI, logs, or API responses.

### Location and time

| Flag | Description |
|---|---|
| `LAT` | Latitude, positive north |
| `LON` | Longitude, negative west |
| `LOCAL_TZ` | POSIX timezone string |

Examples:

```ini
'-D LOCAL_TZ="UTC0"'
'-D LOCAL_TZ="EST5EDT,M3.2.0/2,M11.1.0/2"'
```

### Power and debug

| Flag | Default | Description |
|---|---:|---|
| `ENABLE_DEEP_SLEEP` | 1 | Set to `0` to stay awake all the time |
| `ENABLE_BROWNOUT_DISABLE` | 0 | Disable ESP32 brownout detector |
| `LOG_LEVEL` | 2 | `1` errors, `2` info/warn/error, `3` debug |

> **`ENABLE_BROWNOUT_DISABLE` default changed to `0`** (firmware v4+ / work item 4).
> Leaving the brownout detector enabled is the diagnostic-safe default: brownout resets
> are now visible in the serial log and reset-reason API (`ESP_RST_BROWNOUT`) rather
> than being silently suppressed.
> If your board has a marginal power supply and you need to work around spurious resets
> while investigating, set `-D ENABLE_BROWNOUT_DISABLE=1` temporarily. Fix the power
> supply root cause before deploying.

### Logging

`LOG_LEVEL` is compile-time only. It controls which severity levels are compiled into the firmware:

| Value | Levels compiled in |
|---:|---|
| `1` | errors |
| `2` | errors, warnings, info (default) |
| `3` | errors, warnings, info, debug |

#### Remote UDP/syslog logging

Optional. All flags are compile-time only. When enabled, logs are forwarded as best-effort RFC3164-style UDP datagrams in addition to Serial and the RAM ring. The remote sink has no effect on `/api/logs` or the web UI.

| Flag | Default | Description |
|---|---:|---|
| `ENABLE_REMOTE_LOG` | `0` | Set to `1` to enable the remote sink |
| `REMOTE_LOG_HOST` | `""` | **IPv4 literal** of the syslog/UDP receiver; DNS is never used |
| `REMOTE_LOG_PORT` | `514` | UDP port |
| `REMOTE_LOG_DEVICE` | `"esp32-audio-streamer"` | Device name included in each syslog message |
| `REMOTE_LOG_MIN_LEVEL` | `1` | Remote-only threshold (see table below) |
| `REMOTE_LOG_FAILURE_THRESHOLD` | `3` | Consecutive send failures before entering backoff suspension |
| `REMOTE_LOG_BACKOFF_MS` | `300000` | Backoff suspension duration in ms (default 5 min); suspension clears automatically |
| `REMOTE_LOG_MIN_RSSI_DBM` | `-75` | STA RSSI threshold below which remote sends are skipped (not counted as failures) |

`REMOTE_LOG_MIN_LEVEL` controls which already-compiled log levels are forwarded remotely:

| Value | Remote output |
|---:|---|
| `1` | errors only |
| `2` | errors, warnings |
| `3` | errors, warnings, info |
| `4` | errors, warnings, info, debug |

Limitations:

- `REMOTE_LOG_HOST` must be an IPv4 literal (e.g. `"192.168.1.10"`). DNS resolution is never attempted to avoid blocking the log path.
- UDP datagrams are best-effort: dropped silently when Wi-Fi is unavailable. No retries, no acknowledgement.
- After `REMOTE_LOG_FAILURE_THRESHOLD` consecutive failures the sink enters a timed backoff (`REMOTE_LOG_BACKOFF_MS`) and re-enables automatically.
- Sends are silently skipped when STA RSSI is below `REMOTE_LOG_MIN_RSSI_DBM`; these skips are not counted as failures.
- Remote log state (suspension status, counters) is visible in `/api/perf_status` under the `remote_log` object.
- The remote sink never calls `LOG*` macros internally to prevent recursion.

Example `local_env.ini` fragment:

```ini
-D ENABLE_REMOTE_LOG=1
'-D REMOTE_LOG_HOST="192.168.1.10"'
-D REMOTE_LOG_PORT=514
-D REMOTE_LOG_MIN_LEVEL=2
```

#### Periodic health logging

A compact health snapshot is logged at a configurable interval (uptime, heap, Wi-Fi/RSSI, stream transport, audio ring-buffer drop count, I2S errors).

| Flag | Default | Description |
|---|---:|---|
| `HEALTH_LOG_INTERVAL_MS` | `60000` | Interval in ms; set to `0` to disable |

Health logs appear in Serial, the RAM ring, and the remote sink (subject to `REMOTE_LOG_MIN_LEVEL`). They are info-level and compact.

#### Ring-buffer drop rate limiting

| Flag | Default | Description |
|---|---:|---|
| `RB_DROP_LOG_INTERVAL_MS` | `30000` | Minimum ms between ring-buffer drop warnings; set to `0` to log every drop |

## Deep sleep behavior

When deep sleep is enabled, the firmware:

1. Syncs time with NTP
2. Computes civil dawn/dusk from `LAT` and `LON`
3. Sleeps from dusk until next dawn
4. Stays awake during the day

For development, set:

```ini
-D ENABLE_DEEP_SLEEP=0
```

## Runtime settings

These are stored in NVS and survive reboot:

| Setting | Changed from UI? | Requires audio restart? |
|---|---:|---:|
| `wifi_tx_power_dbm` | yes | no |
| `hpf_enabled` | yes | no |
| `hpf_cutoff_hz` | yes | no |
| `convert_shift` | yes | yes |
| `audio_profile` | yes | yes |

If the Audio page shows different active/configured values, click **Restart Audio**.

## OTA firmware updates

### Partition table

The project ships `partitions/ota_4mb.csv`, a dual-slot OTA-capable layout for 4 MB flash boards (e.g. `wemos_d1_mini32`). It is wired into `platformio.ini` via `board_build.partitions`.

| Partition | Size | Purpose |
|---|---:|---|
| `nvs` | 20 KB | Non-volatile settings |
| `otadata` | 8 KB | Active-slot bookkeeping; initialized by PlatformIO at `0xe000` |
| `app0` | 1536 KB | Primary application slot; serial upload target at `0x10000` |
| `app1` | 1536 KB | Secondary application slot |
| `spiffs` | 960 KB | Data partition |

Slots were sized at 1536 KB (~1.7× the measured firmware size of ~886 KB) to leave room for growth.

### One-time serial migration

Devices shipped or flashed before the OTA partition table was introduced carry the default single-app layout. They must be serial-flashed **once** with the new partition table before browser OTA can work:

```bash
pio run -t upload
```

After that one-time flash, all subsequent firmware updates can be pushed over the browser.

### Upload artifact path

PlatformIO writes the binary to:

```
.pio/build/wemos_d1_mini32/firmware.bin
```

Upload this file through the **System** page in the web UI, or with `curl`:

```bash
curl -X POST \
  -H 'X-ESP32MIC-CSRF: 1' \
  -F 'firmware=@.pio/build/wemos_d1_mini32/firmware.bin' \
  http://<device-ip>/api/ota/upload
```

### Hardware validation flow

After the one-time serial flash:

1. **Happy path** — open the System page, select `firmware.bin`, click Upload. The progress bar fills, the device reboots, and the new build boots from the inactive slot.
2. **Interrupted upload** — if the browser tab is closed or power is lost mid-upload, the previous firmware remains active. The `Update.h` commit only swaps the boot slot at `Update.end()`, so a partial write leaves the running slot untouched.
3. **Night-mode / deep sleep** — OTA sets a maintenance inhibit flag on the scheduler before any flash write begins. Both the normal loop sleep path (`scheduler_trySleepIfNight`) and the early post-NTP sleep path respect this flag and skip sleep until the upload completes or fails.
4. **Active stream** — on upload start, `streamServer_requestStopAndWait()` is called with a 3 s timeout. If the stream does not drain, the upload is aborted before any flash write occurs and the previous firmware continues running.

## Limits and notes

- `STREAM_WAV_ENABLE` only controls `/stream`; `/stream.wav` and `/stream.pcm` always exist.
- `RTSP_PORT=0` disables RTSP at build time.
- `RTSP_MAX_SAMPLE_RATE_HZ` defaults to `24000`; RTSP `DESCRIBE` is rejected above this rate because field testing showed 48 kHz L16/RTSP can fail to reach `PLAY` or exhaust TCP buffers on ESP32, while `stability_24k` works reliably. Set it to `0` to disable this guard.
- `AUDIO_PROFILE_DEFAULT` controls first boot / invalid-NVS fallback only. For BirdNET-Go RTSP deployments, `AUDIO_PROFILE_STABILITY_24K` is recommended.
- Serial messages like `_parseRequest(): Invalid request: \x16\x03\x01...` are TLS ClientHello bytes hitting the plain HTTP server, usually from an `https://` URL or browser HTTPS upgrade. Use `http://<device-ip>/...`; the firmware does not serve HTTPS.
- `LOG_LEVEL` and all remote logging flags are compile-time only; they cannot be changed at runtime.
- `REMOTE_LOG_HOST` must be an IPv4 literal; DNS is never used.
- Remote logging is best-effort UDP only — no retries, no delivery guarantee.
- Setup AP is not a full captive portal; open `http://192.168.4.1/` manually.
- Only one I2S channel is captured.
