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
| `ENABLE_BROWNOUT_DISABLE` | 1 | Disable ESP32 brownout detector |
| `LOG_LEVEL` | 2 | `1` errors, `2` info/warn/error, `3` debug |

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
- `LOG_LEVEL` is compile-time only.
- Setup AP is not a full captive portal; open `http://192.168.4.1/` manually.
- Only one I2S channel is captured.
