# Web UI and API reference

The control server runs on port 80.

```text
http://<device-ip>/
```

## Web UI pages

| Route | Purpose |
|---|---|
| `/` | Dashboard: stream status, HTTP player, quick actions, Wi-Fi summary |
| `/wifi` | Wi-Fi status, scan, credentials, reconnect/forget, TX power |
| `/audio` | Audio status, audio profile, HPF, convert shift, stream counters |
| `/system` | Time, scheduler, heap/stack, logs, reboot/time sync |

## API basics

GET endpoints do not need special headers.

POST endpoints require:

```text
X-ESP32MIC-CSRF: 1
```

POST bodies are form-encoded.

Example:

```bash
curl -X POST \
  -H 'X-ESP32MIC-CSRF: 1' \
  -d 'audio_profile=stability_24k' \
  http://<device-ip>/api/set
```

## GET endpoints

| Route | Description |
|---|---|
| `/api/status` | System status, stream URLs, active transport, runtime settings, Wi-Fi summary |
| `/api/audio_status` | I2S/ring-buffer state, profile, sample rate, HPF, levels, drops, stream counters |
| `/api/perf_status` | Heap, CPU MHz, stack high-water marks, Wi-Fi sleep policy, RSSI |
| `/api/logs` | Recent in-memory logs |
| `/api/wifi_status` | Saved/connected SSID, IPs, RSSI, setup AP state, last error |
| `/api/wifi_scan` | Wi-Fi scan results |

## POST endpoints

| Route | Description |
|---|---|
| `/api/set` | Update runtime settings |
| `/api/wifi/config` | Save Wi-Fi credentials |
| `/api/wifi/reconnect` | Reconnect using saved credentials |
| `/api/wifi/forget` | Erase saved credentials and start setup AP |
| `/api/action/restart-audio` | Restart I2S/audio pipeline |
| `/api/action/reset-peak-hold` | Reset peak-hold level |
| `/api/action/time-sync` | Force NTP sync |
| `/api/action/reboot` | Reboot the ESP32 |

## `/api/set` settings

Accepted keys:

| Key | Values | Notes |
|---|---|---|
| `wifi_tx_power_dbm` | `-1` to `20` | Applied immediately |
| `hpf_enabled` | `0`, `1`, `true`, `false` | Applied immediately |
| `hpf_cutoff_hz` | `20` to `8000` | Applied immediately |
| `convert_shift` | `1` to `31` | Requires audio restart |
| `audio_profile` | `quality_48k`, `stability_24k`, `0`, `1` | Requires audio restart |

Example profile change:

```bash
curl -X POST \
  -H 'X-ESP32MIC-CSRF: 1' \
  -d 'audio_profile=stability_24k' \
  http://<device-ip>/api/set

curl -X POST \
  -H 'X-ESP32MIC-CSRF: 1' \
  http://<device-ip>/api/action/restart-audio
```

## Stream status fields

The dashboard uses `/api/status` stream fields:

| Field | Meaning |
|---|---|
| `active` | Any stream session is connected |
| `active_transport` | `none`, `http`, or `rtsp` |
| `rtsp_streaming` | RTSP session is actively playing |
| `connect_count` | Total stream sessions since boot |
| `http_connect_count` | HTTP stream sessions since boot |
| `rtsp_connect_count` | RTSP sessions since boot |
| `wav_url` | HTTP WAV stream URL |
| `pcm_url` | HTTP raw PCM stream URL |
| `rtsp_url` | RTSP stream URL |

`connect_count` is a session counter, not a current-client count.
