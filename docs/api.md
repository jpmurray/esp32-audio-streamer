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
| `/api/ota/status` | OTA phase, progress, free slot size, last error |

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
| `/api/ota/upload` | Upload a `firmware.bin` for OTA flash; device reboots on success |
| `/api/ota/abort` | Abort an upload in progress |

## `/api/status` build identifier

`/api/status` includes a `build` object that can be used to confirm which firmware booted after OTA:

```json
{
  "build": {
    "id": "May 12 2026 14:03:21",
    "date": "May 12 2026",
    "time": "14:03:21"
  }
}
```

By default this is the compile date/time. It can be overridden at build time with a `BUILD_ID` macro if needed.

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

## OTA firmware update endpoints

### `GET /api/ota/status`

Returns the current OTA state. No CSRF header required.

```bash
curl http://<device-ip>/api/ota/status
```

Response:

```json
{
  "supported": true,
  "phase": "idle",
  "progress": 0,
  "free_ota_space": 1507328,
  "maintenance": false,
  "reboot_pending": false,
  "last_error": ""
}
```

| Field | Type | Description |
|---|---|---|
| `supported` | bool | Always `true` when this endpoint exists |
| `phase` | string | `idle`, `receiving`, `success_reboot_pending`, or `failed` |
| `progress` | int | Upload progress 0–100; only meaningful while `phase` is `receiving` |
| `free_ota_space` | uint | Bytes available in the inactive OTA slot; `0` if no OTA partition is present |
| `maintenance` | bool | `true` while receiving or reboot is pending; deep sleep is inhibited |
| `reboot_pending` | bool | `true` after a successful upload, until the device reboots |
| `last_error` | string | Human-readable error from the most recent failed attempt; empty when none |

---

### `POST /api/ota/upload`

Uploads a firmware binary. Requires `X-ESP32MIC-CSRF: 1`. Send the file as a `multipart/form-data` body (standard browser `<input type="file">` or `curl -F`).

```bash
curl -X POST \
  -H 'X-ESP32MIC-CSRF: 1' \
  -F 'firmware=@.pio/build/wemos_d1_mini32/firmware.bin' \
  http://<device-ip>/api/ota/upload
```

Success response (`200`):

```json
{"ok": true, "message": "Update complete; rebooting"}
```

Error responses:

| HTTP status | `error` value | Cause |
|---|---|---|
| `403` | `CSRF check failed` | Missing or incorrect CSRF header |
| `409` | `update already in progress` | Another upload is active or a successful update is already reboot-pending |
| `500` | `stream did not drain in time` | Active stream could not be stopped before flash |
| `500` | `upload interrupted (client disconnected)` | Browser/network aborted before the upload completed |
| `500` | `image too large (N bytes) for OTA slot (M bytes)` | Binary exceeds inactive slot size |
| `500` | `no OTA partition available` | Device was not serial-flashed with the OTA partition table |
| `500` | `aborted by request` | `/api/ota/abort` was called during upload |
| `500` | _(Update.h error string)_ | Flash write or verification failure |

The device reboots automatically ~1.5 s after the `200` response is sent.

---

### `POST /api/ota/abort`

Aborts an upload that is currently in the `receiving` phase. Requires `X-ESP32MIC-CSRF: 1`.

```bash
curl -X POST \
  -H 'X-ESP32MIC-CSRF: 1' \
  http://<device-ip>/api/ota/abort
```

Success response (`200`):

```json
{"ok": true, "message": "OTA aborted"}
```

Conflict response (`409`, no upload in progress or already past receiving):

```json
{"ok": false, "error": "no update in progress or not safe to abort"}
```

Abort is only accepted while `phase` is `receiving`. It cannot cancel a reboot that is already pending.

---

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
