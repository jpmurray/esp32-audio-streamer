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
| `/api/perf_status` | Heap, CPU MHz, stack high-water marks, Wi-Fi sleep policy, RSSI, remote-log status |
| `/api/logs` | Recent in-memory logs (RAM ring buffer only; remote logging does not change this response) |
| `/api/wifi_status` | Saved/connected SSID, IPs, RSSI, setup AP state, last error |
| `/api/wifi_scan` | Wi-Fi scan results |
| `/api/ota/status` | OTA phase, progress, free slot size, last error |

### `/api/logs` note

`/api/logs` returns the contents of the in-memory RAM ring buffer as a JSON array of log strings. It always reflects all logs written since boot (up to the ring capacity), regardless of whether remote UDP/syslog logging is enabled. Enabling `ENABLE_REMOTE_LOG` does not change the shape or contents of this response.

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

## `/api/audio_status` write-stall diagnostics

The following fields are included in `/api/audio_status` to diagnose TCP write stalls:

| Field | Type | Description |
|---|---|---|
| `write_stall_limit` | int | Compile-time HTTP write-stall limit (default 50) |
| `rtsp_write_stall_limit` | int | Compile-time RTSP write-stall limit (default 50) |
| `stream_write_stalls` | uint | Cumulative zero-byte write stalls in the current/last session |
| `current_max_consecutive_write_stalls` | uint | Longest run of consecutive stalls in the active session (0 when idle) |
| `last_session_write_stalls` | uint | Total stalls from the previous completed session |
| `last_session_max_consecutive_write_stalls` | uint | Max consecutive stalls from the previous completed session |
| `last_write_errno` | int | Best-effort `errno` value after the most recent zero-byte write; 0 if unavailable |

A stalled client is closed after `write_stall_limit` (HTTP) or `rtsp_write_stall_limit` (RTSP) consecutive zero-byte write rounds (~1 ms each), so teardown occurs quickly under TCP backpressure.

RTSP defaults to a maximum advertised sample rate of 24 kHz (`RTSP_MAX_SAMPLE_RATE_HZ=24000`). When the active audio profile exceeds that limit, RTSP `DESCRIBE` is rejected with `551 Option Not Supported`; switch to `stability_24k` or set `RTSP_MAX_SAMPLE_RATE_HZ=0` to disable the guard.

## `/api/audio_status` idle-discard counters

When no HTTP or RTSP consumer is actively receiving audio, the I2S producer still reads I2S DMA and updates level/clip metrics, but skips the ring-buffer send. The discarded data is counted:

| Field | Type | Description |
|---|---|---|
| `idle_discard_count` | uint | Cumulative producer chunks discarded while no consumer was active |
| `idle_discard_bytes` | uint | Cumulative PCM bytes discarded while no consumer was active |

These counters climbing while `stream_active` is `false` is expected normal behaviour. `rb_drop_count` should remain flat while no consumer is connected.

## `/api/perf_status` `remote_log` object

The `remote_log` nested object is always present in `/api/perf_status`, regardless of whether `ENABLE_REMOTE_LOG` was set at build time.

| Field | Type | Description |
|---|---|---|
| `compiled_enabled` | bool | `true` if `ENABLE_REMOTE_LOG=1` was set at build time |
| `configured` | bool | `true` if `REMOTE_LOG_HOST` was a valid IPv4 literal at init |
| `suspended` | bool | `true` if the sink is currently in a backoff suspension window |
| `total_attempts` | uint | Cumulative UDP send attempts since boot |
| `total_successes` | uint | Cumulative successful UDP sends since boot |
| `total_failures` | uint | Cumulative send failures since boot |
| `total_skipped_weak_rssi` | uint | Sends skipped due to RSSI below `REMOTE_LOG_MIN_RSSI_DBM` |
| `consecutive_failures` | uint | Failures since the last success (resets on success or after backoff expires) |
| `suspended_until_ms` | uint | `millis()` value when the current suspension expires; 0 when not suspended |
| `last_rssi_dbm` | int | STA RSSI observed at the last send attempt; 0 if no attempt has been made |

When `suspended` is `true`, remote UDP sends are paused until `suspended_until_ms`. The suspension is cleared automatically when the backoff window expires; the device does **not** need to be rebooted.

When `compiled_enabled` is `false`, all counters are zero and the object is informational only.

## `/api/wifi_status` RSSI quality fields

The following fields are always present in `/api/wifi_status` (and in the `wifi` sub-object of `/api/status`):

| Field | Type | Description |
|---|---|---|
| `rssi_dbm` | int or null | Current STA RSSI in dBm; `null` when disconnected |
| `rssi_quality` | string | `good`, `weak`, `unstable`, or `unknown` (see thresholds below) |
| `rssi_streaming_warning` | bool | `true` when quality is `weak` or `unstable` |
| `rssi_warn_dbm` | int | Threshold at or below which quality is `weak` (compile-time `WIFI_RSSI_WARN_DBM`, default -70) |
| `rssi_unstable_dbm` | int | Threshold at or below which quality is `unstable` (compile-time `WIFI_RSSI_UNSTABLE_DBM`, default -75) |

`rssi_quality` values:

| Value | Condition | Streaming impact |
|---|---|---|
| `good` | RSSI > `rssi_warn_dbm` | Acceptable |
| `weak` | RSSI ≤ `rssi_warn_dbm` | May cause occasional stalls or reconnects |
| `unstable` | RSSI ≤ `rssi_unstable_dbm` | Likely to cause stream interruptions |
| `unknown` | STA disconnected | Cannot be assessed |
