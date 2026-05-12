# Architecture notes

## Main flow

```text
I2S microphone
  -> AudioPipeline producer task
  -> FreeRTOS ring buffer
  -> StreamServer
      -> HTTP :81
      -> RTSP/TCP :8554

Web UI / API :80
  -> HttpControl
  -> RuntimeSettings / NetworkManager / Scheduler
```

## Main modules

| Module | Responsibility |
|---|---|
| `AudioPipeline` | I2S setup, DMA reads, 32-bit to PCM16 conversion, ring-buffer writes |
| `StreamServer` | HTTP audio streaming, RTSP/TCP streaming, stream session diagnostics |
| `HttpControl` | `/api/*` endpoints |
| `NetworkManager` | Wi-Fi credentials, setup AP, reconnect behavior |
| `RuntimeSettings` | NVS-backed runtime settings |
| `Scheduler` | NTP, civil dawn/dusk, sleep timing |
| `LogBuffer` | Centralized logging: Serial output, in-memory RAM ring (exposed via `/api/logs`), optional remote UDP/syslog sink |
| `AppState` | Shared application state and persisted counters |

## Logging flow

All module logging is routed through centralized macros in `LogBuffer.h`. Each call goes through a single sink:

```text
LOG* macro (LOGE/LOGW/LOGI/LOGD)
  -> LogBuffer
      -> Serial (always)
      -> RAM ring buffer (always; exposed via /api/logs)
      -> optional UDP/syslog (if ENABLE_REMOTE_LOG=1 and Wi-Fi connected)
```

Compile-time `LOG_LEVEL` controls which severity levels are compiled in. `REMOTE_LOG_MIN_LEVEL` adds a second threshold for the remote sink only, applied after compile-time filtering. The `/api/logs` endpoint always reflects the RAM ring regardless of remote logging configuration.

## Audio pipeline

The microphone is read as 32-bit I2S samples. The producer shifts samples down to signed 16-bit PCM using `convert_shift`, then pushes chunks into a FreeRTOS ring buffer.

The stream server reads chunks from the ring buffer, applies the high-pass filter, and sends audio to the active client.

## Ring buffer behavior

`RB_CAPACITY_BYTES` is the preferred capacity. If allocation fails, the firmware falls back to smaller sizes so audio can still start on RAM-constrained ESP32 boards.

Before a new stream starts, stale queued audio is drained so clients receive near-live audio rather than old backlog.

## Runtime settings

Runtime settings are stored in NVS and loaded at boot.

Some settings are hot-applied:

- Wi-Fi TX power
- HPF enabled
- HPF cutoff

Some settings are captured only when the audio pipeline starts:

- `convert_shift`
- `audio_profile`

Those require **Restart Audio** to become active.

## Wi-Fi policy

Wi-Fi modem sleep is disabled during normal operation. This favors stream stability over power saving because modem sleep can introduce latency spikes and stream write stalls.

## Scheduler

If deep sleep is enabled, the scheduler uses NTP time plus configured latitude/longitude to compute civil dawn and dusk. The device sleeps at night and wakes at the next dawn.

Set `ENABLE_DEEP_SLEEP=0` for always-on testing or deployments.
