# Streaming details

The firmware exposes one live audio source through HTTP and RTSP/TCP.

## Ports

| Port | Purpose |
|---:|---|
| 80 | Web UI and `/api/*` control endpoints |
| 81 | HTTP audio streams |
| 8554 | RTSP/TCP audio stream |

## HTTP streams

| URL | Format | Notes |
|---|---|---|
| `http://<ip>:81/stream` | Depends on `STREAM_WAV_ENABLE` | Backward-compatible default endpoint |
| `http://<ip>:81/stream.wav` | WAV header + PCM16 | Best HTTP endpoint for browsers and generic players |
| `http://<ip>:81/stream.pcm` | Raw `audio/L16` PCM16 | Minimal overhead; client must know sample rate |

All HTTP streams use the same audio source and same single-client gate.

## RTSP stream

```text
rtsp://<device-ip>:8554/audio/
```

- Transport: RTP over RTSP/TCP interleaved
- UDP RTSP transport is intentionally rejected
- Payload: L16 mono PCM
- Sample rate: active audio profile (`48 kHz` or `24 kHz`)

For ffplay:

```bash
ffplay -rtsp_transport tcp rtsp://<device-ip>:8554/audio/
```

For VLC, make sure it uses RTSP over TCP if it does not automatically retry after the initial UDP setup rejection.

## BirdNET-Go

Recommended source URL:

```text
rtsp://<device-ip>:8554/audio/
```

Suggested starting point:

- Use **Stability — 24 kHz** if Wi-Fi is weak or reconnects happen often
- Use **Quality — 48 kHz** if the stream is stable and you want maximum high-frequency detail

Changing the audio profile requires **Restart Audio** from the UI or API.

## Single-client behavior

Only one stream client can be active at a time, across both HTTP and RTSP.

Examples:

- If VLC is playing RTSP, the dashboard audio player cannot also stream HTTP.
- If the dashboard audio player is active, BirdNET-Go RTSP will be rejected until HTTP stops.

The dashboard shows current stream state and total sessions since boot.

## Stability behavior

The stream server is intentionally tolerant of short network stalls. It also drains stale ring-buffer audio before a new HTTP or RTSP session so new clients start close to live audio instead of receiving old backlog.

Relevant build flags:

| Flag | Default | Meaning |
|---|---:|---|
| `STREAM_WRITE_STALL_LIMIT` | 500 | HTTP write stall tolerance |
| `STREAM_IDLE_TIMEOUT_COUNT` | 30 | HTTP idle ring-buffer timeout, roughly seconds |
| `RTSP_WRITE_STALL_LIMIT` | 500 | RTSP write stall tolerance |
| `RTSP_IDLE_TIMEOUT_COUNT` | 30 | RTSP idle ring-buffer timeout, roughly seconds |
| `RTSP_SETUP_TIMEOUT_MS` | 30000 | RTSP setup/play inactivity timeout |

When a client stops playback abruptly, the ESP32 may log a write failure such as `Connection reset by peer`. That usually means the client closed the socket while audio was being written; it is normal when stopping VLC or similar clients.
