# Changelog

All notable changes to this project will be documented in this file.

## Unreleased

### Added

- OTA updates available from the system page.

## 2.0.0

I should have done changelogs before 2.0.0. I'll do better now, I promise !

### Added

- Added RTSP/TCP audio streaming at `rtsp://<device-ip>:8554/audio/`.
- Added explicit HTTP audio endpoints:
  - `/stream.wav` for WAV-wrapped PCM audio
  - `/stream.pcm` for raw PCM/L16 audio
  - `/stream` remains available for backward compatibility
- Added BirdNET-Go-oriented runtime audio profiles:
  - `quality_48k` for full 48 kHz capture
  - `stability_24k` for lower-bandwidth 24 kHz capture
- Added Audio page controls for changing the active audio profile.
- Added dashboard and Audio page RTSP URL display with copy buttons.
- Added clearer stream status badges for idle, HTTP streaming, RTSP connected, and RTSP streaming states.
- Added richer stream/session diagnostics in status APIs, including active transport, RTSP state, session counters, disconnect reason, session duration, and transmitted bytes.
- Added RTSP task stack high-water reporting and Wi-Fi sleep policy visibility in performance status.
- Added documentation pages for streaming, configuration, API, and architecture details.

### Changed

- Simplified `README.md` into a quick-start focused guide and moved deeper reference material into `docs/`.
- Renamed UI stream counter label from “Connections” to “Sessions since boot”.
- Increased default stream stall/idle tolerance for better behavior with BirdNET-Go and marginal Wi-Fi.
- Increased preferred ring-buffer size and added fallback allocation so constrained boards still boot successfully.
- Stream startup now drains stale buffered audio so new clients start near live audio instead of receiving old backlog.
- Dashboard and Audio page now prefer `/stream.wav` for browser playback.
- Web UI now stops the browser audio player before applying audio settings or restarting audio.

### Fixed

- Fixed audio pipeline startup failure when the preferred ring-buffer size cannot be allocated.
- Fixed deprecated ESP32 I2S communication format constants.
- Fixed dashboard browser playback when `/stream` is configured as raw PCM.
- Fixed RTSP PLAY response compatibility by adding RTP-Info and aligning track naming.
- Fixed RTSP/TCP parser handling so client interleaved binary frames do not corrupt RTSP request parsing.
- Fixed RTSP transport negotiation by rejecting unsupported UDP setup requests instead of pretending TCP was negotiated.
- Fixed misleading RTSP client logging after socket reset by preserving the connected client IP.
- Fixed noisy missing `/favicon.ico` route logs.
- Improved HTTP stream failure handling so repeated write failures across chunks close broken clients more reliably.

### Notes

- RTSP is TCP/interleaved only; UDP RTSP transport is intentionally rejected.
- Only one stream client can be active at a time across HTTP and RTSP.
- Changing `audio_profile` or `convert_shift` requires Restart Audio to apply the new active pipeline settings.
