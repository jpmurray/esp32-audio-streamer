# ESP32 Audio Streamer: Fast Safe Path Implementation Plan

## Locked Decisions

1. Keep HTTP PCM/WAV streaming. Do not adopt RTSP.
2. Keep the existing civil dawn/dusk scheduling and deep-sleep behavior.
3. Keep Wi-Fi credentials, pin mapping, lat/lon, and timezone as compile-time configuration for now.
4. Move the live audio stream to a dedicated stream server/port in a later phase; do not change transport behavior in Phase 1.
5. Keep `GET /status` and `GET /uptime` stable for compatibility.
6. Put all new control and UI features under `/api/*` in later phases.

## Work Items

- [x] Item 1: Extract the current monolithic `src/main.cpp` into small modules without changing behavior.
  - Goal: create module boundaries for shared app state, audio pipeline, and scheduler/time logic so later phases can add control APIs and a dedicated stream server safely.
  - Done when:
    - `src/main.cpp` is reduced to application wiring plus route registration and loop orchestration.
    - Existing `/stream`, `/status`, and `/uptime` behavior still works.
    - Existing NTP sync, dawn/dusk computation, and sleep logic still works.
    - HPF/DSP ownership moves into the audio pipeline module instead of living inside the stream transport handler.
  - Key files/modules:
    - `src/main.cpp`
    - `include/AppState.h`, `src/AppState.cpp`
    - `include/AudioPipeline.h`, `src/AudioPipeline.cpp`
    - `include/Scheduler.h`, `src/Scheduler.cpp`
  - Dependencies: none
  - Size: large

- [x] Item 2: Separate live streaming ownership from the control-plane HTTP server.
  - Goal: keep status/API routes responsive while audio is actively streaming.
  - Done when:
    - audio streaming is served from a dedicated stream server/task on a separate port
    - control/status HTTP remains responsive during active streaming
    - stream state and stream URL are visible in richer status output
  - Key files/modules:
    - `include/StreamServer.h`, `src/StreamServer.cpp`
    - `src/main.cpp`
    - `include/AppState.h`, `src/AppState.cpp`
  - Dependencies: Item 1
  - Size: large

- [x] Item 3: Add runtime settings, strict mutation APIs, logs, and richer status surfaces.
  - Goal: adopt the highest-value portable control-plane features from the BirdNET repos after the codebase is modular and stream ownership is separated.
  - Done when:
    - first-wave runtime settings persist in `Preferences`
    - strict validation exists for mutation endpoints
    - logs, audio metrics, and perf data are available via HTTP APIs
  - Key files/modules:
    - `include/RuntimeSettings.h`, `src/RuntimeSettings.cpp`
    - `include/LogBuffer.h`, `src/LogBuffer.cpp`
    - `include/HttpControl.h`, `src/HttpControl.cpp`
    - `src/main.cpp`
  - Dependencies: Items 1 and 2
  - Size: large

## Scope For The Current Session

Only Item 1 is in scope now.

### Item 1 Guidance

- Preserve existing behavior and endpoint contracts.
- Prefer plain structs and small free functions over classes.
- Keep compile-time configuration intact.
- Move code, do not redesign yet.
- Do not add Web UI, `/api/*`, dedicated stream server, or runtime settings persistence in this item.

### Verification For Item 1

- Firmware still builds.
- `/stream` still serves audio in the current format.
- `/status` and `/uptime` still respond with the expected existing fields.
- Existing NTP retry and night-sleep behavior remain intact.

## Progress Notes

- Complete: Item 1 finished. `main.cpp` is now a thinner wiring layer, and `AppState`, `AudioPipeline`, and `Scheduler` modules were introduced while preserving the legacy `/stream`, `/status`, and `/uptime` routes. HPF/DSP ownership moved into `AudioPipeline`.
- Complete: Item 2 finished. `StreamServer` module added (`include/StreamServer.h`, `src/StreamServer.cpp`). Audio streaming moved to port 81 in its own FreeRTOS task pinned to core 0; control-plane HTTP stays on port 80. `GET /status` now includes `stream.url`, `stream.active`, and `stream.connect_count`. Build verified.
- Complete: Item 4 finished. Embedded Web UI added. `webui/index.html` (plain HTML/CSS/JS, ~10 kB source) is compressed to a 3889-byte gzip PROGMEM blob via `tools/gen_webui_gzip_header.sh` → `include/WebUI_gz.h`. `GET /` now serves the blob with `Content-Encoding: gzip` via `server.send_P()`. The UI polls `/api/status`, `/api/audio_status`, `/api/perf_status` every 5 s and `/api/logs` every 15 s; shows stream URL on port 81 with inline audio player; allows editing `wifi_tx_power_dbm`, `hpf_enabled`, `hpf_cutoff_hz` with CSRF header; and exposes restart-audio, time-sync, reboot actions. Flash usage: 65.2 % (+0.3 %).
- Complete: Item 3 finished. `RuntimeSettings`, `LogBuffer`, and `HttpControl` modules added. Runtime settings (`wifi_tx_power_dbm`, `hpf_enabled`, `hpf_cutoff_hz`) persist in the `runtime` Preferences namespace. All `/api/*` routes registered. CSRF header required for mutations. Build verified (RAM 16.8 %, Flash 64.9 %). Deferred: `convert_shift` (read-only in status), `stream_wav_enable` (unsafe mid-stream), `log_level` (compile-time macros).
- Complete: Item 4 finished. Added an embedded Web UI with source at `webui/index.html`, generator script at `tools/gen_webui_gzip_header.sh`, and generated header at `include/WebUI_gz.h`. `GET /` now serves the gzip-compressed UI, which uses the existing `/api/*` endpoints for status, settings, actions, and logs. Verified locally with `platformio run` (RAM 16.8 %, Flash 65.2 %).
- Pending: Item 5 is UI polish and mobile behavior improvements on top of the existing embedded Web UI. Scope should stay front-end focused and keep the existing `/api/*` surface stable unless a tiny compatibility fix is clearly justified.
