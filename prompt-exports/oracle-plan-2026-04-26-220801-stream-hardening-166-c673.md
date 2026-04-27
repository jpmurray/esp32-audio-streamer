## Final Prompt
<taskname="Stream hardening"/>
<task>Harden `esp32-audio-streamer` network/audio streaming behavior without changing overall architecture or protocol. Focus on robustness of the current HTTP stream path on port 81 under disconnects, stalled clients, partial writes, and audio pipeline backpressure. Keep module boundaries intact (`AudioPipeline`, `StreamServer`, `HttpControl`, `RuntimeSettings`, existing Web UI/API).</task>

<architecture>
- `src/StreamServer.cpp` owns the dedicated stream server task and blocking `/stream` handler, writing ring-buffer audio chunks to `WiFiClient`.
- `src/AudioPipeline.cpp` owns I2S capture producer + ring buffer writes + audio metrics; HPF is applied by `StreamServer` at transmit time.
- `src/HttpControl.cpp` exposes `/api/audio_status`, `/api/status`, `/api/perf_status`, and actions (`restart-audio`, etc.) that already surface stream and audio metrics.
- `src/AppState.cpp` / `include/AppState.h` hold shared globals (`g_ringbuf`, `g_rb_ok`, `g_i2s_ok`, producer task handle).
- `src/main.cpp` wires startup order and lifecycle (`audioPipeline_init()`, `streamServer_init()`, API registration), so stream-hardening changes must fit current sequencing.
- `webui/index.html` already renders stream/audio/perf status and can surface small additional observability fields if needed.
</architecture>

<selected_context>
esp32-audio-streamer/src/StreamServer.cpp: `/stream` request lifecycle, response headers, WAV header path, ringbuffer receive loop, partial-write loop, active/connect counters, stream task setup.
esp32-audio-streamer/include/StreamServer.h: public stream state and init/task API.
esp32-audio-streamer/src/AudioPipeline.cpp: producer backpressure behavior (`xRingbufferSend` drop path), metrics counters (`rb_drop_count`, `i2s_error_count`), stop/init lifecycle.
esp32-audio-streamer/include/AudioPipeline.h: `AudioMetrics` contract and public API used by control plane.
esp32-audio-streamer/src/HttpControl.cpp: current status JSON schemas and where stream/audio counters are exposed; strict API style for adding observability fields.
esp32-audio-streamer/include/HttpControl.h: control route registration boundary.
esp32-audio-streamer/src/AppState.cpp: shared-state definitions used by stream/audio modules.
esp32-audio-streamer/include/AppState.h: shared globals and preferences keys used across modules.
esp32-audio-streamer/src/RuntimeSettings.cpp: runtime setting mutation patterns and persistence approach (for any safe tunables introduced).
esp32-audio-streamer/include/RuntimeSettings.h: runtime settings schema/validation boundaries.
esp32-audio-streamer/src/LogBuffer.cpp: logging backend used for diagnostics and `/api/logs`.
esp32-audio-streamer/include/LogBuffer.h: logging API and limits.
esp32-audio-streamer/src/main.cpp: boot/runtime orchestration and ownership boundaries.
esp32-audio-streamer/webui/index.html: existing UI cards, polling, and stream/audio presentation for minimal observability additions.
esp32-audio-streamer/README.md: documented stream model, constraints, and API contracts to keep aligned.
esp32-audio-streamer/local_env.ini.example: compile-time knobs currently available.
esp32-audio-streamer/platformio.ini: build config loading path.
esp32-audio-streamer/tools/gen_webui_gzip_header.sh: UI embedding pipeline if UI changes are made.
</selected_context>

<relationships>
- `StreamServer` consumes `g_ringbuf` produced by `AudioPipeline` and updates stream globals consumed by `HttpControl` JSON endpoints/UI.
- `AudioPipeline` drop/error metrics currently capture producer-side pressure; there is no dedicated transmit-side error accounting yet in `StreamServer`.
- `HttpControl`/UI poll and display status every few seconds, making them the natural observability surface for stream robustness counters/state.
- `main.cpp` restart-audio action currently stops/reinits pipeline while stream task remains alive, so stream lifecycle handling around transient unavailability matters.
- Existing design intentionally separates control plane (port 80) from stream plane (port 81); keep that design.
</relationships>

<ambiguities>
- No explicit stream socket timeout/retry policy is currently documented; implementation should infer safe defaults consistent with existing behavior.
- Current code tracks connection count/active state and producer drops, but transmit-side failure taxonomy is not yet explicit.
</ambiguities>

<constraints>
- No architectural rewrite, no protocol migration, no board/hardware assumption changes.
- Preserve one-client-at-a-time stream model unless a change is strictly necessary for robustness.
- Favor incremental hardening: bounded write-failure handling, stuck-client escape paths, clearer lifecycle state transitions, and observability counters.
- Keep API/UI additions small and consistent with existing JSON/style conventions.
</constraints>

## Selection
- Files: 18 total (18 full)
- Total tokens: 31198 (Auto view)
- Token breakdown: full 31198

### Files
### Selected Files
/Users/jpmurray/Code/Repositories/Perso/esp32-audio-streamer/
├── include/
│   ├── AppState.h — 588 tokens (full)
│   ├── AudioPipeline.h — 599 tokens (full)
│   ├── HttpControl.h — 174 tokens (full)
│   ├── LogBuffer.h — 421 tokens (full)
│   ├── RuntimeSettings.h — 723 tokens (full)
│   └── StreamServer.h — 268 tokens (full)
├── src/
│   ├── AppState.cpp — 449 tokens (full)
│   ├── AudioPipeline.cpp — 2,740 tokens (full)
│   ├── HttpControl.cpp — 4,637 tokens (full)
│   ├── LogBuffer.cpp — 920 tokens (full)
│   ├── RuntimeSettings.cpp — 1,930 tokens (full)
│   ├── StreamServer.cpp — 1,874 tokens (full)
│   └── main.cpp — 5,556 tokens (full)
├── tools/
│   └── gen_webui_gzip_header.sh — 429 tokens (full)
├── webui/
│   └── index.html — 6,186 tokens (full)
├── README.md — 2,895 tokens (full)
├── local_env.ini.example — 641 tokens (full)
└── platformio.ini — 168 tokens (full)


---

## Generated Plan

## Chat Send ✅
- **Chat**: `stream-hardening-166A8C` | **Mode**: plan

### Response
**Summary**
Use a targeted hardening change, not a refactor. The weak points are localized to `StreamServer` session lifecycle and the `restart-audio` action: the current stream loop can spin forever on zero-byte writes, can take too long to notice disconnects, has no transmit-side observability, and can race `audioPipeline_stop()` during restart. Keep the existing port split, one-client-at-a-time model, ring buffer ownership, and HTTP protocol. Add bounded stream write/idle handling inside `StreamServer`, add a pause-and-drain control surface so `HttpControl` can restart audio safely, expose a small set of transmit-side counters through `/api/audio_status`, and surface those in the existing Web UI.

**Current-state analysis**
`AudioPipeline` owns capture and producer-side backpressure today. `audioPipeline_init()` in `src/AudioPipeline.cpp` installs I2S, creates `g_ringbuf`, starts `i2sProducerTask` on core 0, computes HPF coefficients, and snapshots `g_runtime_settings.convert_shift` into `s_active_convert_shift`. The producer task reads I2S, converts 32-bit samples to PCM-16, updates `AudioMetrics`, and writes chunks into `g_ringbuf` with `xRingbufferSend()`. When the ring buffer is full, it increments `s_rb_drop_count`; this is the only current backpressure metric.

`StreamServer` in `src/StreamServer.cpp` owns the dedicated port-81 `WebServer` and the blocking `/stream` handler running on its own FreeRTOS task on core 0. `handleStream()` checks `g_rb_ok`, increments `g_stream_connect_count`, sets `g_stream_active = true`, writes HTTP headers directly to `WiFiClient`, then loops on `xRingbufferReceive(g_ringbuf, ..., 1000 ms)`. For each received chunk it applies `audioPipeline_applyHPF()` and writes until `to_write == 0` or `client.connected()` becomes false. If `client.write()` returns `0`, it only `delay(1)` and retries forever. There is no timeout, no disconnect taxonomy, no explicit cleanup reason, and no guard against the stream being administratively stopped.

`HttpControl` in `src/HttpControl.cpp` is the control-plane boundary on port 80. `handleApiAudioStatus()` reuses `audioPipeline_getMetrics()` and existing globals (`g_stream_active`, `g_stream_connect_count`, `g_i2s_ok`, `g_rb_ok`) to return a flat JSON payload. There is no transmit-side metric from `StreamServer`, so the UI can only see producer drops and I2S errors, not whether the consumer side is stuck. `handleApiRestartAudio()` currently sends `200 OK` immediately, then calls `audioPipeline_stop()` and `audio


> 💡 Continue this plan conversation with ask_oracle(chat_id: "stream-hardening-166A8C", new_chat: false)