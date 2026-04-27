## Final Prompt
<taskname="ESP32 feature adoption"/>
<task>Compare `esp32-audio-streamer` against `birdnetgo-esp32-rtsp-mic` and `birdnetgo-m5stack-atom-echo-rtsp-mic`, then produce a concrete staged implementation plan (not a rewrite) to bring the best ideas into `esp32-audio-streamer`. Focus on module boundaries, configuration/runtime patterns, hardware assumptions, overlapping capabilities, differences between the two external repos, what to adopt first, and what to avoid.</task>

<architecture>
- `esp32-audio-streamer/src/main.cpp` is a single-file HTTP PCM streamer with: Wi-Fi connect, `/stream`, `/uptime`, `/status`, NTP sync (`configTzTime` + retry), dawn/dusk solar scheduler, deep-sleep-at-night logic, RTC/NVS persistence (`Preferences`), I2S producer task + ring buffer.
- `birdnetgo-esp32-rtsp-mic/esp32_rtsp_mic_birdnetgo/esp32_rtsp_mic_birdnetgo.ino` is a larger RTSP firmware with runtime-configurable audio/network/reliability settings, WebUI-controlled API, WiFiManager onboarding, mDNS, optional OTA, stream scheduling + optional deep sleep outside stream window, thermal latch protection, richer diagnostics, MQTT telemetry + Home Assistant discovery, persistent settings in `Preferences`.
- `birdnetgo-esp32-rtsp-mic/esp32_rtsp_mic_birdnetgo/WebUI.cpp` is API/controller layer (status/audio/perf/thermal/log/action endpoints + strict setting parsing + mutation header auth). UI content is gzip-served from `WebUI_gz.h`, generated from `webui/index.html` via `tools/gen_webui_gzip_header.sh`.
- `birdnetgo-m5stack-atom-echo-rtsp-mic/src/esp32_rtsp_mic_birdnetgo.ino` is a board-specific derivative emphasizing dual-core streaming ownership and M5 hardware features (PDM mic path, LED/AGC controls, socket ownership handoff).
- `birdnetgo-m5stack-atom-echo-rtsp-mic/src/WebUI.cpp` is an inline HTML WebUI variant (not gzipped asset pipeline), with AGC/LED controls and M5-focused diagnostics.
</architecture>

<selected_context>
esp32-audio-streamer/src/main.cpp: Monolithic baseline implementation; contains current scheduling/NTP/deep-sleep behavior, status JSON contract, I2S capture + ring buffer stream path, compile-time flags, and lifecycle sequencing.
esp32-audio-streamer/README.md: Documents current HTTP API and build-flag model used by project.
esp32-audio-streamer/platformio.ini: Current PlatformIO env baseline.
esp32-audio-streamer/local_env.ini.example: Current local config knobs and defaults.

birdnetgo-esp32-rtsp-mic/esp32_rtsp_mic_birdnetgo/esp32_rtsp_mic_birdnetgo.ino: Primary external reference for runtime settings persistence, MQTT/HA discovery, schedule/deep-sleep policy, mDNS/WiFiManager, thermal lockout model, diagnostics, and RTSP server control.
birdnetgo-esp32-rtsp-mic/esp32_rtsp_mic_birdnetgo/WebUI.cpp: Concrete runtime-control API surface and strict input validation model; mutation endpoints, status/perf/audio/thermal/logs/actions.
birdnetgo-esp32-rtsp-mic/esp32_rtsp_mic_birdnetgo/WebUI.h: WebUI integration boundary from main firmware.
birdnetgo-esp32-rtsp-mic/esp32_rtsp_mic_birdnetgo/webui/index.html: Browser UI behavior and settings UX surface mapped to API keys.
birdnetgo-esp32-rtsp-mic/esp32_rtsp_mic_birdnetgo/tools/gen_webui_gzip_header.sh: WebUI asset pipeline used to embed gzip HTML in firmware.
birdnetgo-esp32-rtsp-mic/esp32_rtsp_mic_birdnetgo/README.md: Feature map, defaults, runtime config keys, API capabilities, hardware assumptions for C6 + ICS-43434.
birdnetgo-esp32-rtsp-mic/esp32_rtsp_mic_birdnetgo/CHANGELOG.md: Chronology of added features (time/network schedule, deep sleep policy, MQTT additions, API hardening) useful for staged adoption ordering.
birdnetgo-esp32-rtsp-mic/README.md: Top-level concise product summary + capabilities.

birdnetgo-m5stack-atom-echo-rtsp-mic/src/esp32_rtsp_mic_birdnetgo.ino: Alternative architecture emphasizing dual-core socket ownership, AGC, LED modes, PDM mic path, and synchronization primitives.
birdnetgo-m5stack-atom-echo-rtsp-mic/src/WebUI.cpp: AGC/LED-facing UI/API variant and additional UX hints.
birdnetgo-m5stack-atom-echo-rtsp-mic/src/WebUI.h: Interface boundary (includes `requestStreamStop` extern note).
birdnetgo-m5stack-atom-echo-rtsp-mic/docs/DETAILS.md: Explicit explanation of dual-core ownership model and cross-core safety techniques.
birdnetgo-m5stack-atom-echo-rtsp-mic/README.md: Board-focused feature and defaults summary.
birdnetgo-m5stack-atom-echo-rtsp-mic/platformio.ini: Dependency and board differences for M5/ESP32-PICO setup.
</selected_context>

<relationships>
- `esp32-audio-streamer/src/main.cpp` currently owns all concerns in one translation unit; scheduling, NTP, sleep, web endpoints, and audio pipeline are tightly coupled.
- In birdnetgo-C6 project, `.ino` owns runtime state + persistence; `WebUI.cpp` directly mutates that state via externs and calls back into `.ino` helpers (`saveAudioSettings`, `restartI2S`, time/mDNS/MQTT functions).
- C6 WebUI/API keys map directly to persisted NVS keys in `.ino` (runtime-configurable control plane pattern).
- C6 project’s deep sleep is schedule-window-based with explicit safety gating; baseline project’s deep sleep is dawn/dusk-based nightly sleep.
- M5 project diverges mainly in hardware/audio threading model (PDM, dual-core socket ownership, LED/AGC) and should be treated as selective pattern source rather than direct drop-in.
</relationships>

<comparison_focus>
- Extract strongest ideas from C6 repo likely portable to current project: strict settings API validation, safer mutation semantics, richer diagnostics/status schema patterns, schedule/deep-sleep safety guards, persisted runtime config shape, changelog-informed rollout order.
- Extract strongest ideas from M5 repo that are architecture patterns (not hardware-specific): explicit ownership for streaming socket/task boundaries, cross-core stop/cleanup handshakes, optional AGC concept, operational observability patterns.
- Identify what should be avoided for current codebase: board-specific pin/power assumptions, wholesale RTSP server migration when current project is HTTP PCM, M5-specific libraries (`M5Atom`, `FastLED`), and over-coupling UI layer directly to globals without modularization plan.
</comparison_focus>

<deliverable_expectations>
- Provide a staged migration plan tailored to `esp32-audio-streamer` structure, with phases, prerequisites, risk notes, and test/validation checkpoints.
- Call out concrete modules to introduce (e.g., config/state, API handlers, scheduler policy adapter, telemetry/logging) while preserving current behavior first.
- Distinguish near-term high-value/low-risk adoptions vs. optional advanced features.
- Include explicit handling of capability overlap/conflict: existing civil dawn/dusk sleep model vs. stream-window scheduling model from external repos.
</deliverable_expectations>

<ambiguities>
- `prompt` state in this tab started empty; no extra user-authored text was present beyond the discover request.
- External repos target different hardware and stream transport assumptions (RTSP + single client vs current HTTP PCM); transferability must be judged per module.
</ambiguities>

## Selection
- Files: 18 total (18 full)
- Total tokens: 102628 (Auto view)
- Token breakdown: full 102628

### Files
### Selected Files
/Users/jpmurray/Code/Repositories/Perso/esp32-audio-streamer/
├── src/
│   └── main.cpp — 11,386 tokens (full)
├── README.md — 1,219 tokens (full)
├── local_env.ini.example — 538 tokens (full)
└── platformio.ini — 168 tokens (full)
/Users/jpmurray/Code/Repositories/Tools/birdnetgo-esp32-rtsp-mic/
├── esp32_rtsp_mic_birdnetgo/
│   ├── tools/
│   │   └── gen_webui_gzip_header.sh — 195 tokens (full)
│   ├── webui/
│   │   └── index.html — 16,571 tokens (full)
│   ├── CHANGELOG.md — 1,599 tokens (full)
│   ├── README.md — 3,506 tokens (full)
│   ├── WebUI.cpp — 7,835 tokens (full)
│   ├── WebUI.h — 67 tokens (full)
│   └── esp32_rtsp_mic_birdnetgo.ino — 23,420 tokens (full)
└── README.md — 1,539 tokens (full)
/Users/jpmurray/Code/Repositories/Tools/birdnetgo-m5stack-atom-echo-rtsp-mic/
├── docs/
│   └── DETAILS.md — 1,370 tokens (full)
├── src/
│   ├── WebUI.cpp — 17,025 tokens (full)
│   ├── WebUI.h — 95 tokens (full)
│   └── esp32_rtsp_mic_birdnetgo.ino — 15,123 tokens (full)
├── README.md — 812 tokens (full)
└── platformio.ini — 160 tokens (full)


---

## Generated Plan

## Chat Send ✅
- **Chat**: `esp32-feature-adoption-97988D` | **Mode**: plan

### Response
1. **Summary**

`esp32-audio-streamer` should adopt the `birdnetgo` repos’ control-plane patterns, observability, and scheduling hardening, but keep its existing transport and core operating model: HTTP PCM streaming, civil-dawn/civil-dusk scheduling, RTC/NVS persistence, and the current I2S producer + ring buffer approach. This is not a rewrite, but it is more than a one-file patch: the current monolith and the blocking `/stream` handler are the two real constraints. The right plan is a staged modular refactor that first extracts settings, scheduler, audio pipeline, and logging without changing behavior, then adds strict runtime configuration and API/UI patterns from the C6 repo, while explicitly avoiding RTSP, M5-specific hardware features, and any feature that conflicts with the current dusk/dawn sleep model.

2. **Current-state analysis**

`esp32-audio-streamer/src/main.cpp` currently owns every concern:

- Boot and lifecycle:
  - `setup()` initializes serial, retained boot/schedule state, `Preferences` namespace `sched`, timezone via `setenv("TZ", LOCAL_TZ, 1); tzset();`, Wi-Fi, initial NTP sync via `maybeSyncNtp()`, daily schedule cache via `ensureSchedule()`, and immediate sleep-if-night via `deepSleepUntil(...)`.
  - After time/sleep decisions, it initializes I2S through `initI2SMic()`, creates the ring buffer, starts `i2sProducerTask`, computes HPF coefficients, and registers HTTP routes.
  - `loop()` calls `server.handleClient()`, retries NTP, recomputes the civil twilight schedule, and re-enters deep sleep at night.

- Audio data flow:
  - `i2sProducerTask()` is the capture boundary.
  - Source: `i2s_read(...)` into 32-bit samples.
  - Transformations: right-shift by `CONVERT_SHIFT`, clamp to 16-bit.
  - Destination: `xRingbufferSend(...)` into `g_ringbuf`.
  - Threading: this runs in a FreeRTOS task pinned to core 0.
  - Mutation points: `g_ringbuf`, task handle `g_i2s_task`, I2S driver state.

- HTTP stream data flow:
  - `handleStream()` is the transport boundary.
  - Source: `xRingbufferReceive(...)` from `g_ringbuf`.
  - Transformations: optional HPF is applied here, per outgoing chunk.
  - Destination: blocking `WiFiClient.write(...)` loop until disconnect.
  - Threading: this runs inside the synchronous `WebServer` request handler on the Arduino loop task.
  - Mutation points: HPF state variables `g_hpf_prev_x_i16`, `g_hpf_prev_y_i32`, socket write loop.

- Schedule/time data flow:
  - NTP sync: `configTzTime(...)` in `maybeSyncNtp()` populates system time.
  - Solar schedule: `computeCivilTimesUTC_forDay(...)`, `computeTodayTomorrow(...)`, and `ensureSchedule(...)` compute and cache dawn/dusk in RTC-retained globals.
  - Sleep decision: `trySleepIfNight(...)` checks current time against `g_today_dawn_utc` / `g_today_dusk_utc` and calls `deepSleepUntil(...)`.
  - Persistence:
    - RTC retained state: boot count, cached dawn/dusk, last compute day, last sync/check timestamps.
    - NVS `sched` namespace: `last_wakes`, `next_sleeps`.

- HTTP API ownership:
  - `GET /` returns a generated HTML page.
  - `GET /stream` is the long-lived audio stream.
  - `GET /uptime` returns uptime-only JSON.
  - `GET /status` returns schedule/time/status JSON.
  - All routes are registered on one `WebServer server(SERVER_PORT)`.

Relevant reusable code already present in `esp32-audio-streamer`:

- Solar schedule calculation and RTC/NVS retention are already stronger than either external repo’s time-offset-only model.
- The current I2S producer task + ring buffer already gives the project the main architectural benefit the M5 repo added for capture decoupling.
- The split between `/uptime` and `/status` is already documented in `README.md` and should be preserved.

Relevant patterns to reuse from `birdnetgo-esp32-rtsp-mic`:

- Strict input validation and mutation auth from `esp32_rtsp_mic_birdnetgo/WebUI.cpp`:
  - `requireMutationAuth()`
  - `parseUInt32Strict(...)`
  - `parseInt32Strict(...)`
  - `parseFloatStrict(...)`
- Runtime settings + `Preferences` persistence pattern from `loadAudioSettings()` / `saveAudioSettings()`.
- Log ring buffer pattern from `webui_pushLog(...)`.
- Gzipped Web UI asset pipeline from `webui/index.html`, `tools/gen_webui_gzip_header.sh`, and `WebUI_gz.h`.
- Deep-sleep safety gating from `checkDeepSleepSchedule()`:
  - explicit status codes
  - boot grace
  - “never sleep without valid time”
  - observable next-sleep timing

Relevant patterns to reuse from `birdnetgo-m5stack-atom-echo-rtsp-mic`:

- Only the ownership discipline, not the hardware:
  - audio pipeline owns DSP state
  - control plane should not mutate audio state ad hoc from unrelated files
  - if transport ownership is ever refactored later, use explicit stop/cleanup handshakes like `requestStreamStop(...)`
- Optional audio metering / clipping metrics from its `httpAudioStatus()` shape

Blocking constraints in the current code:

- `handleStream()` is a blocking request handler on the same `WebServer` used for all other routes. While a client is connected to `/stream`, `server.handleClient()` is effectively occupied by that request, so new API/UI requests are not reliably serviced.
- HPF state is applied in the stream consumer, not the audio producer. That couples DSP to the network path and makes filter state transport-owned instead of pipeline-owned.
- Build flags


> 💡 Continue this plan conversation with ask_oracle(chat_id: "esp32-feature-adoption-97988D", new_chat: false)