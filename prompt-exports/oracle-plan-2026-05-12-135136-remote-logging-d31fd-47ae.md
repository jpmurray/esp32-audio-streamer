## Final Prompt
<taskname="Remote Logging"/>
<task>
Implement optional UDP/syslog remote logging and improve overall tracing/log quality for the `esp32-audio-streamer` firmware. Keep it compile-time gated and best-effort: `ENABLE_REMOTE_LOG`, `REMOTE_LOG_HOST`, `REMOTE_LOG_PORT`, `REMOTE_LOG_DEVICE`, and `REMOTE_LOG_MIN_LEVEL`; no retries, no blocking behavior beyond the minimal UDP send path, and no recursive logging from the remote sink. Centralize logging macros so existing module logs can flow through Serial, the RAM ring buffer exposed by `/api/logs`, and optional remote logging.

Also improve boot/reset diagnostics and useful stream/network/audio traces. Periodic health logging is acceptable if lightweight. Avoid heavy runtime configuration or flash persistence unless it is very small and clearly safe.
</task>

<architecture>
`esp32-audio-streamer` is an Arduino/PlatformIO ESP32 firmware. `platformio.ini` defines the main env and includes an optional untracked `local_env.ini`; `local_env.ini.example` documents compile-time build flags.

Logging is currently centered on `include/LogBuffer.h` and `src/LogBuffer.cpp`: `logbuf_init()` sets up a FreeRTOS mutex, `logbuf_printf()`/`logbuf_vprintf()` write to `Serial` and a fixed-size RAM ring, and `logbuf_jsonArray()` serializes the ring for HTTP. There is no remote sink yet.

HTTP control in `src/HttpControl.cpp` registers `/api/logs` via `handleApiLogs()`, plus `/api/status`, `/api/audio_status`, `/api/perf_status`, Wi-Fi settings, OTA actions, reboot, etc. `webui/system.html` displays `/api/logs` and system status.

`src/main.cpp` owns `setup()`/`loop()`, calls `Serial.begin()`, `logbuf_init()`, `appState_init()`, `runtimeSettings_load()`, `networkManager_begin()`, HTTP route registration, `otaManager_init()`, and starts normal services once STA is connected. It is the natural place for boot/reset diagnostics and lightweight periodic health logging.

Module logging is inconsistent:
- `HttpControl.cpp`, `NetworkManager.cpp`, and `OtaManager.cpp` already include `LogBuffer.h` and use local `LOG*` macros that call `logbuf_printf()`.
- `AudioPipeline.cpp`, `RuntimeSettings.cpp`, and `Scheduler.cpp` define local `LOG*` macros around `Serial.printf()`, so those logs bypass the RAM buffer and would bypass any remote sink.
- `StreamServer.cpp` has local logging macros and should be checked in full; it owns HTTP/RTSP session state and is a key place for richer stream traces.

The firmware is split into modules with C-style functions and shared globals in `AppState`: audio capture/ring buffer (`AudioPipeline`), streaming (`StreamServer`), Wi-Fi/setup AP (`NetworkManager`), runtime settings/NVS (`RuntimeSettings`), schedule/NTP/deep sleep (`Scheduler`), OTA upload/maintenance (`OtaManager`), HTTP API (`HttpControl`), and boot/loop (`main`).
</architecture>

<selected_context>
esp32-audio-streamer/include/LogBuffer.h: Full public logging API and compile-time buffer sizes; likely location for centralized logging macros and remote logging declarations.
esp32-audio-streamer/src/LogBuffer.cpp: Full current Serial + RAM ring implementation and JSON escaping; likely location for optional UDP/syslog sink and recursion guard.
esp32-audio-streamer/src/main.cpp: Full boot/loop flow, current top-level logging macros, HTTP route registration, service startup, NTP scheduling; good place for boot/reset diagnostics and periodic health trace.
esp32-audio-streamer/include/AppState.h + src/AppState.cpp: Shared global state (`g_boot_ms`, ring buffer/task handles, prefs state, NTP retry timestamp) used by status/logging/health code.
esp32-audio-streamer/include/AudioPipeline.h + src/AudioPipeline.cpp: Full I2S producer, audio metrics, HPF/config apply path; currently uses `Serial.printf` local `LOGD/LOGI/LOGW/LOGE` macros.
esp32-audio-streamer/include/RuntimeSettings.h + src/RuntimeSettings.cpp: Runtime/NVS settings, validation and apply methods; currently logs via `Serial.printf` local macros and explicitly notes `log_level` runtime support is deferred.
esp32-audio-streamer/include/Scheduler.h + src/Scheduler.cpp: NTP, civil dawn/dusk, deep sleep; currently uses `Serial.printf` local macros including debug schedule traces.
esp32-audio-streamer/include/NetworkManager.h + src/NetworkManager.cpp: Wi-Fi onboarding/STA/AP state and status JSON; already logs through `logbuf_printf()` with `[NET]` prefix.
esp32-audio-streamer/include/StreamServer.h + src/StreamServer.cpp: HTTP stream + RTSP/TCP server, disconnect reasons, counters, stack high-water helpers; important for stream tracing and health logs.
esp32-audio-streamer/include/HttpControl.h + src/HttpControl.cpp: HTTP API including `/api/logs`, `/api/perf_status`, `/api/audio_status`, `/api/reboot`; already logs through `logbuf_printf()` with `[HC]` prefix.
esp32-audio-streamer/include/OtaManager.h + src/OtaManager.cpp: OTA state machine and upload handling; already logs through `logbuf_printf()` and uses maintenance inhibit/audio drain.
esp32-audio-streamer/local_env.ini.example: User-facing compile-time build flags; add remote logging flag documentation here.
esp32-audio-streamer/platformio.ini: PlatformIO environment and `extra_configs = local_env.ini`; ensure new flags work as optional build flags.
esp32-audio-streamer/docs/configuration.md: Build/runtime configuration docs; currently documents `LOG_LEVEL` only.
esp32-audio-streamer/docs/api.md: HTTP API docs, including logs/status/perf endpoints.
esp32-audio-streamer/docs/architecture.md: High-level module architecture.
esp32-audio-streamer/docs/streaming.md: Stream behavior docs; useful for wording stream trace expectations.
esp32-audio-streamer/webui/system.html: System UI polls `/api/logs`, `/api/status`, `/api/perf_status`, OTA status; include if changing displayed log/status behavior.
esp32-audio-streamer/tools/gen_webui_gzip_header.sh: Regenerates `include/WebUI_gz.h` from webui HTML if `system.html` changes.
esp32-audio-streamer/README.md and CHANGELOG.md: User-facing overview and recent history; update only if project conventions suggest it.
</selected_context>

<relationships>
- `setup()` in `main.cpp` -> `Serial.begin()` -> `logbuf_init()` -> first boot logs via current `LOGI` macros.
- `logbuf_printf()` -> `Serial.print()` + RAM ring -> `HttpControl::handleApiLogs()` -> `webui/system.html` log panel.
- `NetworkManager`, `HttpControl`, `OtaManager` local `LOG*` macros -> `logbuf_printf()` already.
- `AudioPipeline`, `RuntimeSettings`, `Scheduler` local `LOG*` macros -> `Serial.printf()` currently, bypassing `LogBuffer` and any future remote sink.
- `StreamServer` owns transport session counters, disconnect reasons, and high-water marks -> `HttpControl::handleApiAudioStatus()` / `handleApiPerfStatus()` expose related status.
- `RuntimeSettings` applies Wi-Fi/audio tuning and persists user settings in NVS; do not add remote logging persistence unless deliberately choosing a small safe extension.
- `local_env.ini.example` + `docs/configuration.md` are the main surfaces for documenting compile-time remote logging flags.
</relationships>

<implementation_notes>
Keep compile-time behavior compatible when `ENABLE_REMOTE_LOG` is absent or false. Be careful with quoted string build flags in PlatformIO examples, matching existing `'-D LOCAL_TZ="UTC0"'` style where needed. Avoid using logging inside the remote-send path itself to prevent recursion. UDP/syslog should tolerate Wi-Fi not being connected and should drop silently. If adding centralized macros in `LogBuffer.h`, verify modules do not collide on existing local macro names or include order.
</implementation_notes>

<verification>
Suggested checks for the next model: build with default config; build with remote logging flags enabled; inspect that all module `Serial.printf` logging call sites are removed or routed through centralized logging; confirm `/api/logs` still returns valid JSON; confirm docs/examples mention defaults, min-level behavior, and syslog/UDP limitations.
</verification>

<ambiguities>
The requested `REMOTE_LOG_MIN_LEVEL` could be interpreted either as independent from `LOG_LEVEL` or as a remote-only threshold applied after compile-time `LOG_LEVEL` filtering. The current architecture has compile-time log elision via macros, so document and implement the chosen behavior explicitly.
</ambiguities>

## Selection
- Files: 29 total (29 full)
- Total tokens: 62993 (Auto view)
- Token breakdown: full 62993

### Files
### Selected Files
/Users/jpmurray/Code/Repositories/Perso/esp32-audio-streamer/
├── docs/
│   ├── api.md — 1,603 tokens (full)
│   ├── architecture.md — 583 tokens (full)
│   ├── configuration.md — 1,556 tokens (full)
│   └── streaming.md — 723 tokens (full)
├── include/
│   ├── AppState.h — 588 tokens (full)
│   ├── AudioPipeline.h — 697 tokens (full)
│   ├── HttpControl.h — 224 tokens (full)
│   ├── LogBuffer.h — 421 tokens (full)
│   ├── NetworkManager.h — 663 tokens (full)
│   ├── OtaManager.h — 606 tokens (full)
│   ├── RuntimeSettings.h — 1,176 tokens (full)
│   ├── Scheduler.h — 588 tokens (full)
│   └── StreamServer.h — 1,878 tokens (full)
├── src/
│   ├── AppState.cpp — 449 tokens (full)
│   ├── AudioPipeline.cpp — 3,246 tokens (full)
│   ├── HttpControl.cpp — 9,147 tokens (full)
│   ├── LogBuffer.cpp — 1,120 tokens (full)
│   ├── NetworkManager.cpp — 5,057 tokens (full)
│   ├── OtaManager.cpp — 3,093 tokens (full)
│   ├── RuntimeSettings.cpp — 2,391 tokens (full)
│   ├── Scheduler.cpp — 3,371 tokens (full)
│   ├── StreamServer.cpp — 9,974 tokens (full)
│   └── main.cpp — 5,534 tokens (full)
├── tools/
│   └── gen_webui_gzip_header.sh — 679 tokens (full)
├── webui/
│   └── system.html — 4,449 tokens (full)
├── CHANGELOG.md — 807 tokens (full)
├── README.md — 985 tokens (full)
├── local_env.ini.example — 1,205 tokens (full)
└── platformio.ini — 180 tokens (full)


---

## Generated Plan

## Chat Send ✅
- **Chat**: `remote-logging-D31FDE` | **Mode**: plan

### Response
## 1. **Summary**

Implement a targeted logging refactor centered on `LogBuffer`: add centralized level-aware logging macros, route all module logs through one sink (`Serial` + RAM ring + optional UDP/syslog), and compile-time gate remote logging with `ENABLE_REMOTE_LOG`, `REMOTE_LOG_HOST`, `REMOTE_LOG_PORT`, `REMOTE_LOG_DEVICE`, and `REMOTE_LOG_MIN_LEVEL`. Remote logging should be best-effort, non-recursive, silently dropped when Wi-Fi or destination is unavailable, and not persisted. Add lightweight boot/reset diagnostics and periodic health traces in `main.cpp`, plus a few high-value stream/network/audio logs where existing tracing is weak.

## 2. **Current-state analysis**

### Logging ownership and data flow today

- `include/LogBuffer.h` exposes:
  - `logbuf_init()`
  - `logbuf_printf()`
  - `logbuf_vprintf()`
  - `logbuf_jsonArray()`
- `src/LogBuffer.cpp` currently owns:
  - fixed RAM ring storage
  - FreeRTOS mutex
  - `Serial.print()` write path
  - JSON serialization for `/api/logs`

Current write flow:

```text
module LOG* macro
  -> either logbuf_printf() or Serial.printf()
  -> Serial and sometimes RAM ring
  -> /api/logs via HttpControl
  -> System UI log panel
```

Modules already using `logbuf_printf()`:

- `HttpControl.cpp`
- `NetworkManager.cpp`
- `OtaManager.cpp`

Modules bypassing `LogBuffer` with `Serial.printf()`:

- `main.cpp`
- `AudioPipeline.cpp`
- `RuntimeSettings.cpp`
- `Scheduler.cpp`
- `StreamServer.cpp`

This blocks remote logging because those modules would bypass any new remote sink.

### API/UI behavior

- `/api/logs` calls `logbuf_jsonArray()` and returns recent RAM logs.
- `webui/system.html` polls `/api/logs`; no UI change is required if the in-memory log format remains line-oriented JSON strings.

### Constraints

- ESP32 Arduino firmware; no heavy runtime config.
- Logging must remain compile-time gated.
- Remote logging must not recurse: the remote sink must not call `LOG*`/`logbuf_printf()`.
- UDP send must be best-effort: no retries and silent drop when STA is disconnected.
- Existing `LOG_LEVEL` currently performs compile-time elision in local macros; preserve this behavior.
- `REMOTE_LOG_MIN_LEVEL` should be a remote-only threshold applied after `LOG_LEVEL` filtering.

## 3. **Design**

### 3.1 Centralized logging API and macros

Modify `include/LogBuffer.h` to become the single logging interface.

Add a closed level enum:

```cpp
enum LogLevel : uint8_t {
    LOG_LEVEL_ERROR = 1,
    LOG_LEVEL_WARN  = 2,
    LOG_LEVEL_INFO  = 3,
    LOG_LEVEL_DEBUG = 4,
};
```

Important: this enum should not replace the existing build flag `LOG_LEVEL`. Because `LOG_LEVEL` is already used as a numeric compile-time macro, avoid naming enum values `LOG_LEVEL_*` if that creates confusion with the macro. Preferred names:

```cpp
enum LogSeverity : uint8_t {
    LOG_SEV_ERROR = 1,
    LOG_SEV_WARN  = 2,
    LOG_SEV_INFO  = 3,
    LOG_SEV_DEBUG = 4,
};
```

Add level-aware logging functions:

```cpp
void logbuf_logv(LogSeverity severity,
                 const char* module,
                 const char* fmt,
                 va_list ap);

void logbuf_log(LogSeverity severity,
                const char* module,
                const char* fmt, ...);
```

Keep existing `logbuf_printf()` and `logbuf_vprintf()` for compatibility, but update them internally to use the same sink path with severity/module marked as unknown/default.

Central macros in `LogBuffer.h`:

```cpp
#ifndef LOG_LEVEL
#define LOG_LEVEL 2
#endif

#if LOG_LEVEL >= 1
#define LOGE(module, fmt, ...) logbuf_log(LOG_SEV_ERROR, module, fmt, ##__VA_ARGS__)
#else
#define LOGE(...) do {} while (0)
#endif

#if LOG_LEVEL >= 2
#define LOGW(module, fmt, ...) logbuf_log(LOG_SEV_WARN, module, fmt, ##__VA_ARGS__)
#define LOGI(module, fmt, ...) logbuf_log(LOG_SEV_INFO, module, fmt, ##__VA_ARGS__)
#else
#define LOGW(...) do {} while (0)
#define LOGI(...) do {} while (0)
#endif

#if LOG_LEVEL >= 3
#define LOGD(module, fmt, ...) logbuf_log(LOG_SEV_DEBUG, module, fmt, ##__VA_ARGS__)
#else
#define LOGD(...) do {} while (0)
#endif
```

Use explicit module strings at call sites:

```cpp
LOGI("NET", "STA connected: SSID='%s' IP=%s RSSI=%d\n", ...);
LOGW("SS", "RTSP: client rejected (stream already active)\n");
```

This avoids per-file duplicated macros and keeps log line formatting consistent.

Final formatted local log line shape:

```text
[I][NET] STA connected: SSID='...' IP=... RSSI=-62
```

Severity mapping:

| Severity | Prefix |
|---|---|
| error | `[E]` |
| warn | `[W]` |
| info | `[I]` |
| debug | `[D]` |

### 3.2 Optional UDP/syslog remote sink

Implement in `src/LogBuffer.cpp`, owned entirely by the logging subsystem.

Compile-time defaults in `LogBuffer.cpp` or `LogBuffer.h`:

```cpp
#ifndef ENABLE_REMOTE_LOG
#define ENABLE_REMOTE_LOG 0
#endif

#ifndef REMOTE_LOG_HOST
#define REMOTE_LOG_HOST ""
#endif

#ifndef REMOTE_LOG_PORT
#define REMOTE_LOG_PORT 514
#endif

#ifndef REMOTE_LOG_DEVICE
#define REMOTE_LOG_DEVICE "esp32-audio-streamer"
#endif

#ifndef REMOTE_LOG_MIN_LEVEL
#define REMOTE_LOG_MIN_LEVEL 1
#endif
```

Chosen interpretation:

- `LOG_LEVEL` controls whether a log call exists at compile time.
- `REMOTE_LOG_MIN_LEVEL` controls which already-emitted logs are forwarded remotely.
- Level ordering: `1=error`, `2=warn`, `3=info`, `4=debug`.
- Remote sends if `severity <= REMOTE_LOG_MIN_LEVEL` is **not** intuitive for this scale. Instead use a threshold helper with current `LOG_LEVEL` semantics:
  - If `REMOTE_LOG_MIN_LEVEL=1`, send errors only.
  - If `REMOTE_LOG_MIN_LEVEL=2`, send errors/warnings.
  - If `REMOTE_LOG_MIN_LEVEL=3`, send errors/warnings/info.
  - If `REMOTE_LOG_MIN_LEVEL=4`, send all including debug.

Since current public `LOG_LEVEL` uses `1=errors`, `2=info+warn+error`, `3=debug`, document remote level as:

| Value | Remote output |
|---:|---|
| `1` | errors |
| `2` | errors, warnings |
| `3` | errors, warnings, info |
| `4` | errors, warnings, info, debug |

To preserve existing `LOG_LEVEL=3` behavior, map debug logs to remote min level `4`, while compile-time debug remains `LOG_LEVEL >= 3`.

Remote sink constraints:

- Use `WiFi.status() == WL_CONNECTED`; otherwise drop.
- Do not log failures from remote send.
- Do not retry.
- Guard against recursion with a static boolean:

```cpp
static bool s_remote_sending = false;
```

- To avoid DNS blocking, require `REMOTE_LOG_HOST` to be an IPv4 literal. Parse once in `logbuf_init()` using `IPAddress::fromString()`.
- If parsing fails, disable remote sink silently.

Remote state shape:

```cpp
#if ENABLE_REMOTE_LOG
static WiFiUDP s_remote_udp;
static IPAddress s_remote_ip;
static bool s_remote_config_ok = false;
static bool s_remote_sending = false;
#endif
```

Syslog payload shape: lightweight RFC3164-style line, no timestamp dependency:

```text
<PRI>REMOTE_LOG_DEVICE module[uptime_ms]: message
```

Suggested PRI mapping using facility `local0` (`16 * 8 = 128`):

| Severity | Syslog severity | PRI |
|---|---:|---:|
| error | 3 | 131 |
| warn | 4 | 132 |
| info | 6 | 134 |
| debug | 7 | 135 |

Do not attempt full RFC5424 formatting because time may be invalid during boot and building structured data increases complexity.

Remote send pseudocode:

```cpp
if (!ENABLE_REMOTE_LOG) return;
if (!s_remote_config_ok) return;
if (s_remote_sending) return;
if (WiFi.status() != WL_CONNECTED) return;
if (!severityAllowedRemote(severity)) return;

s_remote_sending = true;
s_remote_udp.beginPacket(s_remote_ip, REMOTE_LOG_PORT);
s_remote_udp.write((const uint8_t*)payload, strlen(payload));
s_remote_udp.endPacket();
s_remote_sending = false;
```

### 3.3 LogBuffer write path changes

`logbuf_logv()` should:

1. Format caller message into a fixed local buffer.
2. Prefix severity/module for local RAM/Serial.
3. Write prefixed line to Serial.
4. Append prefixed line to RAM ring under mutex.
5. Send syslog payload remotely, if enabled.

Avoid holding the RAM ring mutex while doing UDP send.

Recommended buffer limits:

- Keep existing `LOG_BUF_LINE_LEN` for RAM line length.
- Use a separate syslog payload buffer sized around 256 bytes.
- Truncate safely; never allocate heap in the write path.

Data flow:

```text
LOGI("AP", "profile=%s sample_rate=%d\n", ...)
  -> logbuf_log()
  -> logbuf_logv()
  -> local line "[I][AP] profile=..."
  -> Serial.print()
  -> RAM ring
  -> optional UDP syslog payload
```

### 3.4 Module macro migration

Remove all file-local `LOGE/LOGW/LOGI/LOGD` definitions and include `LogBuffer.h` where needed.

Call sites update pattern:

Before:

```cpp
LOGI("[I2S] init ok: %d Hz\n", sample_rate_hz);
```

After:

```cpp
LOGI("AP", "[I2S] init ok: %d Hz\n", sample_rate_hz);
```

Preferred modules:

| File | Module tag |
|---|---|
| `main.cpp` | `MAIN` |
| `AudioPipeline.cpp` | `AP` |
| `RuntimeSettings.cpp` | `RT` |
| `Scheduler.cpp` | `SCHED` |
| `StreamServer.cpp` | `SS` |
| `NetworkManager.cpp` | `NET` |
| `HttpControl.cpp` | `HC` |
| `OtaManager.cpp` | `OTA` |
| `LogBuffer.cpp` internal diagnostics | none; do not self-log remote errors |

### 3.5 Boot/reset diagnostics

Add in `main.cpp` immediately after `logbuf_init()` and first boot line.

Log:

- firmware/project boot marker
- reset reason for CPU0 and CPU1 using ESP-IDF reset reason API
- chip model/revision/cores
- flash size if available
- compile-time logging/remote config summary
- partition/OTA support summary using `ESP.getFreeSketchSpace()`
- heap initial free/min/total
- configured pins/sample flags where useful

Required includes:

```cpp
#include "esp_system.h"
```

Possible helper functions in `main.cpp`:

```cpp
static const char* resetReasonName(esp_reset_reason_t r);
static void logBootDiagnostics();
```

Do not persist anything.

Example log intent:

```text
[I][MAIN] Booting ESP32 Audio Streamer
[I][MAIN] Reset reason: cpu0=POWERON cpu1=POWERON
[I][MAIN] Chip: ESP32 rev=... cores=2 flash=4194304
[I][MAIN] Build: LOG_LEVEL=2 remote_log=off
[I][MAIN] Heap: free=... min=... total=...
[I][MAIN] OTA slot free=...
```

### 3.6 Periodic health logging

Add lightweight periodic health in `main.cpp::loop()`.

Compile-time tunable:

```cpp
#ifndef HEALTH_LOG_INTERVAL_MS
#define HEALTH_LOG_INTERVAL_MS 60000
#endif
```

State:

```cpp
static uint32_t s_next_health_log_ms = 0;
```

In `loop()`:

```cpp
if (HEALTH_LOG_INTERVAL_MS > 0 && millisReached(s_next_health_log_ms)) {
    s_next_health_log_ms = now_ms + HEALTH_LOG_INTERVAL_MS;
    logHealthSnapshot();
}
```

Health snapshot should be info-level and compact:

- uptime seconds
- heap free/min
- STA connected/connecting/setup AP active
- RSSI if connected
- stream active transport
- audio `rb_drop_count`, `i2s_error_count`
- stream timeout/write stall counters
- OTA phase if not idle or failed

Do not allocate heap. Use existing APIs:

- `networkManager_getStatus()`
- `audioPipeline_getMetrics()`
- stream globals
- `ESP.getFreeHeap()`
- `ESP.getMinFreeHeap()`

Make health logging disable-able with:

```ini
-D HEALTH_LOG_INTERVAL_MS=0
```

### 3.7 Additional trace improvements

Keep this targeted; do not add noisy logs inside per-sample/per-chunk loops.

#### `AudioPipeline.cpp`

Add or preserve logs for:

- init sample rate/profile/convert shift/ring buffer capacity
- ring buffer fallback capacity selection
- stop teardown summary
- producer allocation failure already exists
- first ring-buffer drop warning should be rate-limited

Suggested module-private state:

```cpp
static volatile uint32_t s_last_rb_drop_log_ms = 0;
```

In producer task, when `xRingbufferSend` fails, increment counter and log only every 30s or first drop. Because producer task can be hot, use minimal conditional.

#### `StreamServer.cpp`

Add useful traces:

- HTTP session start: remote IP, format, sample rate
- RTSP session start already logs client IP
- session cleanup already logs reason/bytes/duration; ensure routed through centralized logging
- stream stop request start/end in `streamServer_requestStopAndWait()`, with timeout result
- single-client rejection: already present for RTSP; add equivalent for HTTP if applicable or document that HTTP `WebServer` path blocks in handler and currently relies on active gate behavior.

Important: Do not log every audio chunk or RTP packet.

#### `NetworkManager.cpp`

Already good. Add only:

- setup AP start includes channel/security mode
- reconnect scheduling and timeout already logged
- credential save should continue not logging password

#### `Scheduler.cpp`

Route existing logs through centralized logger. Keep debug solar traces at debug only.

#### `RuntimeSettings.cpp`

Route through centralized logger. No runtime persistence for remote logging.

## 4. **File-by-file impact**

### `include/LogBuffer.h`

Changes:

- Add `LogSeverity` enum.
- Add `logbuf_log()` and `logbuf_logv()`.
- Add centralized `LOGE/LOGW/LOGI/LOGD` macros.
- Document `LOG_LEVEL` and remote logging compile-time flags.
- Keep existing `logbuf_printf()` APIs for compatibility.

Why:

- Centralizes logging so all modules flow through Serial, RAM ring, and optional remote sink.

Dependencies:

- Must land before migrating module call sites.

---

### `src/LogBuffer.cpp`

Changes:

- Implement level-aware formatting.
- Preserve existing RAM ring behavior and JSON escaping.
- Add optional UDP/syslog sink under `#if ENABLE_REMOTE_LOG`.
- Add IPv4 literal parsing of `REMOTE_LOG_HOST` in `logbuf_init()`.
- Add recursion guard around remote send.
- Make `logbuf_printf()` delegate to the shared sink path.

Why:

- Single implementation point for all sinks.

Dependencies:

- Requires `WiFi.h` and `WiFiUdp.h` includes when remote logging is enabled.

---

### `src/main.cpp`

Changes:

- Remove local `LOG*` macro definitions.
- Include/use centralized `LogBuffer.h` macros.
- Add reset-reason helper and boot diagnostics.
- Add periodic health logging helper and loop timer.
- Log normal service start, AP setup URL, and error states using `LOGI("MAIN", ...)`.

Why:

- Main owns boot lifecycle and periodic loop context.

Dependencies:

- Uses `audioPipeline_getMetrics()`, `networkManager_getStatus()`, stream globals, OTA query functions.

---

### `src/AudioPipeline.cpp`

Changes:

- Remove local `Serial.printf` logging macros.
- Route logs through `LOG*("AP", ...)`.
- Add rate-limited ring-buffer drop warning.
- Add teardown summary in `audioPipeline_stop()`.

Why:

- Audio logs currently bypass RAM/remote sinks.

Dependencies:

- Include `LogBuffer.h`.

---

### `src/RuntimeSettings.cpp`

Changes:

- Remove local logging macros.
- Route logs through `LOG*("RT", ...)`.

Why:

- Runtime setting logs currently bypass RAM/remote sinks.

Dependencies:

- Include `LogBuffer.h`.

---

### `src/Scheduler.cpp`

Changes:

- Remove local logging macros.
- Route logs through `LOG*("SCHED", ...)`.
- Replace debug `Serial.printf()` schedule traces with centralized debug logs.

Why:

- Scheduler/NTP/sleep logs are important for remote diagnostics.

Dependencies:

- Include `LogBuffer.h`.

---

### `src/StreamServer.cpp`

Changes:

- Remove local logging macros.
- Route all logs through `LOG*("SS", ...)`.
- Add targeted session start and stop-request logs.
- Ensure no logs are added inside tight RTP/audio send loops except existing terminal conditions.

Why:

- Stream diagnostics are key for remote troubleshooting.

Dependencies:

- Include `LogBuffer.h`.

---

### `src/NetworkManager.cpp`

Changes:

- Remove local duplicated macros.
- Route existing logs through centralized macros with module `NET`.
- Optionally enrich setup AP log with channel/security.

Why:

- Already uses `logbuf_printf()`, but should use central macros for consistency and remote severity filtering.

Dependencies:

- Existing `LogBuffer.h` include remains.

---

### `src/HttpControl.cpp`

Changes:

- Remove local duplicated macros.
- Route existing logs through centralized macros with module `HC`.
- No API behavior change.

Why:

- Consistency and centralized severity handling.

Dependencies:

- Existing `LogBuffer.h` include remains.

---

### `src/OtaManager.cpp`

Changes:

- Remove local duplicated macros.
- Route existing logs through centralized macros with module `OTA`.
- Preserve no-recursive logging guarantee by not logging from `LogBuffer.cpp` remote path.

Why:

- OTA logs already flow through RAM; update for severity consistency.

Dependencies:

- Existing `LogBuffer.h` include remains.

---

### `local_env.ini.example`

Changes:

Add documented optional build flags:

```ini
; Remote UDP/syslog logging (optional, compile-time only)
; -D ENABLE_REMOTE_LOG=1
; '-D REMOTE_LOG_HOST="192.168.1.10"'
; -D REMOTE_LOG_PORT=514
; '-D REMOTE_LOG_DEVICE="esp32-audio-streamer"'
; -D REMOTE_LOG_MIN_LEVEL=2
; -D HEALTH_LOG_INTERVAL_MS=60000
```

Note:

- `REMOTE_LOG_HOST` is an IPv4 literal to avoid DNS blocking.
- `REMOTE_LOG_MIN_LEVEL`: `1=errors`, `2=warnings+errors`, `3=info+warn+error`, `4=debug+...`.
- Remote logs are best-effort UDP, dropped when Wi-Fi is down.

---

### `docs/configuration.md`

Changes:

- Expand logging section.
- Document remote logging flags, defaults, limitations, and examples.
- Document `HEALTH_LOG_INTERVAL_MS`.

Why:

- User-facing compile-time config reference.

---

### `docs/architecture.md`

Changes:

- Update `LogBuffer` responsibility to include centralized macros and optional remote UDP/syslog sink.
- Update architecture flow:

```text
LOG* macros -> LogBuffer -> Serial + RAM ring + optional UDP/syslog
```

---

### `docs/api.md`

Changes:

- Clarify `/api/logs` remains the RAM ring only.
- Mention remote logging does not change API responses.

---

### `CHANGELOG.md`

Changes:

Under `Unreleased`:

- Added optional UDP/syslog remote logging.
- Changed module logging to centralized sink.
- Added boot/reset and periodic health diagnostics.

---

### `webui/system.html`

No required change.

Reason:

- `/api/logs` response shape remains unchanged.

---

### `tools/gen_webui_gzip_header.sh`

No required change.

Reason:

- No Web UI source change planned.

## 5. **Risks and migration**

### Compile-time macro collision

Risk: Existing files define local `LOGI`, `LOGW`, etc. If `LogBuffer.h` also defines them, duplicate macro definitions can cause warnings or unexpected behavior.

Mitigation:

- Remove local logging macro blocks from each migrated `.cpp`.
- Include `LogBuffer.h` after Arduino/system includes is acceptable; macros are independent.

### Remote host DNS blocking

Risk: Arduino UDP hostname resolution can block.

Decision:

- Treat `REMOTE_LOG_HOST` as an IPv4 literal.
- Parse with `IPAddress::fromString()` once in `logbuf_init()`.
- If invalid, remote logging is silently disabled.

### Log format compatibility

Risk: Existing UI regex recognizes `[E]`, `[W]`, `[I]`, `[D]`.

Mitigation:

- Preserve severity prefixes exactly at the start of each RAM log line, e.g. `[I][MAIN] ...`.

### Increased log volume

Risk: Periodic health logs may fill the 64-line RAM ring.

Mitigation:

- Default interval should be 60 seconds.
- Allow `-D HEALTH_LOG_INTERVAL_MS=0` to disable.
- Keep health line compact.

## 6. **Implementation order**

1. **Extend `LogBuffer` API**
   - Add severity enum, level-aware functions, and centralized macros.
   - Keep compatibility wrappers.
   - Build should still pass before call-site migration.

2. **Implement remote UDP/syslog sink**
   - Add compile-time flags/defaults.
   - Add IPv4 parsing in `logbuf_init()`.
   - Add best-effort send path with recursion guard.
   - Verify default build has no remote dependency behavior.

3. **Migrate logging macros file-by-file**
   - Start with `main.cpp`, `AudioPipeline.cpp`, `RuntimeSettings.cpp`, `Scheduler.cpp`, `StreamServer.cpp`.
   - Then simplify already-buffered modules: `NetworkManager.cpp`, `HttpControl.cpp`, `OtaManager.cpp`.
   - This should be one atomic compile fix because macro signatures change.

4. **Add boot/reset diagnostics**
   - Add helpers in `main.cpp`.
   - Log immediately after `logbuf_init()`.

5. **Add periodic health logging**
   - Add `HEALTH_LOG_INTERVAL_MS` default.
   - Add `logHealthSnapshot()` and loop timer.
   - Keep line compact and non-allocating.

6. **Add targeted trace improvements**
   - Stream session start/stop-request logs.
   - Audio ring-buffer drop rate-limited warning.
   - Setup AP detail log.

7. **Update docs and examples**
   - `local_env.ini.example`
   - `docs/configuration.md`
   - `docs/architecture.md`
   - `docs/api.md`
   - `CHANGELOG.md`

8. **Verification**
   - Build default config.
   - Build with:
     ```ini
     -D ENABLE_REMOTE_LOG=1
     '-D REMOTE_LOG_HOST="192.168.1.10"'
     -D REMOTE_LOG_PORT=514
     -D REMOTE_LOG_MIN_LEVEL=3
     ```
   - Confirm `/api/logs` JSON still parses.
   - Confirm no remaining `Serial.printf` logging paths except intentional raw serial operations.


> 💡 Continue this plan conversation with ask_oracle(chat_id: "remote-logging-D31FDE", new_chat: false)