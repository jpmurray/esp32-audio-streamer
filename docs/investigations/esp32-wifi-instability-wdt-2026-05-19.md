# Investigation: ESP32 WiFi Instability and Task WDT Resets

## Summary
The instability is best explained as a compound WiFi/resource-pressure failure, not one isolated code bug. The primary trigger is likely marginal WiFi around `-79` to `-75 dBm`, possibly with hidden power dips because brownout is disabled; application-level amplifiers include synchronous UDP remote logging, always-on audio production with no client, three priority-5 tasks on core 0, and slow TCP write-stall teardown.

## Symptoms
- Previous reset reason: `cpu0=TASK_WDT`.
- STA connects but RSSI is weak: `-79` to `-75` dBm.
- Repeated UDP send failures: `WiFiUdp.cpp:185 endPacket(): could not send data: 12`.
- Audio ring buffer fills and drops continuously when `stream=none`.
- Heap drops from boot free heap ~288 KB to health free heap ~81 KB after services start, then min heap trends down to ~50 KB by 182s.
- Brownout detector is disabled by workaround, so power dips may be hidden instead of surfaced as brownout resets.
- Later logs show repeated `WiFiClient.cpp:429 write(): fail on fd 52, errno: 11, "No more processes"` roughly once per second after ~217s uptime.

## Background / Prior Research
- ESP32/lwIP UDP `errno=12` is `ENOMEM` and is documented by Espressif as a possible result of repeated UDP sends when lower-layer WiFi driver transmit buffers are full. Practical causes include weak signal/congestion, too-high send rate, unreachable destinations, and insufficient TX buffers. Reference: ESP-IDF lwIP guide, UDP ENOMEM limitation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/lwip.html
- ESP32 Task Watchdog resets usually mean a watched task or watched idle task did not get CPU time to reset/feed the TWDT. External research emphasizes long/spinning loops without `vTaskDelay()`/yield, high-priority tasks on core 0, and blocking network operations as common causes. Reference: ESP-IDF Watchdog docs: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/wdts.html
- Weak RSSI around -79 dBm can increase retransmissions and WiFi airtime; higher retransmission load can consume TX buffers and increase current spikes. With brownout detection disabled, supply dips may present as WDT resets or undefined instability rather than brownout reset logs. References: ESP-IDF WiFi station/performance docs: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi-driver/station-scenarios.html and https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi-driver/wifi-performance-and-power-save.html

## Investigator Findings

> Investigation completed 2026-05-19. Read-only; source code unchanged.

---

### 1. UDP Remote Logging — CONFIRMED as early-session stressor, self-healing after 3 failures

**File:** `src/LogBuffer.cpp`

**Mechanism:** `remoteLog()` (LogBuffer.cpp:81) is called synchronously in the caller's task context on **every** log call — no separate logging task, no queue, no rate limiting before the first 3 failures. It uses a persistent static `WiFiUdp` object (`s_remote_udp`, LogBuffer.cpp:40) that calls `beginPacket()` + `write()` + `endPacket()` inline.

**Failure handling:** After 3 consecutive `endPacket()` failures (errno 12 = ENOMEM / TX buffer full), `s_remote_config_ok` is set `false` (LogBuffer.cpp:102–113), permanently disabling UDP for the session. There is no per-errno distinction, no exponential backoff, and no re-enable on WiFi reconnect.

**Verdict:**
- The 3 consecutive UDP failures are confirmed as the source of `WiFiUdp.cpp:185 endPacket(): could not send data: 12` log lines.
- Each failure exercises the lwIP UDP TX path against a congested/weak link, consuming TX buffer resources during the failure window.
- Once the 3-failure threshold trips, UDP stops — so this is a **transient early-boot stressor**, not an ongoing one. It does not cause WDT resets directly, but contributes to TX buffer pressure during boot.
- **The UDP sink is never re-enabled after WiFi reconnect** — the session loses remote logging permanently after the first transient failure cluster (LogBuffer.cpp:108: `s_remote_config_ok = false`). This is documented as intentional but means a flaky boot silently drops all subsequent remote logs.

**Eliminated:** UDP logging as a sustained/ongoing cause. It self-disables.

---

### 2. Audio Producer — CONFIRMED continuously runs with no stream-active gate

**File:** `src/AudioPipeline.cpp`

**Mechanism:** `i2sProducerTask` (AudioPipeline.cpp:110) runs an unconditional `for (;;)` loop. It calls `i2s_read()` with `portMAX_DELAY` (blocks on hardware), converts samples, then attempts `xRingbufferSend(..., timeout=0)` (AudioPipeline.cpp:165 — non-blocking). There is **no check for active clients or stream mode**.

**When ring buffer is full (stream=none):** `xRingbufferSend` returns `pdFALSE` immediately; `s_rb_drop_count` is incremented (AudioPipeline.cpp:166); a rate-limited LOGW fires at most once per 30s (AudioPipeline.cpp:168–175). No `vTaskDelay` is inserted on drop — the task immediately re-enters `i2s_read()` and blocks on hardware.

**Task config:** Core 0, priority 5 (AudioPipeline.cpp:256–258 via `xTaskCreatePinnedToCore`).

**Verdict:**
- Audio drops when `stream=none` are confirmed and expected by design — the producer is intentionally always-on.
- The ring buffer fills completely when no consumer is draining it; every produced chunk is immediately dropped, causing the logged drop warnings.
- Because `i2s_read` blocks on hardware DMA completion (not spinning), this does **not** starve the idle task via busy-loop. However it does keep a priority-5 task on core 0 perpetually runnable.
- **Heap impact:** Each `xRingbufferSend` with a full buffer returns fast — no heap churn from drops. The producer's two malloc'd buffers (in32/out16, ~frames_per_chunk * 6 bytes each) are allocated once at task start and never freed during normal operation.

**Eliminated:** Audio producer as a WDT starvation source via busy-loop. The hardware DMA block provides natural yield.

---

### 3. Stream/RTSP Write Failure Handling — CONFIRMED stale client can persist for ~500ms; errno 11 not inspected

**File:** `src/StreamServer.cpp`

**Mechanism:**
- HTTP streaming write loop (StreamServer.cpp:307): on `client.write()` returning 0, increments `stall_count` and calls `delay(1)`. Client is closed only after **500 consecutive zero-byte writes** (STREAM_WRITE_STALL_LIMIT, StreamServer.cpp:82–83, checked at StreamServer.cpp:312).
- RTSP write helper `rtsp_writeAll` (StreamServer.cpp:411): same pattern — `delay(1)` per zero-byte write, returns false after 500 stalls (RTSP_WRITE_STALL_LIMIT, StreamServer.cpp:67–68).
- **errno 11 (EAGAIN) is never inspected.** The code only checks the return value of `client.write()` for zero.

**Persistence window:** A half-open or slow client can hold the write loop for up to **500 × 1ms = ~500ms** per audio chunk before being disconnected. During this time, `g_stream_active` (or equivalent gate) is held, and `delay(1)` calls yield to the scheduler.

**Task config:** `stream_srv` — core 0, priority 5 (StreamServer.cpp:41–46, created at StreamServer.cpp:908–916). `rtsp_srv` — core 0, priority 5 (StreamServer.cpp:53–57, created at StreamServer.cpp:926–934).

**Verdict:**
- A WiFi-stressed client experiencing repeated EAGAIN (write buffer full) causes the stream task to burn up to 500ms in `delay(1)` loops per chunk before closing the connection.
- The `WiFiClient.cpp:429 write(): fail on fd 52, errno: 11` logs confirm this path is active — writes are hitting EAGAIN (no lwIP socket buffer space), and the stall loop runs 500 iterations before the client is terminated.
- After `client.stop()` (StreamServer.cpp:319), the fd is cleaned up correctly — there is no confirmed fd leak, but the 500-iteration drain is a prolonged hold.
- This is a **real secondary stressor**: while the stream task is stall-looping, it occupies core 0 at priority 5 alongside the audio producer. Both yield via `delay(1)` / `yield()` which does provide scheduling gaps, so WDT is unlikely to trigger from this alone.

**Eliminated:** Persistent fd/socket leak after write failure. The 500-stall limit does close the client.

---

### 4. Core-0 Task Contention — CONFIRMED three priority-5 tasks on core 0; WiFi stack also on core 0

**Files:** `src/AudioPipeline.cpp:256`, `src/StreamServer.cpp:45–46`, `src/StreamServer.cpp:56–57`

**Tasks on core 0, all at priority 5:**
| Task | Core | Priority | Source |
|------|------|----------|--------|
| `i2s_producer` | 0 | 5 | AudioPipeline.cpp:256–258 |
| `stream_srv` | 0 | 5 | StreamServer.cpp:45–46, 908–916 |
| `rtsp_srv` | 0 | 5 | StreamServer.cpp:56–57, 926–934 |

The ESP32 WiFi stack (`wifi` task) also runs on core 0 at priority 23 (ESP-IDF default). The idle task on core 0 runs at priority 0.

**Main loop** runs on core 1 (Arduino default) with `delay(2)` at the bottom (main.cpp:597). The main loop calls `server.handleClient()` and `networkManager_loop()` every iteration — both can block briefly.

**Verdict:**
- Three equal-priority tasks on core 0 compete with each other and with WiFi internals. FreeRTOS round-robins equal-priority tasks, so each gets time-sliced.
- With `i2s_producer` blocking on `i2s_read` (hardware DMA), it yields while waiting for hardware — this is not a busy-loop starvation source.
- **When a stream write stall occurs**, `stream_srv` calls `delay(1)` 500 times per chunk — each `delay(1)` yields, so core 0 idle task does get scheduled in between. This is not a clean WDT starvation path.
- **The most plausible WDT path**: if the WiFi stack on core 0 is itself delayed (due to TX buffer exhaustion from weak RSSI / high retransmissions at -79 dBm), the `esp_timer` or `wifi` task on core 0 may not complete its watchdog feed. This is an **lwIP/WiFi driver interaction**, not application-level starvation.

---

### 5. Brownout Disabled — CONFIRMED; power cofactor unobservable

**File:** `src/main.cpp:169` (`disableBrownout()` function), called at `main.cpp:543–548` when `ENABLE_BROWNOUT_DISABLE=1` (default).

**Impact:** Current spikes from WiFi retransmissions (common at -79 dBm RSSI) cause supply dips. With brownout disabled, these present as undefined resets (including WDT if the CPU stalls during the dip) rather than clean brownout-reset logs. This makes the true reset cause undiagnosable from logs alone.

---

### 6. Scheduler / NTP Cadence — NOT a stressor

**File:** `src/Scheduler.cpp`, `src/main.cpp:583–587`

NTP sync is attempted at most once per 60 seconds from the main loop, and only if time is invalid or >24h since last sync (Scheduler.cpp:195–206). Health snapshots log every 60s (main.cpp:597–603). These are well-throttled and not sources of CPU pressure.

---

### Summary: Eliminated Hypotheses

| Hypothesis | Status |
|---|---|
| UDP logging as ongoing stressor | ELIMINATED — self-disables after 3 failures |
| Audio producer busy-loop starving idle task | ELIMINATED — hardware DMA block yields naturally |
| Persistent fd/socket leak after write failure | ELIMINATED — 500-stall limit closes client |
| Scheduler/NTP as CPU pressure source | ELIMINATED — well throttled |

---

### Likely Root Cause

The `cpu0=TASK_WDT` reset is most likely caused by the **WiFi driver task on core 0 failing to feed the TWDT** under sustained TX buffer exhaustion from weak signal (-79 dBm). The application-level contribution is:

1. **Boot-time UDP logging burst** hits a congested TX path, consuming TX buffers during the window when WiFi is most stressed (initial association + logging + NTP + HTTP).
2. **Three priority-5 tasks on core 0** (audio producer, stream_srv, rtsp_srv) compete with WiFi internals, reducing scheduling headroom for the WiFi task.
3. **Brownout disabled**: actual power dips during WiFi retransmission bursts are invisible and may manifest as CPU stalls that look like WDT timeouts.
4. **Write stall loop** (errno 11 / EAGAIN): once a stream client is connected over weak WiFi, the 500-stall × delay(1) drain holds `stream_srv` active for extended periods, keeping core 0 busy during WiFi stress windows.

None of these are individually decisive — the instability is a **compounding effect** of weak signal + TX buffer pressure + core-0 task density + hidden power dips.

---

### Recommended Fix Locations

| Priority | Fix | File:Line |
|---|---|---|
| **High** | Add WiFi RSSI threshold gate: warn/reconnect below -75 dBm | `src/NetworkManager.cpp` (RSSI read at ~line 427) |
| **High** | Re-enable brownout detector or use softer threshold (not fully disabled) | `src/main.cpp:543–548` |
| **Medium** | Gate audio producer on stream-active flag; stop producing when `stream=none` to reduce core-0 load and ring-buffer churn | `src/AudioPipeline.cpp:110` (loop entry) |
| **Medium** | Reduce write stall limit (500→50) or add exponential backoff for EAGAIN; close client faster on repeated errno 11 | `src/StreamServer.cpp:82–83`, `67–68` |
| **Medium** | Move `i2s_producer` or `stream_srv`/`rtsp_srv` to core 1 to reduce core-0 competition with WiFi stack | `src/AudioPipeline.cpp:258`, `src/StreamServer.cpp:46,57` |
| **Low** | Re-enable UDP remote logging after WiFi reconnect (currently permanently disabled after first 3 failures) | `src/LogBuffer.cpp:108` |
| **Low** | Increase `delay(2)` in main loop to `delay(10)` to give core-1 idle task more headroom | `src/main.cpp:597` |

## Investigation Log

### Phase 1 - Initial Assessment
**Hypothesis:** Instability may be due to a combination of weak WiFi, UDP logging/NTP failures, task starvation/watchdog behavior, audio production without consumers, heap pressure, or hidden power/brownout issues.
**Findings:** Initial report created from user-provided boot log.
**Evidence:** User log excerpt in chat; report path: `/Users/jpmurray/Code/Repositories/Perso/esp32-audio-streamer/docs/investigations/esp32-wifi-instability-wdt-2026-05-19.md`.
**Conclusion:** Needs external ESP32 facts plus workspace investigation.

## Root Cause

The `cpu0=TASK_WDT` reset is most likely caused by weak-WiFi TX-buffer/resource pressure, possibly compounded by hidden power dips, with application-level amplification from synchronous UDP logging, core-0 task concentration, continuous no-consumer audio production, and slow TCP write-stall teardown.

Evidence:
- Weak WiFi is present in the failing boot (`RSSI=-79` initially, later `-75`). External ESP-IDF documentation identifies UDP `errno=12` as `ENOMEM`/TX-buffer exhaustion when lower-layer transmit buffers are full.
- `src/LogBuffer.cpp:77-113` sends eligible logs synchronously via `WiFiUDP.beginPacket()` / `write()` / `endPacket()` in the caller context. This explains the early `WiFiUdp.cpp:185 endPacket(): could not send data: 12` errors. The current code self-disables remote UDP logging after 3 consecutive failures, so it is an early-session stressor rather than a sustained sender.
- `src/AudioPipeline.cpp:110-175` continuously reads I2S and sends to the ring buffer even when no client is active. This directly explains `stream=none` with rising ring-buffer drops. Because `i2s_read(..., portMAX_DELAY)` blocks on DMA, this is not a direct busy-loop WDT cause, but it is unnecessary core-0/background load.
- `src/AudioPipeline.cpp:253-258`, `src/StreamServer.cpp:40-57`, and `src/StreamServer.cpp:907-934` place `i2s_producer`, `stream_srv`, and `rtsp_srv` on core 0 at priority 5, sharing the core with WiFi/lwIP activity.
- `src/StreamServer.cpp:67-83`, `src/StreamServer.cpp:307-319`, and `src/StreamServer.cpp:405-420` retry zero-byte writes up to 500 times with `delay(1)` and do not inspect `errno`. The later `WiFiClient.cpp:429 write(): errno=11` logs are consistent with TCP/lwIP backpressure under weak WiFi. Cleanup eventually calls `client.stop()`, so a persistent fd leak is not proven.
- `src/main.cpp:518-548` disables the brownout detector when `ENABLE_BROWNOUT_DISABLE=1`; the boot log confirms this workaround was active. That makes power dips during WiFi retransmission/current spikes invisible in reset diagnostics.

Eliminated or lower-probability hypotheses:
- Remote UDP logging as a sustained root cause: mostly eliminated because it self-disables after 3 consecutive failures in current code.
- Audio producer busy-loop starvation: eliminated; I2S read blocks with `portMAX_DELAY`.
- Ring-buffer drops as a heap leak: eliminated; drops are expected with no consumer and non-blocking `xRingbufferSend(..., 0)`.
- Persistent TCP fd/socket leak: not proven; stall limits eventually call `client.stop()`.
- NTP as sustained UDP source: low probability; NTP is startup/retry cadence, not high-frequency.
- OTA as active cause: low probability unless OTA was active in the failing window.
- Min-heap trend as proof of leak: not proven; `min_free` is a low-water mark and does not recover by design.

## Recommendations

1. **Re-enable brownout diagnostics and validate power first** — `src/main.cpp:518-548`, `local_env.ini.example`, `docs/configuration.md`.
   - Test with `ENABLE_BROWNOUT_DISABLE=0` and a known-good supply/cable.
   - If resets become brownout resets, power delivery is a major cause.
2. **Add weak-RSSI guardrails and operator feedback** — `src/NetworkManager.cpp`, `src/HttpControl.cpp`, `docs/configuration.md`.
   - Warn below about `-70 dBm`; treat `<= -75 dBm` as unstable for streaming.
   - Surface warnings in status/perf endpoints and UI.
3. **Harden remote UDP logging** — `src/LogBuffer.cpp:77-113`.
   - Keep self-disable, but expose a local flag/log when remote logging disables.
   - Skip or back off remote UDP logging when RSSI is weak, heap is low, or recent failures occurred.
   - Consider moving remote logging to a low-priority async queue instead of sending in the caller path.
4. **Reduce no-client audio churn** — `src/AudioPipeline.cpp:110-175`.
   - Gate ring-buffer production on active stream/client state, or discard silently without filling the ring buffer when `stream=none`.
5. **Close stalled TCP clients faster** — `src/StreamServer.cpp:67-83`, `src/StreamServer.cpp:307-319`, `src/StreamServer.cpp:405-420`.
   - Lower `STREAM_WRITE_STALL_LIMIT` / `RTSP_WRITE_STALL_LIMIT` from 500, or use faster close/backoff for repeated EAGAIN-style failures.
   - Reset per-session diagnostic state at session start.
6. **Reduce core-0 contention** — `src/AudioPipeline.cpp:253-258`, `src/StreamServer.cpp:40-57`, `src/StreamServer.cpp:907-934`.
   - Move one lower-priority application task off core 0 or lower priority.
   - Disable RTSP with `RTSP_PORT=0` if unused.
   - Replace RTSP connected-but-not-playing `yield()` loops with real `vTaskDelay()` where applicable.

Validation order:
1. Build with remote logging disabled and verify UDP `errno=12` disappears.
2. Move the board/AP to get RSSI better than `-65 dBm` and compare WDT/TCP/UDP errors.
3. Re-enable brownout detector and test with a stronger supply.
4. Switch to `stability_24k` and/or disable RTSP if unused.
5. Watch `/api/perf_status` and `/api/audio_status` for RSSI, heap, stack high-water marks, drops, write stalls, and timeouts.

## Preventive Measures
- Keep brownout detection enabled during stability investigations; only disable as a documented, temporary workaround.
- Treat RSSI below `-70 dBm` as a warning and `<= -75 dBm` as unsuitable for reliable 48 kHz streaming.
- Avoid synchronous network I/O in logging paths on constrained devices.
- Avoid producing into bounded buffers when no consumer exists unless the drops are explicitly harmless and silent.
- Keep WiFi/lwIP core scheduling headroom: minimize same-core high-priority application tasks and close stalled clients promptly.
- Document recommended stability settings: remote logging off or warning-only during field debugging, `stability_24k` for marginal WiFi, RTSP disabled when unused, and adequate power supply capacity.
