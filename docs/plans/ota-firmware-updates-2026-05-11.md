# OTA Firmware Updates: Plan

## Goal

Add browser-driven OTA firmware updates to `esp32-audio-streamer` so a deployed device can receive a PlatformIO `firmware.bin` without serial flashing, while preserving the existing Web UI, `/api/*` control surface, Wi-Fi onboarding, audio streaming, and dawn/dusk deep-sleep behavior.

## Background

- The control-plane HTTP server is wired in `src/main.cpp:454-470`; `/api/*` routes live in `src/HttpControl.cpp:799-837`.
- Mutating API calls already require `X-ESP32MIC-CSRF: 1` via `src/HttpControl.cpp:72-83`; `handleApiReboot()` is the local precedent for sending a response before restarting (`src/HttpControl.cpp:786-795`).
- The System page is the natural upload surface: gzipped UI pages are served from `src/main.cpp:156-202` and regenerated through `tools/gen_webui_gzip_header.sh:1-68`.
- Normal services start after STA and scheduler checks in `src/main.cpp:111-151`; the loop continues HTTP, network, NTP, and sleep work in `src/main.cpp:485-507`.
- Deep sleep can currently happen from `scheduler_trySleepIfNight()` / `scheduler_deepSleepUntil()` (`src/Scheduler.cpp:252-281`), so OTA needs a maintenance guard before any flash write begins.
- `platformio.ini:13-19` has no custom OTA partition table. Browser OTA is not reliable until the device is serial-flashed once with `otadata` plus two app slots.
- Sibling BirdNET sketches use `ArduinoOTA`, but this project should use an HTTP upload route backed by Arduino-ESP32 `Update.h` so updates fit the existing browser control plane.

## Approach

Build a small OTA subsystem around Arduino-ESP32 `Update.h` rather than adding IDE-driven `ArduinoOTA`.

1. Confirm target flash size and current `firmware.bin` size, then add a tracked dual-slot partition table and wire it through `platformio.ini`.
2. Introduce an `OtaManager` module that owns OTA phase, progress, errors, maintenance state, and deferred reboot.
3. Let `OtaManager` be the owner of maintenance intent, but expose it to `Scheduler` through a simple inhibit flag/API so all sleep paths are blocked during OTA.
4. Add `/api/ota/status`, `/api/ota/upload`, and `/api/ota/abort` in `HttpControl.cpp`, using Arduino `WebServer`'s multipart upload-handler overload instead of the existing simple-lambda route shape.
5. Extend `webui/system.html` with a `.bin` upload card and regenerate `include/WebUI_gz.h`.
6. Document the one-time serial migration, API contract, and validation flow.

First implementation should support dual-slot uploads, interrupted upload failure, stream/audio shutdown before flash, progress/status reporting, and deferred reboot after success. Defer full ESP-IDF rollback confirmation and token-based auth unless field deployment requires them immediately.

## Work Items

### Orchestration Progress

- [x] Item 1 — OTA partitioning added (`partitions/ota_4mb.csv`, `platformio.ini`). Firmware measured at 906,432 bytes; board flash assumed from `wemos_d1_mini32` as 4 MB. Corrected after serial-flash testing so PlatformIO's `0xe000` boot_app0/otadata and `0x10000` serial firmware offsets match the CSV. Note: `.vscode/c_cpp_properties.json` changed during build/sizing and needs final review before commit.
- [x] Item 2 — Core `OtaManager`, scheduler inhibit, stream/audio protection, and OTA API routes implemented. Verified route registration, loop wiring, scheduler guards, corrected partition comments, free OTA space reporting, oversized-image preflight, interrupted-upload cleanup, client-side abort handling, and concurrent-upload rejection without corrupting active OTA state. Agent reported clean builds.
- [x] Item 3 — System Web UI upload flow added and `include/WebUI_gz.h` regenerated. Verified XHR upload, CSRF header, progress/status polling, reboot/error messaging, and `/api/ota/status` integration.
- [x] Item 4 — API/configuration docs and validation notes updated. Verified `/api/ota/*` response docs, partition/migration notes, artifact path, and validation checklist. `local_env.ini.example` intentionally unchanged.

1. **Size and add OTA-capable partitioning.**
   - Build the current firmware and record `.pio/build/wemos_d1_mini32/firmware.bin` size.
   - Confirm actual target-board flash size before committing slot sizes.
   - Add `partitions/ota_4mb.csv` with `nvs`, `otadata`, `app0`, `app1`, and a data partition sized for the measured binary plus growth margin.
   - Add `board_build.partitions = partitions/ota_4mb.csv` to `platformio.ini`.
   - Document that deployed devices need one serial flash with the new partition table before browser OTA can work.

2. **Create and wire `OtaManager`.**
   - Add `include/OtaManager.h` and `src/OtaManager.cpp` in the existing free-function module style.
   - Track phases such as `idle`, `receiving`, `success_reboot_pending`, and `failed`.
   - Expose status snapshot helpers, `otaManager_maintenanceActive()`, `otaManager_rebootPending()`, and `otaManager_loop()`.
   - Include `OtaManager.h` in `src/main.cpp`, call init from `setup()`, and call `otaManager_loop()` from `loop()` after `server.handleClient()`.
   - Schedule reboot from `otaManager_loop()` after the successful upload response has had time to leave the device.

3. **Protect OTA from sleep and service contention.**
   - Add a scheduler maintenance inhibit API in `include/Scheduler.h` / `src/Scheduler.cpp`, with `OtaManager` as the only writer.
   - Check the inhibit flag in both `scheduler_trySleepIfNight()` and `scheduler_deepSleepUntil()` so it covers normal loop sleep and the early post-NTP sleep path in `startNormalServicesOnce()`.
   - On upload start, set maintenance inhibit before accepting firmware bytes.
   - Stop active streams with `streamServer_requestStopAndWait()` using a bounded timeout; if streams do not drain, abort the upload before flash writes begin.
   - Stop audio capture after streams drain and before `Update.begin()` to free RAM and reduce flash-write contention.
   - Keep maintenance inhibit active from upload start through reboot; clear it only on failure or abort.

4. **Add OTA API endpoints.**
   - `GET /api/ota/status`: return support, phase, progress, free OTA space if available, and last error.
   - `POST /api/ota/upload`: register with `server.on(path, HTTP_POST, finalHandler, uploadHandler)`; keep per-chunk state in `OtaManager`, call `Update.begin()`, `Update.write()`, `Update.end()`, then schedule deferred reboot.
   - `POST /api/ota/abort`: abort only when receiving and safe.
   - Reject concurrent uploads, missing CSRF, oversized images, failed stream shutdown, and failed writes with JSON errors and log entries.

5. **Update the System Web UI.**
   - Add a System-page card for selecting `.pio/build/wemos_d1_mini32/firmware.bin`.
   - Use `XMLHttpRequest` for upload progress, include `X-ESP32MIC-CSRF: 1`, and show clear “do not power off” / “rebooting” states.
   - Poll `/api/ota/status` alongside existing System-page status/log polling.
   - Regenerate `include/WebUI_gz.h` with `tools/gen_webui_gzip_header.sh`.

6. **Update minimum docs and examples.**
   - `docs/api.md`: document `/api/ota/*` routes and response shapes.
   - `docs/configuration.md`: document partitioning, one-time serial migration, and upload artifact path.
   - `local_env.ini.example`: only add OTA-related notes if first implementation also adds build flags.

7. **Validate on hardware.**
   - Serial-flash once with the new partition table.
   - Upload a valid `firmware.bin` through the browser and confirm reboot into the new build.
   - Interrupt an upload and confirm the previous firmware remains bootable.
   - Start an update during night-mode conditions and confirm deep sleep is inhibited.
   - Start an update while a stream is active and confirm the upload aborts cleanly if the stream cannot drain before flash writes begin.

## Open Questions

- Exact partition sizes depend on the measured binary size and confirmed flash size.
- OTA in AP-only setup mode should be allowed only if tested explicitly; otherwise start with STA-connected browser updates.
- Full bootloader rollback confirmation and `OTA_ADMIN_TOKEN` are intentionally follow-up hardening items unless deployment requirements make either mandatory for the first build.

## References

- ESP-IDF OTA API: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ota.html
- Arduino-ESP32 docs: https://docs.espressif.com/projects/arduino-esp32/en/latest/
- Arduino-ESP32 `Update.h`: https://github.com/espressif/arduino-esp32/blob/master/libraries/Update/src/Update.h
- PlatformIO Espressif32 OTA docs: https://docs.platformio.org/en/latest/platforms/espressif32.html#over-the-air-ota-update
- Design critique: `docs/reviews/ota-firmware-updates-2026-05-11-critique.md`
- Prior local plan style: `docs/phase1-fast-safe-plan.md`
