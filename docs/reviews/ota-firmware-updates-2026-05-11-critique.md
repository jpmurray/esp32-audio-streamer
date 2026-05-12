# OTA Firmware Updates Plan — Bounded Critique

## 1. Top 3 under-specified seams

1. **Upload handling inside `WebServer` is not specified enough.** The plan says `POST /api/ota/upload` should accept multipart firmware and call `Update.*` (`docs/plans/ota-firmware-updates-2026-05-11.md:57-60`), but does not define the Arduino `WebServer` upload callback shape, route registration form, or where per-chunk state lives. This matters because current routes are simple lambdas only (`src/HttpControl.cpp:804-820`), not multipart handlers.

2. **Audio/stream shutdown ordering is vague.** The plan requires stopping active streams and audio before flash writes (`docs/plans/ota-firmware-updates-2026-05-11.md:51`), but does not say whether OTA blocks until `streamServer_requestStopAndWait()` completes or what timeout/failure behavior should abort the upload. The relevant stop API exists (`include/StreamServer.h:101-109`) and `audioPipeline_stop()` is destructive (`include/AudioPipeline.h:31-38`), so ordering should be clarified before coding.

3. **Maintenance inhibit lifecycle has two competing owners.** The plan assigns inhibit state to `OtaManager` (`docs/plans/ota-firmware-updates-2026-05-11.md:34-37`) and also asks for a scheduler maintenance inhibit API (`docs/plans/ota-firmware-updates-2026-05-11.md:47-50`). It needs a single source of truth: scheduler-owned flag, OTA-owned predicate, or both with explicit synchronization. The guard must cover both direct sleep paths (`src/Scheduler.cpp:253-281`) and the early post-NTP sleep path in service startup (`src/main.cpp:119-130`).

## 2. Contradictions or missing dependencies

- **Token header dependency is missing from route setup.** The plan adds `X-ESP32MIC-OTA-TOKEN` (`docs/plans/ota-firmware-updates-2026-05-11.md:65-66`), but current header collection only registers `X-ESP32MIC-CSRF` (`src/HttpControl.cpp:800-802`). Without extending `collectHeaders`, token checks may silently fail.
- **Partition sizing depends on current binary size but is left as an open question.** Work item 1 asks to add `partitions/ota_4mb.csv` first (`docs/plans/ota-firmware-updates-2026-05-11.md:27-31`), while the open question says exact sizes should be finalized from binary and flash size (`docs/plans/ota-firmware-updates-2026-05-11.md:83`). Size confirmation should precede committing the CSV.
- **`otaManager_loop()` is not wired into the main loop in the work items.** The approach introduces it (`docs/plans/ota-firmware-updates-2026-05-11.md:34-37`), but the plan never explicitly updates `loop()` (`src/main.cpp:485-507`) to call it.

## 3. Risk of over-planning — cut or simplify

- Defer **optional token support** unless there is a concrete deployment requirement; CSRF-only is already the stated first-pass baseline (`docs/plans/ota-firmware-updates-2026-05-11.md:64-67`).
- Cut broad documentation updates from first implementation to the minimum migration/API notes. `architecture.md` changes can wait until the implementation shape is stable (`docs/plans/ota-firmware-updates-2026-05-11.md:73-77`).
- Keep validation to three acceptance tests first: valid upload/reboot, interrupted upload preserves old firmware, sleep inhibited during upload (`docs/plans/ota-firmware-updates-2026-05-11.md:79-87`). Stream-active OTA can follow after shutdown semantics are nailed down.

## 4. Questions that would change implementation order

1. What is the current firmware binary size and actual flash size of target deployed boards? If app slots are too small, partition design must happen before all API/UI work.
2. Should OTA be allowed while normal audio services are not started, e.g. AP-only setup mode? If yes, upload routes and UI availability may need to work before `startNormalServicesOnce()`.
3. On active stream shutdown failure, should upload abort immediately or continue after force-stopping audio? This decides whether stream/audio shutdown is prerequisite work or error handling inside upload.
4. Is `OTA_ADMIN_TOKEN` required for the first field deployment? If not, remove it from first pass to avoid changing header collection and UI status shape.
