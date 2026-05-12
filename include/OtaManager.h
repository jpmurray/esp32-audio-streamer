#pragma once
// OtaManager.h — browser-driven OTA firmware update subsystem.
//
// Uses Arduino-ESP32 Update.h (HTTP multipart upload via WebServer).
// Phases:
//   idle                 — no update in progress
//   receiving            — Update.begin() called, chunks being written
//   success_reboot_pending — Update.end() succeeded; reboot deferred
//   failed               — Update failed or was aborted; cleared on next attempt
//
// The OTA manager also owns the scheduler maintenance inhibit: it sets the flag
// before any flash write and clears it only on failure, abort, or after reboot.

#include <Arduino.h>
#include <WebServer.h>
#include <stdint.h>

// ------------------------------------------------------------
// Phase enum (public so HttpControl can report it)
// ------------------------------------------------------------
enum class OtaPhase : uint8_t {
    idle = 0,
    receiving,
    success_reboot_pending,
    failed
};

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------
void otaManager_init();
void otaManager_loop();   // call every loop() after server.handleClient()

// ------------------------------------------------------------
// State queries
// ------------------------------------------------------------
OtaPhase    otaManager_phase();
int         otaManager_progressPct();   // 0-100; valid while receiving
const char* otaManager_lastError();     // empty string when none
bool        otaManager_maintenanceActive(); // true while receiving or reboot pending
bool        otaManager_rebootPending();

// ------------------------------------------------------------
// Upload handlers (registered by HttpControl via WebServer::on overload)
// ------------------------------------------------------------
// Final response handler (called after all chunks arrive).
void otaManager_handleUploadFinal(WebServer& server);
// Per-chunk upload handler.
void otaManager_handleUploadChunk(WebServer& server);

// ------------------------------------------------------------
// Abort (called by POST /api/ota/abort handler)
// ------------------------------------------------------------
// Returns true if abort was accepted (was in receiving phase).
bool otaManager_abort();
