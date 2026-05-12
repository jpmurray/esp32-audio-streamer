// OtaManager.cpp — browser-driven OTA firmware update subsystem.
//
// Upload flow:
//   1. POST /api/ota/upload arrives (multipart).
//   2. otaManager_handleUploadChunk() is called for each chunk:
//      - On first chunk: CSRF check, set maintenance inhibit, stop streams,
//        stop audio, call Update.begin().
//      - Subsequent chunks: Update.write().
//      - On last chunk: Update.end(); set phase to success_reboot_pending.
//      - On UPLOAD_FILE_ABORTED (client disconnect / xhr.abort()): failUpload().
//   3. otaManager_handleUploadFinal() sends the JSON response.
//   4. otaManager_loop() issues ESP.restart() after a brief delay.

#include "OtaManager.h"
#include "Scheduler.h"
#include "StreamServer.h"
#include "AudioPipeline.h"
#include "AppState.h"
#include "LogBuffer.h"

#include <Arduino.h>
#include <Update.h>
#include <WebServer.h>

// ----------------------------------------------------------------
// Tunables
// ----------------------------------------------------------------
#ifndef OTA_STREAM_DRAIN_TIMEOUT_MS
#define OTA_STREAM_DRAIN_TIMEOUT_MS 3000
#endif

#ifndef OTA_REBOOT_DELAY_MS
#define OTA_REBOOT_DELAY_MS 1500
#endif

// ----------------------------------------------------------------
// Logging: use centralized macros from LogBuffer.h.
// ----------------------------------------------------------------

// ----------------------------------------------------------------
// Module state
// ----------------------------------------------------------------
static OtaPhase   s_phase           = OtaPhase::idle;
static int        s_progress_pct    = 0;
static char       s_last_error[128] = "";
static uint32_t   s_reboot_at_ms    = 0;
static bool       s_update_started  = false;  // Update.begin() succeeded
static bool       s_audio_stopped   = false;  // true only when OTA stopped the pipeline
static size_t     s_total_size      = 0;
static size_t     s_written         = 0;
static bool       s_csrf_failed     = false;  // set in first chunk, reported in final
static bool       s_rejected        = false;  // concurrent-upload rejection; reported in final without touching OTA state

// ----------------------------------------------------------------
// Internal helpers
// ----------------------------------------------------------------
static void setError(const char* msg) {
    strncpy(s_last_error, msg, sizeof(s_last_error) - 1);
    s_last_error[sizeof(s_last_error) - 1] = '\0';
}

static void resetState() {
    s_phase          = OtaPhase::idle;
    s_progress_pct   = 0;
    s_last_error[0]  = '\0';
    s_reboot_at_ms   = 0;
    s_update_started = false;
    s_audio_stopped  = false;
    s_total_size     = 0;
    s_written        = 0;
    s_csrf_failed    = false;
    s_rejected       = false;
    scheduler_setMaintenanceInhibit(false);
}

// Central failure/cleanup path.
// Calls Update.abort() if flash was open, clears inhibit, restores audio only
// if OTA was the one that stopped it, then sets phase to failed.
static void failUpload(const char* reason) {
    setError(reason);
    if (s_update_started) {
        Update.abort();
        s_update_started = false;
    }
    scheduler_setMaintenanceInhibit(false);
    if (s_audio_stopped) {
        audioPipeline_init();
        s_audio_stopped = false;
        LOGI("OTA", "Audio pipeline restored after failure\n");
    }
    s_phase = OtaPhase::failed;
    LOGE("OTA", "Upload failed: %s\n", reason);
}

// Check CSRF header on the server request.
static bool csrfOk(WebServer& server) {
    return server.hasHeader("X-ESP32MIC-CSRF") &&
           server.header("X-ESP32MIC-CSRF") == "1";
}

// ----------------------------------------------------------------
// Public API
// ----------------------------------------------------------------
void otaManager_init() {
    resetState();
    LOGI("OTA", "OtaManager initialised\n");
}

void otaManager_loop() {
    if (s_phase == OtaPhase::success_reboot_pending && s_reboot_at_ms > 0) {
        if (millis() >= s_reboot_at_ms) {
            LOGI("OTA", "Rebooting into new firmware\n");
            Serial.flush();
            delay(100);
            ESP.restart();
        }
    }
}

OtaPhase    otaManager_phase()       { return s_phase; }
int         otaManager_progressPct() { return s_progress_pct; }
const char* otaManager_lastError()   { return s_last_error; }

bool otaManager_maintenanceActive() {
    return s_phase == OtaPhase::receiving ||
           s_phase == OtaPhase::success_reboot_pending;
}

bool otaManager_rebootPending() {
    return s_phase == OtaPhase::success_reboot_pending;
}

bool otaManager_abort() {
    if (s_phase != OtaPhase::receiving) return false;
    LOGI("OTA", "Abort requested via API\n");
    failUpload("aborted by request");
    return true;
}

// ----------------------------------------------------------------
// Per-chunk upload handler
// ----------------------------------------------------------------
void otaManager_handleUploadChunk(WebServer& server) {
    HTTPUpload& upload = server.upload();

    // ---- Client disconnected or connection lost mid-upload ----
    if (upload.status == UPLOAD_FILE_ABORTED) {
        // Only clean up if we were actively receiving (i.e. Update.begin() was
        // called).  If pre-flight failed we were never in receiving phase.
        if (s_phase == OtaPhase::receiving) {
            failUpload("upload interrupted (client disconnected)");
        }
        return;
    }

    if (upload.status == UPLOAD_FILE_START) {
        // ---- First chunk: pre-flight checks ----

        // CSRF must be on the outer request, not the multipart chunk.
        if (!csrfOk(server)) {
            s_csrf_failed = true;
            return;
        }

        // Reject if already in progress — set a per-request flag and return
        // immediately.  Do NOT touch s_phase, the maintenance inhibit, or any
        // other OTA state; the existing upload or reboot sequence must continue
        // undisturbed.  otaManager_handleUploadFinal() reads s_rejected to send
        // the 409 response for this incoming request only.
        if (s_phase == OtaPhase::receiving ||
            s_phase == OtaPhase::success_reboot_pending) {
            s_rejected = true;
            return;
        }

        // Reset any prior failure.
        resetState();
        s_phase = OtaPhase::receiving;
        scheduler_setMaintenanceInhibit(true);
        LOGI("OTA", "Upload started: %s\n", upload.filename.c_str());

        // Stop active streams before touching flash.
        bool streams_ok = streamServer_requestStopAndWait(OTA_STREAM_DRAIN_TIMEOUT_MS);
        if (!streams_ok) {
            failUpload("stream did not drain in time");
            return;
        }

        // Stop audio pipeline to free RAM/DMA before flash writes.
        // Set s_audio_stopped before calling stop so failUpload() knows to restore.
        audioPipeline_stop();
        s_audio_stopped = true;
        LOGI("OTA", "Audio pipeline stopped for OTA\n");

        // Determine firmware size for preflight and progress reporting.
        // Preference order:
        //   1. upload.totalSize — populated by WebServer from the multipart part's
        //      Content-Length sub-header (most reliable when the browser sends it).
        //   2. Outer HTTP Content-Length — includes multipart framing overhead so it
        //      overestimates the actual binary; usable only as a loose upper bound.
        //   3. 0 — size unknown; Update.h's own internal size check is the final guard.
        size_t hint_size = 0;
        if (upload.totalSize > 0) {
            hint_size = (size_t)upload.totalSize;
        } else if (server.hasHeader("Content-Length")) {
            hint_size = (size_t)server.header("Content-Length").toInt();
        }
        s_total_size = hint_size;

        // Preflight: reject before any flash write if the image is clearly too large.
        // ESP.getFreeSketchSpace() returns the inactive OTA slot size; returns 0
        // when no OTA partition table is present.
        uint32_t free_slot = ESP.getFreeSketchSpace();
        if (free_slot == 0) {
            failUpload("no OTA partition available");
            return;
        }
        if (hint_size > 0 && hint_size > (size_t)free_slot) {
            char err[80];
            snprintf(err, sizeof(err),
                     "image too large (%u bytes) for OTA slot (%u bytes)",
                     (unsigned)hint_size, (unsigned)free_slot);
            failUpload(err);
            return;
        }

        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            char err[64];
            strncpy(err, Update.errorString(), sizeof(err) - 1); err[sizeof(err)-1] = '\0';
            failUpload(err);
            return;
        }

        s_update_started = true;
        s_written        = 0;
        s_progress_pct   = 0;
        LOGI("OTA", "Update.begin() ok; free slot %u bytes\n", (unsigned)free_slot);
    }

    if (upload.status == UPLOAD_FILE_WRITE) {
        if (!s_update_started || s_phase != OtaPhase::receiving) return;

        size_t written = Update.write(upload.buf, upload.currentSize);
        if (written != upload.currentSize) {
            char err[64];
            strncpy(err, Update.errorString(), sizeof(err) - 1); err[sizeof(err)-1] = '\0';
            failUpload(err);
            return;
        }
        s_written += written;
        if (s_total_size > 0) {
            s_progress_pct = (int)((s_written * 100ULL) / s_total_size);
        }
    }

    if (upload.status == UPLOAD_FILE_END) {
        if (!s_update_started || s_phase != OtaPhase::receiving) return;

        if (!Update.end(true)) {
            char err[64];
            strncpy(err, Update.errorString(), sizeof(err) - 1); err[sizeof(err)-1] = '\0';
            s_update_started = false;  // Update already closed; don't call abort
            failUpload(err);
            return;
        }

        s_update_started = false;
        s_audio_stopped  = false;  // device is rebooting; no need to restore audio
        s_progress_pct   = 100;
        s_phase          = OtaPhase::success_reboot_pending;
        s_reboot_at_ms   = millis() + OTA_REBOOT_DELAY_MS;
        // Maintenance inhibit intentionally kept active until reboot.
        LOGI("OTA", "Update complete (%u bytes); rebooting in %d ms\n",
             (unsigned)s_written, OTA_REBOOT_DELAY_MS);
    }
}

// ----------------------------------------------------------------
// Final response handler (called after multipart ends)
// ----------------------------------------------------------------
void otaManager_handleUploadFinal(WebServer& server) {
    if (s_csrf_failed) {
        server.send(403, "application/json",
                    "{\"ok\":false,\"error\":\"CSRF check failed\"}");
        return;
    }
    if (s_rejected) {
        s_rejected = false;
        server.send(409, "application/json",
                    "{\"ok\":false,\"error\":\"update already in progress\"}");
        return;
    }
    if (s_phase == OtaPhase::success_reboot_pending) {
        server.send(200, "application/json",
                    "{\"ok\":true,\"message\":\"Update complete; rebooting\"}");
        return;
    }
    // Phase is failed or (edge case) idle after ABORTED with no prior receiving state.
    char body[256];
    snprintf(body, sizeof(body),
             "{\"ok\":false,\"error\":\"%s\"}", s_last_error[0] ? s_last_error : "unknown");
    server.send(500, "application/json", body);
}
