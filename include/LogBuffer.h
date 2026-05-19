#pragma once
// LogBuffer.h — Circular in-memory log buffer for /api/logs, with centralized
//               level-aware logging API and optional UDP/syslog remote sink.
//
// All module logging should go through LOGE/LOGW/LOGI/LOGD macros defined here.
// Each macro accepts a module tag string and a printf-style format:
//
//     LOGI("NET", "STA connected: SSID='%s' RSSI=%d\n", ssid, rssi);
//
// Output goes to Serial, the RAM ring buffer (exposed by /api/logs), and
// optionally a best-effort UDP/syslog remote sink when ENABLE_REMOTE_LOG=1.
//
// Thread-safe via a FreeRTOS mutex (initialized in logbuf_init()).
// No heap allocation in the write path.

#include <stddef.h>
#include <stdarg.h>
#include <stdint.h>

// ------------------------------------------------------------
// Buffer sizing
// ------------------------------------------------------------

// Maximum number of lines retained in the ring buffer.
#ifndef LOG_BUF_LINES
#define LOG_BUF_LINES 64
#endif

// Maximum characters per log line (including null terminator).
#ifndef LOG_BUF_LINE_LEN
#define LOG_BUF_LINE_LEN 128
#endif

// ------------------------------------------------------------
// Compile-time log level
// ------------------------------------------------------------
// 1 = errors only
// 2 = errors, warnings, info  (default)
// 3 = errors, warnings, info, debug
#ifndef LOG_LEVEL
#define LOG_LEVEL 2
#endif

// ------------------------------------------------------------
// Remote UDP/syslog sink (compile-time opt-in)
// ------------------------------------------------------------
// Set ENABLE_REMOTE_LOG=1 in build flags to activate.
// REMOTE_LOG_HOST must be an IPv4 literal (e.g. "192.168.1.10").
// DNS is not used to avoid blocking.
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

// Remote severity threshold (independent of compile-time LOG_LEVEL).
// Only log entries with severity <= REMOTE_LOG_MIN_LEVEL are forwarded.
// 1=errors, 2=warnings+errors, 3=info+warn+error, 4=debug+all
#ifndef REMOTE_LOG_MIN_LEVEL
#define REMOTE_LOG_MIN_LEVEL 2
#endif

// Number of consecutive send failures before the remote sink enters backoff.
// After this many failures the sink is suspended for REMOTE_LOG_BACKOFF_MS.
#ifndef REMOTE_LOG_FAILURE_THRESHOLD
#define REMOTE_LOG_FAILURE_THRESHOLD 3
#endif

// Suspension duration in milliseconds after hitting REMOTE_LOG_FAILURE_THRESHOLD.
// After this window the sink is re-enabled automatically.
// Default: 5 minutes.
#ifndef REMOTE_LOG_BACKOFF_MS
#define REMOTE_LOG_BACKOFF_MS 300000
#endif

// STA RSSI threshold below which remote UDP sends are skipped entirely.
// Skipped sends do NOT count as send failures.
// Set to a large negative value (e.g. -120) to disable the RSSI guard.
#ifndef REMOTE_LOG_MIN_RSSI_DBM
#define REMOTE_LOG_MIN_RSSI_DBM -75
#endif

// ------------------------------------------------------------
// Log severity
// ------------------------------------------------------------

enum LogSeverity : uint8_t {
    LOG_SEV_ERROR = 1,
    LOG_SEV_WARN  = 2,
    LOG_SEV_INFO  = 3,
    LOG_SEV_DEBUG = 4,
};

// ------------------------------------------------------------
// Centralized logging macros
// ------------------------------------------------------------
// Usage: LOGI("MODULE", "format %s\n", arg)
// These compile away to nothing when LOG_LEVEL is below the severity.

#if LOG_LEVEL >= 1
#define LOGE(mod, fmt, ...) logbuf_log(LOG_SEV_ERROR, mod, fmt, ##__VA_ARGS__)
#else
#define LOGE(mod, ...) do {} while (0)
#endif

#if LOG_LEVEL >= 2
#define LOGW(mod, fmt, ...) logbuf_log(LOG_SEV_WARN, mod, fmt, ##__VA_ARGS__)
#define LOGI(mod, fmt, ...) logbuf_log(LOG_SEV_INFO, mod, fmt, ##__VA_ARGS__)
#else
#define LOGW(mod, ...) do {} while (0)
#define LOGI(mod, ...) do {} while (0)
#endif

#if LOG_LEVEL >= 3
#define LOGD(mod, fmt, ...) logbuf_log(LOG_SEV_DEBUG, mod, fmt, ##__VA_ARGS__)
#else
#define LOGD(mod, ...) do {} while (0)
#endif

// ------------------------------------------------------------
// Remote log status snapshot
// ------------------------------------------------------------

// Read-only snapshot of the remote logging sink state.
// Fields are valid regardless of ENABLE_REMOTE_LOG value.
struct RemoteLogStatus {
    bool     compiled_enabled;        // ENABLE_REMOTE_LOG was 1 at build time
    bool     configured;              // IP address parsed successfully
    bool     suspended;               // currently in backoff suspension
    uint32_t total_attempts;          // cumulative send attempts
    uint32_t total_successes;         // cumulative successful sends
    uint32_t total_failures;          // cumulative send failures
    uint32_t total_skipped_weak_rssi; // sends skipped due to weak RSSI
    uint32_t consecutive_failures;    // failures since last success
    uint32_t suspended_until_ms;      // millis() value when suspension expires
    int      last_rssi_dbm;           // RSSI observed at last attempt (0 = N/A)
};

// Fill *out with the current remote logging state.
// Safe to call from any task. No LOG* calls are made internally.
void logbuf_getRemoteStatus(RemoteLogStatus* out);

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------

// Initialize the log buffer, mutex, and remote sink (if enabled).
// Call once from setup() before any logging.
void logbuf_init();

// ------------------------------------------------------------
// Level-aware logging API
// ------------------------------------------------------------

// Primary entry point: severity + module tag + printf-style message.
void logbuf_log(LogSeverity severity, const char* module, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));

// va_list version for macro forwarding.
void logbuf_logv(LogSeverity severity, const char* module, const char* fmt, va_list ap);

// ------------------------------------------------------------
// Legacy compatibility API (kept for existing call sites)
// ------------------------------------------------------------

// printf-style write to Serial + ring buffer (severity treated as INFO).
void logbuf_printf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// va_list version.
void logbuf_vprintf(const char* fmt, va_list ap);

// ------------------------------------------------------------
// Reading
// ------------------------------------------------------------

// Append all buffered lines to `out` as a JSON array of strings.
// `out` must be at least `out_sz` bytes.
// Returns number of bytes written (excluding null terminator), or -1 on overflow.
int logbuf_jsonArray(char* out, size_t out_sz);
