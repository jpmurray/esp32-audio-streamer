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
