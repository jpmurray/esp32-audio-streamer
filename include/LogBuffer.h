#pragma once
// LogBuffer.h — Circular in-memory log buffer for /api/logs.
//
// Stores the last LOG_BUF_LINES log lines in a fixed-size ring buffer.
// logbuf_printf() writes to both Serial and the ring buffer atomically.
//
// The buffer is allocated statically; no heap needed.
// Thread-safe via a FreeRTOS mutex (initialized in logbuf_init()).

#include <stddef.h>
#include <stdarg.h>

// Maximum number of lines retained in the ring buffer.
#ifndef LOG_BUF_LINES
#define LOG_BUF_LINES 64
#endif

// Maximum characters per log line (including null terminator).
#ifndef LOG_BUF_LINE_LEN
#define LOG_BUF_LINE_LEN 128
#endif

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------

// Initialize the log buffer and its mutex.  Call once before any logging.
void logbuf_init();

// ------------------------------------------------------------
// Writing
// ------------------------------------------------------------

// printf-style write to Serial + ring buffer.
void logbuf_printf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// va_list version for macro forwarding.
void logbuf_vprintf(const char* fmt, va_list ap);

// ------------------------------------------------------------
// Reading
// ------------------------------------------------------------

// Append all buffered lines to `out` as a JSON array of strings.
// `out` must be at least `out_sz` bytes.
// Returns number of bytes written (excluding null terminator), or -1 on overflow.
int logbuf_jsonArray(char* out, size_t out_sz);
