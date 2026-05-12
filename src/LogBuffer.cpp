// LogBuffer.cpp — Circular in-memory log ring buffer with optional UDP/syslog sink.
//
// Write path: logbuf_log() -> logbuf_logv()
//   1. Formats the message with severity/module prefix.
//   2. Writes prefixed line to Serial.
//   3. Appends to RAM ring under mutex.
//   4. (If ENABLE_REMOTE_LOG) sends RFC3164-style UDP syslog, best-effort.
//
// Remote sink constraints:
//   - Best-effort: no retries, silent drop when Wi-Fi is down.
//   - Non-recursive: s_remote_sending guard prevents re-entry.
//   - REMOTE_LOG_HOST must be an IPv4 literal; DNS is never called.
//   - Mutex is released before UDP send to avoid blocking other tasks.

#include "LogBuffer.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#if ENABLE_REMOTE_LOG
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

// --------------------------------------------------------
// Storage
// --------------------------------------------------------
static char     s_lines[LOG_BUF_LINES][LOG_BUF_LINE_LEN];
static uint16_t s_head  = 0;   // index of next slot to write
static uint16_t s_count = 0;   // number of valid entries (saturates at LOG_BUF_LINES)
static SemaphoreHandle_t s_mutex = nullptr;

// --------------------------------------------------------
// Remote sink state (only compiled when enabled)
// --------------------------------------------------------
#if ENABLE_REMOTE_LOG
static WiFiUDP   s_remote_udp;
static IPAddress s_remote_ip;
static bool      s_remote_config_ok = false;
static bool      s_remote_sending   = false;
#endif

// --------------------------------------------------------
// Internal helpers
// --------------------------------------------------------

static const char* severityPrefix(LogSeverity sev) {
    switch (sev) {
        case LOG_SEV_ERROR: return "[E]";
        case LOG_SEV_WARN:  return "[W]";
        case LOG_SEV_INFO:  return "[I]";
        case LOG_SEV_DEBUG: return "[D]";
        default:            return "[?]";
    }
}

#if ENABLE_REMOTE_LOG
// RFC3164 syslog priority: facility local0 (128) + severity mapping.
static int syslogPRI(LogSeverity sev) {
    switch (sev) {
        case LOG_SEV_ERROR: return 128 + 3;  // local0.error
        case LOG_SEV_WARN:  return 128 + 4;  // local0.warning
        case LOG_SEV_INFO:  return 128 + 6;  // local0.info
        case LOG_SEV_DEBUG: return 128 + 7;  // local0.debug
        default:            return 128 + 6;
    }
}

static bool severityAllowedRemote(LogSeverity sev) {
    return (uint8_t)sev <= (uint8_t)REMOTE_LOG_MIN_LEVEL;
}

static void remoteLog(LogSeverity sev, const char* module, const char* msg) {
    if (s_remote_sending) return;
    if (!s_remote_config_ok) return;
    if (WiFi.status() != WL_CONNECTED) return;
    if (!severityAllowedRemote(sev)) return;

    s_remote_sending = true;

    // Build RFC3164-style payload: <PRI>DEVICE [MODULE] message
    char payload[256];
    snprintf(payload, sizeof(payload), "<%d>%s [%s] %s",
             syslogPRI(sev),
             REMOTE_LOG_DEVICE,
             module ? module : "-",
             msg);

    s_remote_udp.beginPacket(s_remote_ip, REMOTE_LOG_PORT);
    s_remote_udp.write((const uint8_t*)payload, strlen(payload));
    s_remote_udp.endPacket();

    s_remote_sending = false;
}
#endif // ENABLE_REMOTE_LOG

// --------------------------------------------------------
// Lifecycle
// --------------------------------------------------------
void logbuf_init() {
    s_head  = 0;
    s_count = 0;
    if (!s_mutex) s_mutex = xSemaphoreCreateMutex();

#if ENABLE_REMOTE_LOG
    // Parse IPv4 literal once at init. Disable silently on failure.
    const char* host = REMOTE_LOG_HOST;
    if (host && host[0] != '\0') {
        if (s_remote_ip.fromString(host)) {
            s_remote_config_ok = true;
        }
        // If parse fails, s_remote_config_ok stays false — sink is silently disabled.
    }
#endif
}

// --------------------------------------------------------
// Level-aware write path
// --------------------------------------------------------
void logbuf_logv(LogSeverity severity, const char* module, const char* fmt, va_list ap) {
    // 1. Format caller message.
    char msg[LOG_BUF_LINE_LEN];
    vsnprintf(msg, sizeof(msg), fmt, ap);

    // 2. Build prefixed local line: "[I][MODULE] message"
    char line[LOG_BUF_LINE_LEN];
    if (module && module[0] != '\0') {
        snprintf(line, sizeof(line), "%s[%s] %s", severityPrefix(severity), module, msg);
    } else {
        snprintf(line, sizeof(line), "%s %s", severityPrefix(severity), msg);
    }

    // 3. Write to Serial.
    Serial.print(line);

    // 4. Append to RAM ring under mutex.
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    strncpy(s_lines[s_head], line, LOG_BUF_LINE_LEN - 1);
    s_lines[s_head][LOG_BUF_LINE_LEN - 1] = '\0';
    s_head = (s_head + 1) % LOG_BUF_LINES;
    if (s_count < LOG_BUF_LINES) s_count++;
    if (s_mutex) xSemaphoreGive(s_mutex);

    // 5. Optional remote sink — called after mutex is released.
#if ENABLE_REMOTE_LOG
    remoteLog(severity, module, msg);
#endif
}

void logbuf_log(LogSeverity severity, const char* module, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    logbuf_logv(severity, module, fmt, ap);
    va_end(ap);
}

// --------------------------------------------------------
// Legacy compatibility wrappers
// --------------------------------------------------------
void logbuf_vprintf(const char* fmt, va_list ap) {
    logbuf_logv(LOG_SEV_INFO, nullptr, fmt, ap);
}

void logbuf_printf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    logbuf_logv(LOG_SEV_INFO, nullptr, fmt, ap);
    va_end(ap);
}

// --------------------------------------------------------
// Reading — produce JSON array of strings
// --------------------------------------------------------
int logbuf_jsonArray(char* out, size_t out_sz) {
    if (!out || out_sz < 4) return -1;

    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);

    // Determine oldest entry index
    uint16_t start = (s_count < LOG_BUF_LINES) ? 0 : s_head;
    uint16_t total = s_count;

    // Copy snapshot to heap buffer to avoid large stack allocation (~8 KB).
    char (*snapshot)[LOG_BUF_LINE_LEN] = (char (*)[LOG_BUF_LINE_LEN])
        malloc((size_t)total * LOG_BUF_LINE_LEN);
    if (!snapshot) {
        if (s_mutex) xSemaphoreGive(s_mutex);
        return -1;
    }
    for (uint16_t i = 0; i < total; ++i) {
        uint16_t idx = (start + i) % LOG_BUF_LINES;
        memcpy(snapshot[i], s_lines[idx], LOG_BUF_LINE_LEN);
    }

    if (s_mutex) xSemaphoreGive(s_mutex);

    // Build JSON array
    size_t pos = 0;
    auto append = [&](const char* s) -> bool {
        size_t len = strlen(s);
        if (pos + len >= out_sz) return false;
        memcpy(out + pos, s, len);
        pos += len;
        return true;
    };

    if (!append("[")) { free(snapshot); return -1; }
    for (uint16_t i = 0; i < total; ++i) {
        if (i > 0 && !append(",")) { free(snapshot); return -1; }
        if (!append("\"")) { free(snapshot); return -1; }
        // JSON-escape each line so r.json() can parse log output reliably.
        for (const char* p = snapshot[i]; *p; ++p) {
            const char* esc = nullptr;
            switch (*p) {
                case '\\': esc = "\\\\"; break;
                case '"':  esc = "\\\""; break;
                case '\b': esc = "\\b"; break;
                case '\f': esc = "\\f"; break;
                case '\n': esc = "\\n"; break;
                case '\r': esc = "\\r"; break;
                case '\t': esc = "\\t"; break;
                default:
                    break;
            }
            if (esc) {
                if (!append(esc)) { free(snapshot); return -1; }
                continue;
            }
            if ((unsigned char)*p < 0x20) {
                char hex[7];
                snprintf(hex, sizeof(hex), "\\u%04x", (unsigned char)*p);
                if (!append(hex)) { free(snapshot); return -1; }
                continue;
            }
            if (pos + 2 >= out_sz) { free(snapshot); return -1; }
            out[pos++] = *p;
        }
        if (!append("\"")) { free(snapshot); return -1; }
    }
    if (!append("]")) { free(snapshot); return -1; }
    out[pos] = '\0';
    free(snapshot);
    return (int)pos;
}
