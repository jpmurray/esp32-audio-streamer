// HttpControl.cpp — /api/* route handlers for the control-plane HTTP server.

#include "HttpControl.h"
#include "AppState.h"
#include "AudioPipeline.h"
#include "Scheduler.h"
#include "StreamServer.h"
#include "RuntimeSettings.h"
#include "NetworkManager.h"
#include "LogBuffer.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --------------------------------------------------------
// Logging
// --------------------------------------------------------
#ifndef LOG_LEVEL
#define LOG_LEVEL 2
#endif
#if LOG_LEVEL >= 2
#define LOGI(fmt, ...) logbuf_printf("[I][HC] " fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) logbuf_printf("[W][HC] " fmt, ##__VA_ARGS__)
#else
#define LOGI(...) do {} while (0)
#define LOGW(...) do {} while (0)
#endif
#if LOG_LEVEL >= 1
#define LOGE(fmt, ...) logbuf_printf("[E][HC] " fmt, ##__VA_ARGS__)
#else
#define LOGE(...) do {} while (0)
#endif

// --------------------------------------------------------
// Compile-time defaults exposed as read-only in status
// --------------------------------------------------------
#ifndef STREAM_PORT
#define STREAM_PORT 81
#endif
#ifndef SAMPLE_RATE_HZ
#define SAMPLE_RATE_HZ 48000
#endif
#ifndef CONVERT_SHIFT
#define CONVERT_SHIFT 11
#endif
#ifndef STREAM_WAV_ENABLE
#define STREAM_WAV_ENABLE 0
#endif
#ifndef LAT
#define LAT 51.4630911
#endif
#ifndef LON
#define LON -3.1678763
#endif
#ifndef LOCAL_TZ
#define LOCAL_TZ "UTC0"
#endif

// --------------------------------------------------------
// CSRF guard helper
// --------------------------------------------------------
static bool csrfOk(WebServer& server) {
    // Require header  X-ESP32MIC-CSRF: 1
    if (server.hasHeader("X-ESP32MIC-CSRF")) {
        return server.header("X-ESP32MIC-CSRF") == "1";
    }
    return false;
}

static void rejectCsrf(WebServer& server) {
    server.send(403, "application/json",
                "{\"error\":\"missing or invalid X-ESP32MIC-CSRF header\"}");
}

static String getArgStr(WebServer& server, const char* key) {
    for (int i = 0; i < server.args(); ++i) {
        if (server.argName(i) == key) return server.arg(i);
    }
    return String();
}

static String jsonEscape(const char* s) {
    String out;
    if (!s) return out;
    out.reserve(strlen(s) + 8);
    for (const char* p = s; *p; ++p) {
        char c = *p;
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"':  out += "\\\""; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if ((uint8_t)c < 0x20) {
                    char esc[7];
                    snprintf(esc, sizeof(esc), "\\u%04x", (unsigned)c);
                    out += esc;
                } else {
                    out += c;
                }
                break;
        }
    }
    return out;
}

static void sendJsonError(WebServer& server, int code, const char* error) {
    server.send(code, "application/json",
                String("{\"error\":\"") + jsonEscape(error) + "\"}");
}

// --------------------------------------------------------
// GET /api/status
// --------------------------------------------------------
static void handleApiStatus(WebServer& server) {
    time_t now = time(nullptr);
    char now_iso[24];
    scheduler_formatIso8601UTC(now, now_iso, sizeof(now_iso));
    scheduler_ensureSchedule(now);

    char tdawn_iso[24], tdusk_iso[24], mdawn_iso[24], mdusk_iso[24];
    scheduler_formatIso8601UTC(g_today_dawn_utc,    tdawn_iso, sizeof(tdawn_iso));
    scheduler_formatIso8601UTC(g_today_dusk_utc,    tdusk_iso, sizeof(tdusk_iso));
    scheduler_formatIso8601UTC(g_tomorrow_dawn_utc, mdawn_iso, sizeof(mdawn_iso));
    scheduler_formatIso8601UTC(g_tomorrow_dusk_utc, mdusk_iso, sizeof(mdusk_iso));

    const char* mode = "unknown";
    if (scheduler_timeIsValid()) {
        mode = (now >= g_today_dawn_utc && now < g_today_dusk_utc) ? "day" : "night";
    }

    char next_type[28] = "unknown"; time_t next_at = 0; uint32_t seconds_until = 0;
    if (scheduler_timeIsValid()) {
        if (strcmp(mode, "day") == 0) {
            strcpy(next_type, "sleep_at_civil_dusk"); next_at = g_today_dusk_utc;
        } else {
            strcpy(next_type, "wake_at_civil_dawn");
            next_at = (now < g_today_dawn_utc) ? g_today_dawn_utc : g_tomorrow_dawn_utc;
        }
        if (next_at > now) seconds_until = (uint32_t)(next_at - now);
    }
    char next_at_iso[24]; scheduler_formatIso8601UTC(next_at, next_at_iso, sizeof(next_at_iso));

    char stream_host[16];
    networkManager_streamHostIpString(stream_host, sizeof(stream_host));
    char stream_url[64] = "";
    if (stream_host[0]) {
        snprintf(stream_url, sizeof(stream_url), "http://%s:%d/stream",
                 stream_host, (int)STREAM_PORT);
    }

    char wifi_json[900];
    if (networkManager_writeStatusJson(wifi_json, sizeof(wifi_json)) < 0) {
        strcpy(wifi_json, "{}");
    }

    char buf[2048]; int n = 0;
    n += snprintf(buf + n, sizeof(buf) - n,
        "{\"now_utc\":\"%s\","
        "\"mode\":\"%s\","
        "\"boot_count\":%lu,"
        "\"uptime_sec\":%lu,"
        "\"location\":{\"lat\":%.5f,\"lon\":%.5f},"
        "\"today\":{\"civil_dawn_utc\":\"%s\",\"civil_dusk_utc\":\"%s\"},"
        "\"tomorrow\":{\"civil_dawn_utc\":\"%s\",\"civil_dusk_utc\":\"%s\"},"
        "\"next_event\":{\"type\":\"%s\",\"at_utc\":\"%s\",\"seconds_until\":%u},"
        "\"stream\":{\"url\":\"%s\",\"active\":%s,\"connect_count\":%lu},"
        "\"wifi\":%s,"
        "\"settings\":{\"wifi_tx_power_dbm\":%d,\"hpf_enabled\":%s,\"hpf_cutoff_hz\":%d,\"convert_shift\":%d}"
        "}",
        now_iso, mode,
        (unsigned long)g_boot_count,
        (unsigned long)(millis() / 1000UL),
        (float)LAT, (float)LON,
        tdawn_iso, tdusk_iso,
        mdawn_iso, mdusk_iso,
        next_type, next_at_iso, seconds_until,
        stream_url,
        g_stream_active ? "true" : "false",
        (unsigned long)g_stream_connect_count,
        wifi_json,
        (int)g_runtime_settings.wifi_tx_power_dbm,
        g_runtime_settings.hpf_enabled ? "true" : "false",
        (int)g_runtime_settings.hpf_cutoff_hz,
        (int)g_runtime_settings.convert_shift
    );

    if (n <= 0) { server.send(500, "application/json", "{\"error\":\"formatting\"}"); return; }
    server.send(200, "application/json", buf);
}

// --------------------------------------------------------
// GET /api/audio_status
// --------------------------------------------------------
static void handleApiAudioStatus(WebServer& server) {
    char stream_host[16];
    networkManager_streamHostIpString(stream_host, sizeof(stream_host));
    char stream_url[64] = "";
    if (stream_host[0]) {
        snprintf(stream_url, sizeof(stream_url), "http://%s:%d/stream",
                 stream_host, (int)STREAM_PORT);
    }

    AudioMetrics m = audioPipeline_getMetrics();

    char buf[800]; int n = 0;
    n += snprintf(buf + n, sizeof(buf) - n,
        "{"
        "\"i2s_ok\":%s,"
        "\"ringbuf_ok\":%s,"
        "\"stream_active\":%s,"
        "\"stream_connect_count\":%lu,"
        "\"stream_url\":\"%s\","
        "\"sample_rate_hz\":%d,"
        "\"convert_shift\":%d,"
        "\"stream_wav_enable\":%s,"
        "\"hpf_enabled\":%s,"
        "\"hpf_cutoff_hz\":%d,"
        "\"peak_level\":%d,"
        "\"peak_hold\":%d,"
        "\"clip_count\":%lu,"
        "\"clipped_last_block\":%s,"
        "\"i2s_error_count\":%lu,"
        "\"rb_drop_count\":%lu,"
        "\"stream_tx_bytes\":%lu,"
        "\"stream_write_stalls\":%lu,"
        "\"stream_timeout_count\":%lu"
        "}",
        g_i2s_ok     ? "true" : "false",
        g_rb_ok      ? "true" : "false",
        g_stream_active ? "true" : "false",
        (unsigned long)g_stream_connect_count,
        stream_url,
        (int)SAMPLE_RATE_HZ,
        audioPipeline_getActiveConvertShift(),
        (STREAM_WAV_ENABLE) ? "true" : "false",
        g_runtime_settings.hpf_enabled ? "true" : "false",
        (int)g_runtime_settings.hpf_cutoff_hz,
        (int)m.peak_level,
        (int)m.peak_hold,
        (unsigned long)m.clip_count,
        m.clipped_last_block ? "true" : "false",
        (unsigned long)m.i2s_error_count,
        (unsigned long)m.rb_drop_count,
        (unsigned long)g_stream_tx_bytes,
        (unsigned long)g_stream_write_stalls,
        (unsigned long)g_stream_timeout_count
    );

    if (n <= 0) { server.send(500, "application/json", "{\"error\":\"formatting\"}"); return; }
    server.send(200, "application/json", buf);
}

// --------------------------------------------------------
// GET /api/perf_status
// --------------------------------------------------------
static void handleApiPerfStatus(WebServer& server) {
    // Heap metrics
    uint32_t heap_free     = (uint32_t)ESP.getFreeHeap();
    uint32_t heap_min_free = (uint32_t)ESP.getMinFreeHeap();
    uint32_t heap_total    = (uint32_t)ESP.getHeapSize();

    // Task list for stack high-water marks
    uint32_t i2s_hwm   = g_i2s_task  ? (uint32_t)uxTaskGetStackHighWaterMark(g_i2s_task)  : 0;
    uint32_t loop_hwm  = (uint32_t)uxTaskGetStackHighWaterMark(nullptr); // calling task = loop

    // CPU frequency
    uint32_t cpu_mhz = getCpuFrequencyMhz();

    char buf[512]; int n = 0;
    n += snprintf(buf + n, sizeof(buf) - n,
        "{"
        "\"uptime_ms\":%lu,"
        "\"heap\":{\"free\":%lu,\"min_free\":%lu,\"total\":%lu},"
        "\"stack_hwm\":{\"i2s_producer\":%lu,\"loop\":%lu},"
        "\"cpu_mhz\":%lu,"
        "\"wifi_rssi_dbm\":%s"
        "}",
        (unsigned long)millis(),
        (unsigned long)heap_free,
        (unsigned long)heap_min_free,
        (unsigned long)heap_total,
        (unsigned long)i2s_hwm,
        (unsigned long)loop_hwm,
        (unsigned long)cpu_mhz,
        networkManager_staConnected() ? String(WiFi.RSSI()).c_str() : "null"
    );

    if (n <= 0) { server.send(500, "application/json", "{\"error\":\"formatting\"}"); return; }
    server.send(200, "application/json", buf);
}

// --------------------------------------------------------
// GET /api/logs
// --------------------------------------------------------
static void handleApiLogs(WebServer& server) {
    // Build  {"logs": [...]}
    // Allow up to 64 lines * ~130 chars each ≈ 8.3 KB + wrapper
    const size_t BUF_SZ = 10240;
    char* buf = (char*)malloc(BUF_SZ);
    if (!buf) { server.send(500, "application/json", "{\"error\":\"oom\"}"); return; }

    int n = 0;
    n += snprintf(buf + n, BUF_SZ - n, "{\"logs\":");
    int arr_n = logbuf_jsonArray(buf + n, BUF_SZ - n);
    if (arr_n < 0) {
        free(buf);
        server.send(500, "application/json", "{\"error\":\"log buffer overflow\"}");
        return;
    }
    n += arr_n;
    n += snprintf(buf + n, BUF_SZ - n, "}");
    server.send(200, "application/json", buf);
    free(buf);
}

// --------------------------------------------------------
// Strict input parsers
// --------------------------------------------------------

// Parse a strict decimal integer from `s`.
// Accepts an optional leading '-', then only ASCII digits.
// Rejects empty strings, leading/trailing whitespace, and non-digit chars.
// Returns true and sets *out on success; false otherwise.
static bool strictParseInt(const String& s, int* out) {
    if (!out || s.length() == 0) return false;
    const char* p = s.c_str();
    size_t len = s.length();
    size_t i = 0;
    if (p[i] == '-') { ++i; }          // allow leading minus
    if (i >= len) return false;         // bare '-' is invalid
    for (; i < len; ++i) {
        if (p[i] < '0' || p[i] > '9') return false;
    }
    // Safe to convert; value fits in long on any platform pio targets
    long v = strtol(p, nullptr, 10);
    if (v > INT_MAX || v < INT_MIN) return false;
    *out = (int)v;
    return true;
}

// Strict bool: accepts exactly "0", "1", "true", "false" (case-sensitive).
// Returns true and sets *out on success; false otherwise.
static bool strictParseBool(const String& s, bool* out) {
    if (!out) return false;
    if (s == "1" || s == "true")  { *out = true;  return true; }
    if (s == "0" || s == "false") { *out = false; return true; }
    return false;
}

// --------------------------------------------------------
// Wi-Fi management endpoints
// --------------------------------------------------------
static void handleApiWifiStatus(WebServer& server) {
    char buf[900];
    if (networkManager_writeStatusJson(buf, sizeof(buf)) < 0) {
        server.send(500, "application/json", "{\"error\":\"wifi status overflow\"}");
        return;
    }
    server.send(200, "application/json", buf);
}

static void handleApiWifiScan(WebServer& server) {
    int count = WiFi.scanNetworks();
    if (count < 0) {
        server.send(500, "application/json", "{\"error\":\"wifi scan failed\"}");
        return;
    }

    String json;
    json.reserve(64 + (count * 96));
    json += "{\"networks\":[";
    for (int i = 0; i < count; ++i) {
        if (i) json += ",";
        json += "{\"ssid\":\"";
        json += jsonEscape(WiFi.SSID(i).c_str());
        json += "\",\"rssi_dbm\":";
        json += String(WiFi.RSSI(i));
        json += ",\"encrypted\":";
        json += (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "false" : "true";
        json += ",\"channel\":";
        json += String(WiFi.channel(i));
        json += "}";
    }
    json += "]}";
    WiFi.scanDelete();
    server.send(200, "application/json", json);
}

static void handleApiWifiConfig(WebServer& server) {
    if (!csrfOk(server)) { rejectCsrf(server); return; }

    String ssid = getArgStr(server, "ssid");
    String password_action = getArgStr(server, "password_action");
    String password = getArgStr(server, "password");
    if (!password_action.length()) password_action = "set";

    char errmsg[128] = "";
    if (password_action == "keep") {
        NetworkStatusSnapshot snap;
        networkManager_getStatus(&snap);
        if (!snap.has_credentials) {
            sendJsonError(server, 400, "cannot keep password without saved credentials");
            return;
        }
        if (!ssid.length()) ssid = String(snap.saved_ssid);
        if (ssid != String(snap.saved_ssid)) {
            sendJsonError(server, 400, "password_action=keep requires existing SSID");
            return;
        }
        if (!networkManager_requestReconnect(errmsg, sizeof(errmsg))) {
            sendJsonError(server, 400, errmsg);
            return;
        }
    } else if (password_action == "set" || password_action == "clear") {
        if (password_action == "clear") password = "";
        if (!networkManager_saveCredentials(ssid, password, errmsg, sizeof(errmsg))) {
            sendJsonError(server, 400, errmsg);
            return;
        }
        if (!networkManager_requestReconnect(errmsg, sizeof(errmsg))) {
            sendJsonError(server, 400, errmsg);
            return;
        }
    } else {
        sendJsonError(server, 400, "password_action must be set, keep, or clear");
        return;
    }

    server.send(200, "application/json",
                "{\"ok\":true,\"action\":\"wifi-config\",\"reconnect_started\":true}");
}

static void handleApiWifiReconnect(WebServer& server) {
    if (!csrfOk(server)) { rejectCsrf(server); return; }
    char errmsg[128] = "";
    if (!networkManager_requestReconnect(errmsg, sizeof(errmsg))) {
        sendJsonError(server, 400, errmsg);
        return;
    }
    server.send(200, "application/json", "{\"ok\":true,\"action\":\"wifi-reconnect\"}");
}

static void handleApiWifiForget(WebServer& server) {
    if (!csrfOk(server)) { rejectCsrf(server); return; }

    bool disconnect_sta = true;
    String disconnect_arg = getArgStr(server, "disconnect");
    if (disconnect_arg.length() && !strictParseBool(disconnect_arg, &disconnect_sta)) {
        sendJsonError(server, 400, "disconnect must be 0, 1, true, or false");
        return;
    }

    char errmsg[128] = "";
    if (!networkManager_requestForgetAndStartAp(disconnect_sta, errmsg, sizeof(errmsg))) {
        sendJsonError(server, 500, errmsg);
        return;
    }

    NetworkStatusSnapshot snap;
    networkManager_getStatus(&snap);
    String json = "{\"ok\":true,\"action\":\"wifi-forget\",\"setup_ap_active\":";
    json += snap.setup_ap_active ? "true" : "false";
    json += ",\"ap_ip\":\"";
    json += jsonEscape(snap.ap_ip);
    json += "\",\"ap_ssid\":\"";
    json += jsonEscape(snap.ap_ssid);
    json += "\"}";
    server.send(200, "application/json", json);
}

// --------------------------------------------------------
// POST /api/set
// Accepts application/x-www-form-urlencoded body OR query params.
// Supported keys: wifi_tx_power_dbm, hpf_enabled, hpf_cutoff_hz, convert_shift
// convert_shift: persisted and flagged; restart-audio required to take effect.
// --------------------------------------------------------
static void handleApiSet(WebServer& server) {
    if (!csrfOk(server)) { rejectCsrf(server); return; }

    bool any = false;
    bool restart_audio_required = false;
    char errmsg[128] = "";

    auto getArgStr = [&](const char* key) -> String {
        for (int i = 0; i < server.args(); ++i) {
            if (server.argName(i) == key) return server.arg(i);
        }
        return String();
    };

    String val;

    val = getArgStr("wifi_tx_power_dbm");
    if (val.length()) {
        int v;
        if (!strictParseInt(val, &v)) {
            server.send(400, "application/json",
                        "{\"error\":\"wifi_tx_power_dbm must be an integer\"}");
            return;
        }
        if (!runtimeSettings_setWifiTxPowerDbm(v, errmsg, sizeof(errmsg))) {
            server.send(400, "application/json",
                        String("{\"error\":\"") + errmsg + "\"}");
            return;
        }
        any = true;
    }

    val = getArgStr("hpf_enabled");
    if (val.length()) {
        bool b;
        if (!strictParseBool(val, &b)) {
            server.send(400, "application/json",
                        "{\"error\":\"hpf_enabled must be 0, 1, true, or false\"}");
            return;
        }
        runtimeSettings_setHpfEnabled(b, errmsg, sizeof(errmsg));
        any = true;
    }

    val = getArgStr("hpf_cutoff_hz");
    if (val.length()) {
        int v;
        if (!strictParseInt(val, &v)) {
            server.send(400, "application/json",
                        "{\"error\":\"hpf_cutoff_hz must be an integer\"}");
            return;
        }
        if (!runtimeSettings_setHpfCutoffHz(v, errmsg, sizeof(errmsg))) {
            server.send(400, "application/json",
                        String("{\"error\":\"") + errmsg + "\"}");
            return;
        }
        any = true;
    }

    val = getArgStr("convert_shift");
    if (val.length()) {
        int v;
        if (!strictParseInt(val, &v)) {
            server.send(400, "application/json",
                        "{\"error\":\"convert_shift must be an integer\"}");
            return;
        }
        if (!runtimeSettings_setConvertShift(v, errmsg, sizeof(errmsg))) {
            server.send(400, "application/json",
                        String("{\"error\":\"") + errmsg + "\"}");
            return;
        }
        any = true;
        restart_audio_required = true;
    }

    if (!any) {
        server.send(400, "application/json",
                    "{\"error\":\"no recognised setting key provided\"}");
        return;
    }

    char buf[320]; int n = 0;
    n += snprintf(buf + n, sizeof(buf) - n,
        "{\"ok\":true,\"restart_audio_required\":%s,"
        "\"settings\":{\"wifi_tx_power_dbm\":%d,\"hpf_enabled\":%s,\"hpf_cutoff_hz\":%d,\"convert_shift\":%d}}",
        restart_audio_required ? "true" : "false",
        (int)g_runtime_settings.wifi_tx_power_dbm,
        g_runtime_settings.hpf_enabled ? "true" : "false",
        (int)g_runtime_settings.hpf_cutoff_hz,
        (int)g_runtime_settings.convert_shift
    );
    server.send(200, "application/json", buf);
}

// --------------------------------------------------------
// POST /api/action/reset-peak-hold
// --------------------------------------------------------
static void handleApiResetPeakHold(WebServer& server) {
    if (!csrfOk(server)) { rejectCsrf(server); return; }
    audioPipeline_resetPeakHold();
    server.send(200, "application/json", "{\"ok\":true,\"action\":\"reset-peak-hold\"}");
}

// --------------------------------------------------------
// POST /api/action/restart-audio
// --------------------------------------------------------
static void handleApiRestartAudio(WebServer& server) {
    if (!csrfOk(server)) { rejectCsrf(server); return; }
    LOGI("restart-audio requested\n");
    server.send(200, "application/json", "{\"ok\":true,\"action\":\"restart-audio\"}");
    // Signal the stream loop to exit cleanly before tearing down the pipeline.
    // This prevents a race where audioPipeline_stop() destroys the ring buffer
    // while the stream task is still reading from it.
    if (g_stream_active) {
        g_stream_stop_requested = true;
        LOGI("restart-audio: waiting for stream session to drain\n");
        // Wait up to 2 s for the stream loop to see the flag and exit.
        uint32_t wait_start = millis();
        while (g_stream_active && (millis() - wait_start) < 2000) {
            delay(20);
        }
        if (g_stream_active) {
            LOGW("restart-audio: stream session did not drain in time, proceeding\n");
        }
    }
    delay(50);
    audioPipeline_stop();
    delay(100);
    if (!audioPipeline_init()) {
        LOGE("restart-audio: audioPipeline_init() failed\n");
    }
}

// --------------------------------------------------------
// POST /api/action/time-sync
// --------------------------------------------------------
static void handleApiTimeSync(WebServer& server) {
    if (!csrfOk(server)) { rejectCsrf(server); return; }
    if (!networkManager_staConnected()) {
        sendJsonError(server, 409, "STA Wi-Fi is not connected");
        return;
    }
    LOGI("time-sync requested\n");
    scheduler_maybeSyncNtp();
    time_t now = time(nullptr);
    char now_iso[24]; scheduler_formatIso8601UTC(now, now_iso, sizeof(now_iso));
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"ok\":true,\"action\":\"time-sync\",\"now_utc\":\"%s\"}", now_iso);
    server.send(200, "application/json", buf);
}

// --------------------------------------------------------
// POST /api/action/reboot
// --------------------------------------------------------
static void handleApiReboot(WebServer& server) {
    if (!csrfOk(server)) { rejectCsrf(server); return; }
    LOGI("reboot requested\n");
    server.send(200, "application/json", "{\"ok\":true,\"action\":\"reboot\"}");
    delay(200);
    ESP.restart();
}

// --------------------------------------------------------
// Route registration
// --------------------------------------------------------
void httpControl_registerRoutes(WebServer& server) {
    // Collect custom headers that WebServer needs to parse
    static const char* CUSTOM_HEADERS[] = {"X-ESP32MIC-CSRF"};
    server.collectHeaders(CUSTOM_HEADERS, 1);

    // Use lambdas to capture the server reference into the zero-arg handlers
    server.on("/api/status",       HTTP_GET,  [&server]() { handleApiStatus(server);      });
    server.on("/api/audio_status", HTTP_GET,  [&server]() { handleApiAudioStatus(server); });
    server.on("/api/perf_status",  HTTP_GET,  [&server]() { handleApiPerfStatus(server);  });
    server.on("/api/logs",         HTTP_GET,  [&server]() { handleApiLogs(server);        });
    server.on("/api/wifi_status",  HTTP_GET,  [&server]() { handleApiWifiStatus(server);  });
    server.on("/api/wifi_scan",    HTTP_GET,  [&server]() { handleApiWifiScan(server);    });
    server.on("/api/set",          HTTP_POST, [&server]() { handleApiSet(server);         });
    server.on("/api/wifi/config",    HTTP_POST, [&server]() { handleApiWifiConfig(server);    });
    server.on("/api/wifi/reconnect", HTTP_POST, [&server]() { handleApiWifiReconnect(server); });
    server.on("/api/wifi/forget",    HTTP_POST, [&server]() { handleApiWifiForget(server);    });
    server.on("/api/action/restart-audio", HTTP_POST, [&server]() { handleApiRestartAudio(server); });
    server.on("/api/action/time-sync",     HTTP_POST, [&server]() { handleApiTimeSync(server);     });
    server.on("/api/action/reboot",        HTTP_POST, [&server]() { handleApiReboot(server);       });
    server.on("/api/action/reset-peak-hold", HTTP_POST, [&server]() { handleApiResetPeakHold(server); });

    // Suppress harmless browser noise (favicon, etc.) with a silent 404/204.
    server.onNotFound([&server]() {
        const String& path = server.uri();
        if (path == "/favicon.ico") {
            server.send(204, "text/plain", "");
            return;
        }
        if (!path.startsWith("/api/") && networkManager_setupApActive()) {
            server.sendHeader("Location", "/", true);
            server.send(302, "text/plain", "");
            return;
        }
        server.send(404, "application/json",
                    String("{\"error\":\"not found\",\"path\":\"") + jsonEscape(path.c_str()) + "\"}");
    });

    LOGI("Registered /api/* routes\n");
}
