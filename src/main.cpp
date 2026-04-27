// main.cpp — Application wiring: route registration, setup(), loop().
// Heavy logic lives in AppState / AudioPipeline / Scheduler modules.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "AppState.h"
#include "AudioPipeline.h"
#include "Scheduler.h"
#include "StreamServer.h"
#include "RuntimeSettings.h"
#include "LogBuffer.h"
#include "HttpControl.h"
#include "WebUI_gz.h"

// ------------------------------------------------------------
// Logging (compile-time): -D LOG_LEVEL=1/2/3
// 1 = errors only, 2 = info+warn+error (default), 3 = verbose/debug
// ------------------------------------------------------------
#ifndef LOG_LEVEL
#define LOG_LEVEL 2
#endif

#if LOG_LEVEL >= 3
#define LOGD(fmt, ...) logbuf_printf("[D] " fmt, ##__VA_ARGS__)
#else
#define LOGD(...) do {} while (0)
#endif

#if LOG_LEVEL >= 2
#define LOGI(fmt, ...) logbuf_printf("[I] " fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) logbuf_printf("[W] " fmt, ##__VA_ARGS__)
#else
#define LOGI(...) do {} while (0)
#define LOGW(...) do {} while (0)
#endif

#if LOG_LEVEL >= 1
#define LOGE(fmt, ...) logbuf_printf("[E] " fmt, ##__VA_ARGS__)
#else
#define LOGE(...) do {} while (0)
#endif

#if LOG_LEVEL >= 3
#define LOG_DOT() Serial.print('.')
#define LOG_NL()  Serial.println()
#else
#define LOG_DOT() do {} while (0)
#define LOG_NL()  do {} while (0)
#endif

// ------------------------------------------------------------
// Compile-time configuration
// ------------------------------------------------------------
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YOUR_PASSWORD"
#endif
#ifndef WIFI_TX_POWER_DBM
#define WIFI_TX_POWER_DBM 15
#endif
#ifndef ENABLE_BROWNOUT_DISABLE
#define ENABLE_BROWNOUT_DISABLE 1
#endif
#ifndef SERVER_PORT
#define SERVER_PORT 80
#endif
#ifndef STREAM_PORT
#define STREAM_PORT 81
#endif
#ifndef STREAM_WAV_ENABLE
#define STREAM_WAV_ENABLE 0
#endif
#ifndef LOCAL_TZ
#define LOCAL_TZ "UTC0"
#endif
#ifndef SAMPLE_RATE_HZ
#define SAMPLE_RATE_HZ 48000
#endif
#ifndef LAT
#define LAT 51.4630911
#endif
#ifndef LON
#define LON -3.1678763
#endif

// ------------------------------------------------------------
// HTTP server
// ------------------------------------------------------------
static WebServer server(SERVER_PORT);

// ------------------------------------------------------------
// Wi-Fi helpers
// ------------------------------------------------------------
static wifi_power_t mapTxPowerDbm(int dbm) {
    if (dbm >= 20) return WIFI_POWER_19_5dBm;
    if (dbm >= 19) return WIFI_POWER_19dBm;
    if (dbm >= 18) return WIFI_POWER_18_5dBm;
    if (dbm >= 17) return WIFI_POWER_17dBm;
    if (dbm >= 15) return WIFI_POWER_15dBm;
    if (dbm >= 13) return WIFI_POWER_13dBm;
    if (dbm >= 11) return WIFI_POWER_11dBm;
    if (dbm >= 9)  return WIFI_POWER_8_5dBm;
    if (dbm >= 7)  return WIFI_POWER_7dBm;
    if (dbm >= 5)  return WIFI_POWER_5dBm;
    if (dbm >= 2)  return WIFI_POWER_2dBm;
    return WIFI_POWER_MINUS_1dBm;
}

static void connectWiFiBlocking() {
    const char* ssid = WIFI_SSID;
    const char* pass = WIFI_PASS;
    WiFi.persistent(false);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(200);
    esp_wifi_stop();
    esp_wifi_deinit();
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    if (String(WIFI_SSID) == "YOUR_SSID" || String(WIFI_PASS) == "YOUR_PASSWORD") {
        LOGW("WIFI_SSID/WIFI_PASS are placeholders.\n");
        LOGW("Set them in platformio.ini build_flags, then rebuild/flash.\n");
    }

    LOGI("Connecting to Wi-Fi SSID: '%s'\n", ssid);
    wifi_power_t txp = mapTxPowerDbm((int)WIFI_TX_POWER_DBM);
    WiFi.setTxPower(txp);
    LOGI("[WiFi] TX power target=%d dBm (mapped enum=%d)\n", (int)WIFI_TX_POWER_DBM, (int)txp);
    WiFi.begin(ssid, pass);

    uint32_t dot = 0, lastDiag = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        LOG_DOT();
        if ((++dot % 40) == 0) LOG_NL();
        if (millis() - lastDiag > 10000) {
#if LOG_LEVEL >= 3
            LOGD("\n[WiFi] Still connecting... printing diagnostics\n");
            WiFi.printDiag(Serial);
#endif
            lastDiag = millis();
        }
    }
    LOG_NL();
#if LOG_LEVEL >= 2
    Serial.print("[I] Connected. IP address: ");
    Serial.println(WiFi.localIP());
#endif
}

static void disableBrownout() {
#ifdef RTC_CNTL_BROWN_OUT_REG
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif
}

// ------------------------------------------------------------
// HTTP route handlers
// ------------------------------------------------------------
static void handleRoot() {
    // Serve the embedded gzip-compressed Web UI.
    // The browser decompresses it automatically when Content-Encoding: gzip is set.
    server.sendHeader("Content-Encoding", "gzip");
    server.sendHeader("Cache-Control", "no-cache");
    // send() copies from PROGMEM via the WebServer internals; we must use
    // a RAM copy because WebServer::send(code, type, buf, len) is available.
    server.send_P(200, "text/html", (const char*)WEBUI_INDEX_GZ, WEBUI_INDEX_GZ_LEN);
}

// handleStream() moved to StreamServer.cpp (port STREAM_PORT)

static void handleUptime() {
    unsigned long total = millis() / 1000UL;
    unsigned long d = total / 86400UL; total %= 86400UL;
    unsigned long h = total / 3600UL;  total %= 3600UL;
    unsigned long m = total / 60UL;    unsigned long s = total % 60UL;

    char human[64]; size_t pos = 0;
    if (d > 0) pos += snprintf(human + pos, sizeof(human) - pos, "%lud ", d);
    if (d > 0 || h > 0) pos += snprintf(human + pos, sizeof(human) - pos, "%luh ", h);
    if (d > 0 || h > 0 || m > 0) pos += snprintf(human + pos, sizeof(human) - pos, "%lum ", m);
    snprintf(human + pos, sizeof(human) - pos, "%lus", s);

    time_t now = time(nullptr);
    char now_iso[24]; scheduler_formatIso8601UTC(now, now_iso, sizeof(now_iso));
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

    String wakes_csv  = g_prefs.isKey(PREF_KEY_LAST_WAKES)  ? g_prefs.getString(PREF_KEY_LAST_WAKES,  "") : String("");
    String sleeps_csv = g_prefs.isKey(PREF_KEY_NEXT_SLEEPS) ? g_prefs.getString(PREF_KEY_NEXT_SLEEPS, "") : String("");

    char buf[1024]; int n = 0;
    n += snprintf(buf + n, sizeof(buf) - n,
                  "{\"uptime\": %lu, \"uptime_human\": \"%s\", \"days\": %lu, \"hours\": %lu, \"minutes\": %lu, \"seconds\": %lu, ",
                  (millis() / 1000UL), human, d, h, m, s);
    n += snprintf(buf + n, sizeof(buf) - n, "\"now_utc\": \"%s\", ", now_iso);
    char last_check_iso[24]; scheduler_formatIso8601UTC(g_last_ntp_check_utc, last_check_iso, sizeof(last_check_iso));
    n += snprintf(buf + n, sizeof(buf) - n, "\"last_ntp_check_utc\": \"%s\", ", last_check_iso);
    n += snprintf(buf + n, sizeof(buf) - n,
                  "\"today\": {\"civil_dawn_utc\": \"%s\", \"civil_dusk_utc\": \"%s\"}, ",
                  tdawn_iso, tdusk_iso);
    n += snprintf(buf + n, sizeof(buf) - n,
                  "\"boot_count\": %lu, \"schedule_basis\": \"civil_twilight_-6deg\", \"location\": {\"lat\": %.5f, \"lon\": %.5f}, ",
                  (unsigned long)g_boot_count, (float)LAT, (float)LON);
    n += snprintf(buf + n, sizeof(buf) - n,
                  "\"tomorrow\": {\"civil_dawn_utc\": \"%s\", \"civil_dusk_utc\": \"%s\"}, \"mode\": \"%s\", ",
                  mdawn_iso, mdusk_iso, mode);
    n += snprintf(buf + n, sizeof(buf) - n,
                  "\"next_event\": {\"type\": \"%s\", \"at_utc\": \"%s\", \"seconds_until\": %u}, ",
                  next_type, next_at_iso, seconds_until);

    auto appendIsoArray = [&](const char* key, const String& csv) {
        n += snprintf(buf + n, sizeof(buf) - n, "\"%s\":[", key);
        int count = 0;
        if (csv.length() > 0) {
            char tmp[64]; strncpy(tmp, csv.c_str(), sizeof(tmp)); tmp[sizeof(tmp)-1] = 0;
            char* save; char* tok = strtok_r(tmp, ",", &save);
            while (tok && count < 3) {
                time_t t = (time_t)strtoll(tok, nullptr, 10);
                char iso[24]; scheduler_formatIso8601UTC(t, iso, sizeof(iso));
                n += snprintf(buf + n, sizeof(buf) - n, "%s\"%s\"", (count ? "," : ""), iso);
                ++count; tok = strtok_r(nullptr, ",", &save);
            }
        }
        n += snprintf(buf + n, sizeof(buf) - n, "]");
    };

    appendIsoArray("last_three_wakes_utc",  wakes_csv);
    n += snprintf(buf + n, sizeof(buf) - n, ", ");
    appendIsoArray("next_three_sleeps_utc", sleeps_csv);
    n += snprintf(buf + n, sizeof(buf) - n, "}");

    if (n <= 0) { server.send(500, "application/json", "{\"error\":\"formatting\"}"); return; }
    server.send(200, "application/json", buf);
}

static void handleUptime_LegacyOnly() {
    unsigned long total = millis() / 1000UL;
    unsigned long d = total / 86400UL; total %= 86400UL;
    unsigned long h = total / 3600UL;  total %= 3600UL;
    unsigned long m = total / 60UL;    unsigned long s = total % 60UL;

    char human[64]; size_t pos = 0;
    if (d > 0) pos += snprintf(human + pos, sizeof(human) - pos, "%lud ", d);
    if (d > 0 || h > 0) pos += snprintf(human + pos, sizeof(human) - pos, "%luh ", h);
    if (d > 0 || h > 0 || m > 0) pos += snprintf(human + pos, sizeof(human) - pos, "%lum ", m);
    snprintf(human + pos, sizeof(human) - pos, "%lus", s);

    char buf[192];
    int n = snprintf(buf, sizeof(buf),
                     "{\"uptime\": %lu, \"uptime_human\": \"%s\", \"days\": %lu, \"hours\": %lu, \"minutes\": %lu, \"seconds\": %lu}",
                     (millis() / 1000UL), human, d, h, m, s);
    if (n < 0) { server.send(500, "application/json", "{\"error\":\"formatting\"}"); return; }
    server.send(200, "application/json", buf);
}

static void handleStatus() {
    time_t now = time(nullptr);
    char now_iso[24]; scheduler_formatIso8601UTC(now, now_iso, sizeof(now_iso));
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

    String wakes_csv  = g_prefs.isKey(PREF_KEY_LAST_WAKES)  ? g_prefs.getString(PREF_KEY_LAST_WAKES,  "") : String("");
    String sleeps_csv = g_prefs.isKey(PREF_KEY_NEXT_SLEEPS) ? g_prefs.getString(PREF_KEY_NEXT_SLEEPS, "") : String("");

    char buf[1536]; int n = 0;
    n += snprintf(buf + n, sizeof(buf) - n, "{\"now_utc\": \"%s\", ", now_iso);
    char last_check_iso[24]; scheduler_formatIso8601UTC(g_last_ntp_check_utc, last_check_iso, sizeof(last_check_iso));
    n += snprintf(buf + n, sizeof(buf) - n, "\"last_ntp_check_utc\": \"%s\", ", last_check_iso);
    char now_local[32]; scheduler_formatIso8601Local(now, now_local, sizeof(now_local));
    char last_check_local[32]; scheduler_formatIso8601Local(g_last_ntp_check_utc, last_check_local, sizeof(last_check_local));
    n += snprintf(buf + n, sizeof(buf) - n, "\"now_local\": \"%s\", ", now_local);
    n += snprintf(buf + n, sizeof(buf) - n, "\"last_ntp_check_local\": \"%s\", ", last_check_local);
    n += snprintf(buf + n, sizeof(buf) - n,
                  "\"today\": {\"civil_dawn_utc\": \"%s\", \"civil_dusk_utc\": \"%s\"}, ",
                  tdawn_iso, tdusk_iso);
    char tdawn_local[32], tdusk_local[32];
    scheduler_formatIso8601Local(g_today_dawn_utc, tdawn_local, sizeof(tdawn_local));
    scheduler_formatIso8601Local(g_today_dusk_utc, tdusk_local, sizeof(tdusk_local));
    n += snprintf(buf + n, sizeof(buf) - n,
                  "\"today_local\": {\"civil_dawn_local\": \"%s\", \"civil_dusk_local\": \"%s\"}, ",
                  tdawn_local, tdusk_local);
    n += snprintf(buf + n, sizeof(buf) - n,
                  "\"boot_count\": %lu, \"schedule_basis\": \"civil_twilight_-6deg\", \"location\": {\"lat\": %.5f, \"lon\": %.5f}, ",
                  (unsigned long)g_boot_count, (float)LAT, (float)LON);
    n += snprintf(buf + n, sizeof(buf) - n,
                  "\"tomorrow\": {\"civil_dawn_utc\": \"%s\", \"civil_dusk_utc\": \"%s\"}, \"mode\": \"%s\", ",
                  mdawn_iso, mdusk_iso, mode);
    char mdawn_local[32], mdusk_local[32];
    scheduler_formatIso8601Local(g_tomorrow_dawn_utc, mdawn_local, sizeof(mdawn_local));
    scheduler_formatIso8601Local(g_tomorrow_dusk_utc, mdusk_local, sizeof(mdusk_local));
    n += snprintf(buf + n, sizeof(buf) - n,
                  "\"tomorrow_local\": {\"civil_dawn_local\": \"%s\", \"civil_dusk_local\": \"%s\"}, ",
                  mdawn_local, mdusk_local);
    n += snprintf(buf + n, sizeof(buf) - n,
                  "\"next_event\": {\"type\": \"%s\", \"at_utc\": \"%s\", \"seconds_until\": %u}, ",
                  next_type, next_at_iso, seconds_until);
    char next_at_local[32]; scheduler_formatIso8601Local(next_at, next_at_local, sizeof(next_at_local));
    n += snprintf(buf + n, sizeof(buf) - n,
                  "\"next_event_local\": {\"type\": \"%s\", \"at_local\": \"%s\"}, ",
                  next_type, next_at_local);
    n += snprintf(buf + n, sizeof(buf) - n, "\"timezone\": {\"posix\": \"%s\"}, ", LOCAL_TZ);

    auto appendIsoArray = [&](const char* key, const String& csv) {
        n += snprintf(buf + n, sizeof(buf) - n, "\"%s\":[", key);
        int count = 0;
        if (csv.length() > 0) {
            char tmp[64]; strncpy(tmp, csv.c_str(), sizeof(tmp)); tmp[sizeof(tmp)-1] = 0;
            char* save; char* tok = strtok_r(tmp, ",", &save);
            while (tok && count < 3) {
                time_t t = (time_t)strtoll(tok, nullptr, 10);
                char iso[24]; scheduler_formatIso8601UTC(t, iso, sizeof(iso));
                n += snprintf(buf + n, sizeof(buf) - n, "%s\"%s\"", (count ? "," : ""), iso);
                ++count; tok = strtok_r(nullptr, ",", &save);
            }
        }
        n += snprintf(buf + n, sizeof(buf) - n, "]");
    };
    auto appendIsoArrayLocal = [&](const char* key, const String& csv) {
        n += snprintf(buf + n, sizeof(buf) - n, ", \"%s\":[", key);
        int count = 0;
        if (csv.length() > 0) {
            char tmp[64]; strncpy(tmp, csv.c_str(), sizeof(tmp)); tmp[sizeof(tmp)-1] = 0;
            char* save; char* tok = strtok_r(tmp, ",", &save);
            while (tok && count < 3) {
                time_t t = (time_t)strtoll(tok, nullptr, 10);
                char iso[32]; scheduler_formatIso8601Local(t, iso, sizeof(iso));
                n += snprintf(buf + n, sizeof(buf) - n, "%s\"%s\"", (count ? "," : ""), iso);
                ++count; tok = strtok_r(nullptr, ",", &save);
            }
        }
        n += snprintf(buf + n, sizeof(buf) - n, "]");
    };

    appendIsoArray("last_three_wakes_utc",  wakes_csv);
    n += snprintf(buf + n, sizeof(buf) - n, ", ");
    appendIsoArray("next_three_sleeps_utc", sleeps_csv);
    appendIsoArrayLocal("last_three_wakes_local",  wakes_csv);
    appendIsoArrayLocal("next_three_sleeps_local", sleeps_csv);

    // Stream state (populated by StreamServer module)
    char stream_url[64];
    snprintf(stream_url, sizeof(stream_url), "http://%s:%d/stream",
             WiFi.localIP().toString().c_str(), (int)STREAM_PORT);
    n += snprintf(buf + n, sizeof(buf) - n,
                  ", \"stream\": {\"url\": \"%s\", \"active\": %s, \"connect_count\": %lu}",
                  stream_url,
                  g_stream_active ? "true" : "false",
                  (unsigned long)g_stream_connect_count);

    n += snprintf(buf + n, sizeof(buf) - n, "}");

    if (n <= 0) { server.send(500, "application/json", "{\"error\":\"formatting\"}"); return; }
    server.send(200, "application/json", buf);
}

// ------------------------------------------------------------
// setup / loop
// ------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(200);
    LOG_NL();
    logbuf_init();
    LOGI("Booting ESP32 Audio Streamer\n");

    appState_init();
    runtimeSettings_load();
    setenv("TZ", LOCAL_TZ, 1); tzset();

#if ENABLE_BROWNOUT_DISABLE
    LOGI("[PMIC] Brownout detector disabled (workaround enabled)\n");
    disableBrownout();
#else
    LOGI("[PMIC] Brownout workaround disabled (leaving detector enabled)\n");
#endif

    connectWiFiBlocking();
    // Apply persisted Wi-Fi TX power (runtimeSettings_load ran before connect,
    // but connectWiFiBlocking uses the compile-time default; re-apply now).
    {
        wifi_power_t txp = mapTxPowerDbm((int)g_runtime_settings.wifi_tx_power_dbm);
        WiFi.setTxPower(txp);
        LOGI("[WiFi] boot TX power applied: %d dBm\n", (int)g_runtime_settings.wifi_tx_power_dbm);
    }
    scheduler_maybeSyncNtp();

    time_t now = time(nullptr);
    if (scheduler_timeIsValid()) {
        scheduler_ensureSchedule(now);
        if (now >= g_today_dawn_utc && now < g_today_dusk_utc) {
            scheduler_pushCsvEpochRolling(PREF_KEY_LAST_WAKES, now);
            scheduler_refreshNextSleeps(g_today_dusk_utc, g_tomorrow_dusk_utc);
            g_last_mode = 1;
        } else {
            time_t nd = scheduler_nextCivilDawnAfter(now);
            if (nd > now) {
                LOGI("[MODE] Night after NTP; scheduling deep sleep until next dawn (%ld)\n", (long)nd);
#if !defined(ENABLE_DEEP_SLEEP) || ENABLE_DEEP_SLEEP
                scheduler_deepSleepUntil(nd);
#else
                LOGI("[MODE] Deep sleep disabled (ENABLE_DEEP_SLEEP=0); staying awake despite night.\n");
#endif
            } else {
                LOGW("[MODE] Night after NTP but could not find future dawn; staying awake\n");
            }
        }
    } else {
        LOGW("[NTP] Time invalid at boot; will retry in loop (no sleep decisions yet)\n");
    }

    if (!audioPipeline_init()) {
        LOGE("[I2S] init failed; /stream will 503\n");
    } else {
        // Apply persisted HPF config over the compile-time defaults used by init.
        audioPipeline_setHpfConfig(g_runtime_settings.hpf_enabled,
                                   (int)g_runtime_settings.hpf_cutoff_hz);
    }

    // Start dedicated stream server on STREAM_PORT (default 81)
    streamServer_init();

    // Control-plane routes (port SERVER_PORT, default 80)
    server.on("/", HTTP_GET, handleRoot);
    server.on("/uptime", HTTP_GET, handleUptime_LegacyOnly);
    server.on("/status", HTTP_GET, handleStatus);
    httpControl_registerRoutes(server);
    server.begin();
    LOGI("HTTP control server started on :%d\n", (int)SERVER_PORT);
    LOGI("Audio stream available on :%d/stream\n", (int)STREAM_PORT);
}

void loop() {
    server.handleClient();

    uint32_t now_ms = millis();
    if (now_ms - g_next_ntp_retry_ms > 60000) {
        g_next_ntp_retry_ms = now_ms;
        scheduler_maybeSyncNtp();
    }

    time_t nowt = time(nullptr);
    if (scheduler_timeIsValid()) {
        scheduler_ensureSchedule(nowt);
        scheduler_trySleepIfNight(nowt);
    }

    delay(2);
}
