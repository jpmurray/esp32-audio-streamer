// Scheduler.cpp — NTP sync, civil dawn/dusk solar computation, and sleep control.

#include "Scheduler.h"
#include "AppState.h"
#include "AudioPipeline.h"   // audioPipeline_stop()

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

// ------------------------------------------------------------
// Logging
// ------------------------------------------------------------
#ifndef LOG_LEVEL
#define LOG_LEVEL 2
#endif

#if LOG_LEVEL >= 3
#define LOGD(fmt, ...) Serial.printf("[D] " fmt, ##__VA_ARGS__)
#else
#define LOGD(...) do {} while (0)
#endif

#if LOG_LEVEL >= 2
#define LOGI(fmt, ...) Serial.printf("[I] " fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) Serial.printf("[W] " fmt, ##__VA_ARGS__)
#else
#define LOGI(...) do {} while (0)
#define LOGW(...) do {} while (0)
#endif

#if LOG_LEVEL >= 1
#define LOGE(fmt, ...) Serial.printf("[E] " fmt, ##__VA_ARGS__)
#else
#define LOGE(...) do {} while (0)
#endif

// ------------------------------------------------------------
// Compile-time configuration
// ------------------------------------------------------------
#ifndef LOCAL_TZ
#define LOCAL_TZ "UTC0"
#endif
#ifndef LAT
#define LAT 51.4630911
#endif
#ifndef LON
#define LON -3.1678763
#endif

static const double kLat = (double)LAT;
static const double kLon = (double)LON;

// ------------------------------------------------------------
// Internal helpers
// ------------------------------------------------------------
static double deg2rad(double d) { return d * (PI / 180.0); }

static double jdFromDateUTC(int y, int m, int d) {
    if (m <= 2) { y -= 1; m += 12; }
    int A = y / 100;
    int B = 2 - A + A / 4;
    return floor(365.25 * (y + 4716)) + floor(30.6001 * (m + 1)) + d + B - 1524.5;
}

static time_t epochFromJD(double jd) {
    double days = jd - 2440587.5;
    double secs = days * 86400.0;
    if (secs < 0) return 0;
    return (time_t)(secs + 0.5);
}

struct CivilTimes { time_t dawn; time_t dusk; bool valid; };

static CivilTimes computeCivilTimesUTC_forDay(int y, int m, int d,
                                               double lat_deg, double lon_deg) {
    CivilTimes out = {0, 0, true};
    const double J2000 = 2451545.0;
    const double h0    = deg2rad(-6.0);
    const double phi   = deg2rad(lat_deg);
    const double Lw    = -lon_deg;

    double JD    = jdFromDateUTC(y, m, d);
    double n     = round((JD - J2000 - 0.0009) - (Lw / 360.0));
    double Jstar = J2000 + 0.0009 + (Lw / 360.0) + n;
    double M     = deg2rad(357.5291 + 0.98560028 * (Jstar - J2000));
    double C     = deg2rad(1.9148) * sin(M) + deg2rad(0.0200) * sin(2.0 * M) + deg2rad(0.0003) * sin(3.0 * M);
    double lambda = M + C + deg2rad(102.9372) + PI;
    double delta  = asin(sin(lambda) * sin(deg2rad(23.44)));
    double Jtransit = Jstar + 0.0053 * sin(M) - 0.0069 * sin(2.0 * lambda);
    double cosH0    = (sin(h0) - sin(phi) * sin(delta)) / (cos(phi) * cos(delta));

    if (cosH0 > 1.0 || cosH0 < -1.0) { out.valid = false; return out; }

    double H0    = acos(cosH0);
    double Jrise = Jtransit - H0 / (2.0 * PI);
    double Jset  = Jtransit + H0 / (2.0 * PI);

    time_t rise      = epochFromJD(Jrise);
    time_t set       = epochFromJD(Jset);
    time_t day_start = epochFromJD(JD);
    time_t day_end   = day_start + 86400;

    while (rise < day_start)  rise += 86400;
    while (rise >= day_end)   rise -= 86400;
    while (set  < day_start)  set  += 86400;
    while (set  >= day_end)   set  -= 86400;

    out.dawn = rise;
    out.dusk = set;

#if LOG_LEVEL >= 3
    char dawn_iso[24], dusk_iso[24];
    scheduler_formatIso8601UTC(out.dawn, dawn_iso, sizeof(dawn_iso));
    scheduler_formatIso8601UTC(out.dusk, dusk_iso, sizeof(dusk_iso));
    Serial.printf("[D] Civil times UTC for %04d-%02d-%02d lat=%.5f lon=%.5f -> dawn=%s dusk=%s\n",
                  y, m, d, (float)lat_deg, (float)lon_deg, dawn_iso, dusk_iso);
#endif
    return out;
}

static void computeTodayTomorrow(time_t now,
                                  time_t* tdawn, time_t* tdusk,
                                  time_t* mdawn, time_t* mdusk) {
    struct tm tm_utc; gmtime_r(&now, &tm_utc);
    CivilTimes t = computeCivilTimesUTC_forDay(
        tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday, kLat, kLon);

    time_t tmp = now + 86400;
    struct tm tm2; gmtime_r(&tmp, &tm2);
    CivilTimes t2 = computeCivilTimesUTC_forDay(
        tm2.tm_year + 1900, tm2.tm_mon + 1, tm2.tm_mday, kLat, kLon);

    if (tdawn) *tdawn = t.dawn;
    if (tdusk) *tdusk = t.dusk;
    if (mdawn) *mdawn = t2.dawn;
    if (mdusk) *mdusk = t2.dusk;
}

static uint32_t ymdFromUtc(time_t t) {
    struct tm tm_utc; gmtime_r(&t, &tm_utc);
    return (uint32_t)(tm_utc.tm_year + 1900) * 10000u
         + (uint32_t)(tm_utc.tm_mon + 1)     * 100u
         + (uint32_t)tm_utc.tm_mday;
}

static bool waitForNtp(uint32_t timeout_ms) {
    uint32_t start = millis();
    while ((millis() - start) < timeout_ms) {
        if (scheduler_timeIsValid()) return true;
        delay(200);
    }
    return scheduler_timeIsValid();
}

static void gracefulShutdown() {
    audioPipeline_stop();
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
}

// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------
bool scheduler_timeIsValid() {
    return time(nullptr) > 1577836800; // 2020-01-01
}

void scheduler_formatIso8601UTC(time_t t, char* out, size_t out_sz) {
    if (t <= 0) { snprintf(out, out_sz, "null"); return; }
    struct tm tm_utc; gmtime_r(&t, &tm_utc);
    snprintf(out, out_sz, "%04d-%02d-%02dT%02d:%02d:%02dZ",
             tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday,
             tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec);
}

void scheduler_formatIso8601Local(time_t t, char* out, size_t out_sz) {
    if (t <= 0) { snprintf(out, out_sz, "null"); return; }
    struct tm tm_loc; localtime_r(&t, &tm_loc);
    char base[24];
    snprintf(base, sizeof(base), "%04d-%02d-%02dT%02d:%02d:%02d",
             tm_loc.tm_year + 1900, tm_loc.tm_mon + 1, tm_loc.tm_mday,
             tm_loc.tm_hour, tm_loc.tm_min, tm_loc.tm_sec);
    char zraw[8] = {0}, zfmt[8] = {0};
    size_t n = strftime(zraw, sizeof(zraw), "%z", &tm_loc);
    if (n >= 5) {
        zfmt[0] = zraw[0]; zfmt[1] = zraw[1]; zfmt[2] = zraw[2];
        zfmt[3] = ':';
        zfmt[4] = zraw[3]; zfmt[5] = zraw[4]; zfmt[6] = 0;
    } else {
        strncpy(zfmt, "+00:00", sizeof(zfmt)); zfmt[sizeof(zfmt)-1] = 0;
    }
    snprintf(out, out_sz, "%s%s", base, zfmt);
}

void scheduler_maybeSyncNtp() {
    time_t now = time(nullptr);
    if (!scheduler_timeIsValid() || (now - g_last_ntp_sync_utc) > 86400) {
        g_last_ntp_check_utc = now;
        LOGI("[NTP] Sync starting...\n");
        configTzTime(LOCAL_TZ, "pool.ntp.org", "time.nist.gov");
        if (waitForNtp(15000)) {
            g_last_ntp_sync_utc = time(nullptr);
            LOGI("[NTP] Sync ok: %ld\n", (long)g_last_ntp_sync_utc);
        } else {
            LOGW("[NTP] Sync failed, will retry later\n");
        }
    }
}

void scheduler_ensureSchedule(time_t now) {
    uint32_t ymd = ymdFromUtc(now);
    if (g_last_compute_ymd != ymd || g_today_dawn_utc == 0 || g_today_dusk_utc == 0) {
        computeTodayTomorrow(now,
                             &g_today_dawn_utc, &g_today_dusk_utc,
                             &g_tomorrow_dawn_utc, &g_tomorrow_dusk_utc);
        g_last_compute_ymd = ymd;
        scheduler_refreshNextSleeps(g_today_dusk_utc, g_tomorrow_dusk_utc);
        LOGI("[SCHED] Recomputed dawn/dusk. today: %ld/%ld, tomorrow: %ld/%ld\n",
             (long)g_today_dawn_utc, (long)g_today_dusk_utc,
             (long)g_tomorrow_dawn_utc, (long)g_tomorrow_dusk_utc);
#if LOG_LEVEL >= 3
        char now_iso[24], td_iso[24], ts_iso[24], nd_iso[24], ns_iso[24];
        scheduler_formatIso8601UTC(now,                  now_iso, sizeof(now_iso));
        scheduler_formatIso8601UTC(g_today_dawn_utc,     td_iso,  sizeof(td_iso));
        scheduler_formatIso8601UTC(g_today_dusk_utc,     ts_iso,  sizeof(ts_iso));
        scheduler_formatIso8601UTC(g_tomorrow_dawn_utc,  nd_iso,  sizeof(nd_iso));
        scheduler_formatIso8601UTC(g_tomorrow_dusk_utc,  ns_iso,  sizeof(ns_iso));
        Serial.printf("[D] now=%s lat=%.6f lon=%.6f today.dawn=%s today.dusk=%s tomorrow.dawn=%s tomorrow.dusk=%s\n",
                      now_iso, (float)kLat, (float)kLon, td_iso, ts_iso, nd_iso, ns_iso);
#endif
    }
}

time_t scheduler_nextCivilDawnAfter(time_t now) {
    for (int k = 0; k < 4; ++k) {
        time_t t = now + (time_t)k * 86400;
        struct tm tm_utc; gmtime_r(&t, &tm_utc);
        CivilTimes c = computeCivilTimesUTC_forDay(
            tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday, kLat, kLon);
        if (!c.valid) continue;
        if (c.dawn > now) return c.dawn;
    }
    return 0;
}

void scheduler_deepSleepUntil(time_t target) {
    if (target <= 0) return;
    time_t now = time(nullptr);
    int64_t sec = (int64_t)target - (int64_t)now;
    if (sec < 5) sec = 5;
#if !defined(ENABLE_DEEP_SLEEP) || ENABLE_DEEP_SLEEP
    LOGI("[SLEEP] Deep sleeping for %lld sec until %ld\n", (long long)sec, (long)target);
    esp_sleep_enable_timer_wakeup((uint64_t)sec * 1000000ULL);
    Serial.flush(); delay(50);
    gracefulShutdown();
    delay(200);
    esp_deep_sleep_start();
#else
    static time_t s_last_logged_target = 0;
    if (target != s_last_logged_target) {
        LOGI("[SLEEP] Deep sleep disabled (ENABLE_DEEP_SLEEP=0); would have slept %lld sec. Continuing.\n", (long long)sec);
        s_last_logged_target = target;
    }
#endif
}

void scheduler_trySleepIfNight(time_t now) {
    if (!scheduler_timeIsValid()) return;
    if (millis() - g_boot_ms < 20000) return;
    scheduler_ensureSchedule(now);
    if (now >= g_today_dusk_utc || now < g_today_dawn_utc) {
        time_t nd = scheduler_nextCivilDawnAfter(now);
        if (nd > now) scheduler_deepSleepUntil(nd);
    }
}

void scheduler_pushCsvEpochRolling(const char* key, time_t value) {
    if (value <= 0 || !g_prefs_inited) return;
    char buf[64];
    String cur = g_prefs.isKey(key) ? g_prefs.getString(key, "") : String("");
    char tmp[64]; tmp[0] = '\0';
    if (cur.length() > 0 && cur.length() < sizeof(tmp))
        strncpy(tmp, cur.c_str(), sizeof(tmp));
    time_t vals[4] = {0, 0, 0, 0}; int count = 0;
    if (tmp[0]) {
        char* save; char* tok = strtok_r(tmp, ",", &save);
        while (tok && count < 3) { vals[count++] = (time_t)strtoll(tok, nullptr, 10); tok = strtok_r(nullptr, ",", &save); }
    }
    vals[count++] = value;
    if (count > 3) {
        vals[0] = vals[count-3]; vals[1] = vals[count-2]; vals[2] = vals[count-1]; count = 3;
    }
    int n = snprintf(buf, sizeof(buf),
                     (count == 3 ? "%lld,%lld,%lld" : (count == 2 ? "%lld,%lld" : "%lld")),
                     (long long)vals[0],
                     (count >= 2 ? (long long)vals[1] : 0LL),
                     (count >= 3 ? (long long)vals[2] : 0LL));
    if (n > 0) g_prefs.putString(key, buf);
}

void scheduler_setCsvEpochList(const char* key, time_t a, time_t b, time_t c) {
    if (!g_prefs_inited) return;
    char buf[64];
    if (a && b && c)  snprintf(buf, sizeof(buf), "%lld,%lld,%lld", (long long)a, (long long)b, (long long)c);
    else if (a && b)  snprintf(buf, sizeof(buf), "%lld,%lld",      (long long)a, (long long)b);
    else if (a)       snprintf(buf, sizeof(buf), "%lld",           (long long)a);
    else              buf[0] = 0;
    g_prefs.putString(key, buf);
}

void scheduler_refreshNextSleeps(time_t today_dusk, time_t tomorrow_dusk) {
    time_t d2 = tomorrow_dusk;
    if (scheduler_timeIsValid() && tomorrow_dusk) {
        time_t t = tomorrow_dusk + 86400;
        struct tm tm2; gmtime_r(&t, &tm2);
        CivilTimes t2 = computeCivilTimesUTC_forDay(
            tm2.tm_year + 1900, tm2.tm_mon + 1, tm2.tm_mday, kLat, kLon);
        d2 = t2.dusk;
    }
    scheduler_setCsvEpochList(PREF_KEY_NEXT_SLEEPS, today_dusk, tomorrow_dusk, d2);
}
