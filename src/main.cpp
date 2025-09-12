#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <Preferences.h>
#ifndef PI
#define PI 3.14159265358979323846
#endif
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/i2s.h"
#include "esp_wifi.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

// ------------------------------------------------------------
// Logging (compile-time): -D LOG_LEVEL=1/2/3
// 1 = errors only, 2 = info+warn+error (default), 3 = verbose/debug
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

#if LOG_LEVEL >= 3
#define LOG_DOT() Serial.print('.')
#define LOG_NL()  Serial.println()
#else
#define LOG_DOT() do {} while (0)
#define LOG_NL()  do {} while (0)
#endif

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
// Stream format toggle (set via extra configs/local_env.ini)
#ifndef STREAM_WAV_ENABLE
#define STREAM_WAV_ENABLE 0   // 0 = raw audio/L16, 1 = audio/x-wav with header
#endif

static WebServer server(SERVER_PORT);
static Preferences g_prefs;
static const char* const PREF_NS = "sched";
static const char* const PREF_KEY_LAST_WAKES = "last_wakes";      // <= 15 chars
static const char* const PREF_KEY_NEXT_SLEEPS = "next_sleeps";    // <= 15 chars
static bool g_prefs_inited = false;
static bool g_i2s_ok = false;
static bool g_rb_ok = false;

#ifndef SAMPLE_RATE_HZ
#define SAMPLE_RATE_HZ 48000
#endif
#ifndef CONVERT_SHIFT
#define CONVERT_SHIFT 11
#endif
#ifndef HPF_ENABLE
#define HPF_ENABLE 1
#endif
#ifndef HPF_CUTOFF_HZ
#define HPF_CUTOFF_HZ 100
#endif
#ifndef CHUNK_FRAMES
#define CHUNK_FRAMES 1024
#endif
#ifndef RB_CAPACITY_BYTES
#define RB_CAPACITY_BYTES (64 * 1024)
#endif
#ifndef USE_RIGHT_CHANNEL
#define USE_RIGHT_CHANNEL 1
#endif
#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

#ifndef PIN_I2S_WS
#define PIN_I2S_WS 25
#endif
#ifndef PIN_I2S_SCK
#define PIN_I2S_SCK 33
#endif
#ifndef PIN_I2S_SD
#define PIN_I2S_SD 32
#endif
#ifndef I2S_PORT_NUM
#define I2S_PORT_NUM 0
#endif

#ifndef LAT
#define LAT 51.4630911
#endif
#ifndef LON
#define LON -3.1678763
#endif

static bool g_hpf_enabled = (HPF_ENABLE != 0);
static float g_hpf_R = 0.0f;           // computed once from cutoff
static float g_hpf_prev_x = 0.0f;      // kept for reference/logging, not used in fast path
static float g_hpf_prev_y = 0.0f;
static int16_t g_hpf_prev_x_i16 = 0;   // fast integer path state
static int32_t g_hpf_prev_y_i32 = 0;   // fast integer path state
static int32_t g_hpf_a_q15 = 0;        // R in Q15

// I2S microphone configuration (INMP44/INMP441 style)
static const i2s_port_t I2S_PORT = (i2s_port_t)I2S_PORT_NUM;
// Pins are configurable via build flags PIN_I2S_WS, PIN_I2S_SCK, PIN_I2S_SD

static const int BITS_PER_SAMPLE = 32;     // Read 32-bit from I2S mic
static const int BYTES_PER_SAMPLE_IN = BITS_PER_SAMPLE / 8; // 4 bytes input
static const int OUT_BITS = 16;            // Stream 16-bit PCM

// DMA: 4 buffers × 1024 samples (per channel). Adjust if needed.
#ifndef DMA_BUF_COUNT_CFG
#define DMA_BUF_COUNT_CFG 4
#endif
static const int DMA_BUF_COUNT = DMA_BUF_COUNT_CFG;
static const int DMA_BUF_LEN   = 1024;     // frames per DMA buffer (ESP-IDF max is 1024)

// Convert mic channel: strap typically LEFT; controlled by USE_RIGHT_CHANNEL flag
#if USE_RIGHT_CHANNEL
static const i2s_channel_fmt_t I2S_CHAN_FMT = I2S_CHANNEL_FMT_ONLY_RIGHT;
#else
static const i2s_channel_fmt_t I2S_CHAN_FMT = I2S_CHANNEL_FMT_ONLY_LEFT;
#endif

// Temporary scratch buffers (producer side uses its own local buffers)
static int32_t i2s_in32[1024];         // legacy scratch, kept for size reference
static int16_t pcm16_out[1024];        // legacy scratch, kept for size reference

// Ring buffer for decoupling I2S and streamer
static RingbufHandle_t g_ringbuf = nullptr;
static const size_t RINGBUF_CAPACITY_BYTES = RB_CAPACITY_BYTES; // configurable via build flag
static TaskHandle_t g_i2s_task = nullptr;

// ----- Scheduling / RTC retained state -----
RTC_DATA_ATTR uint32_t g_boot_count = 0;
RTC_DATA_ATTR time_t g_today_dawn_utc = 0;
RTC_DATA_ATTR time_t g_today_dusk_utc = 0;
RTC_DATA_ATTR time_t g_tomorrow_dawn_utc = 0;
RTC_DATA_ATTR time_t g_tomorrow_dusk_utc = 0;
RTC_DATA_ATTR uint32_t g_last_compute_ymd = 0; // YYYYMMDD UTC
RTC_DATA_ATTR time_t g_last_ntp_sync_utc = 0;
RTC_DATA_ATTR time_t g_last_ntp_check_utc = 0;
RTC_DATA_ATTR uint8_t g_last_mode = 0; // 0=unknown/night,1=day

static uint32_t g_boot_ms = 0; // monotonic since boot for sleep guard

static uint32_t g_next_ntp_retry_ms = 0; // millis schedule while awake

// Location
static const double kLat = (double)LAT;
static const double kLon = (double)LON;

// Dynamic index page is rendered in handleRoot() based on build flags
// (STREAM_WAV_ENABLE, SAMPLE_RATE_HZ)
// Old static HTML kept here for reference
/* static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>ESP32 Audio Streamer</title>
    <style>
      body { font-family: -apple-system, system-ui, Roboto, Arial, sans-serif; margin: 2rem; }
      .card { max-width: 680px; padding: 1rem 1.25rem; border: 1px solid #ddd; border-radius: 8px; }
      code { background: #f6f8fa; padding: 0.2rem 0.4rem; border-radius: 4px; }
      .warn { color: #b35600; }
    </style>
  </head>
  <body>
    <h1>ESP32 Audio Streamer</h1>
    <div class="card">
      <p>Status: <strong>Server is running</strong></p>
      <p>
        Once audio streaming is enabled, this player will work:
      </p>
      <audio id="player" controls preload="none" src="/stream"></audio>
      <p>
        Streaming raw PCM: <code>audio/L16</code>, mono 48 kHz.
      </p>
    </div>
  </body>
</html>
)HTML"; */

static void handleRoot() {
  String html;
  html.reserve(1024);
  html += F("<!doctype html>\n");
  html += F("<html>\n  <head>\n    <meta charset=\"utf-8\" />\n    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />\n    <title>ESP32 Audio Streamer</title>\n    <style>\n      body { font-family: -apple-system, system-ui, Roboto, Arial, sans-serif; margin: 2rem; }\n      .card { max-width: 680px; padding: 1rem 1.25rem; border: 1px solid #ddd; border-radius: 8px; }\n      code { background: #f6f8fa; padding: 0.2rem 0.4rem; border-radius: 4px; }\n      .warn { color: #b35600; }\n    </style>\n  </head>\n  <body>\n    <h1>ESP32 Audio Streamer</h1>\n    <div class=\"card\">\n      <p>Status: <strong>Server is running</strong></p>\n      <p>\n        Once audio streaming is enabled, this player will work:\n      </p>\n      <audio id=\"player\" controls preload=\"none\" src=\"/stream\"></audio>\n      <p>\n        ");
  if (STREAM_WAV_ENABLE) {
    html += F("Streaming WAV: <code>audio/x-wav</code>, mono ");
  } else {
    html += F("Streaming raw PCM: <code>audio/L16</code>, mono ");
  }
  html += String((int)SAMPLE_RATE_HZ);
  html += F(" Hz.\n      </p>\n    </div>\n  </body>\n</html>\n");

  server.send(200, "text/html", html);
}

static void handleStream() {
  if (!g_rb_ok) { server.send(503, "text/plain", "Ring buffer not ready"); return; }

  // Start a raw PCM stream: HTTP/1.1 without Content-Length; keep connection open
  WiFiClient client = server.client();
  client.setNoDelay(true);
  client.print("HTTP/1.1 200 OK\r\n");
  if (STREAM_WAV_ENABLE) {
    client.print("Content-Type: audio/x-wav\r\n");
  } else {
    client.print("Content-Type: audio/L16; rate=");
    client.print(SAMPLE_RATE_HZ);
    client.print("; channels=1\r\n");
  }
  client.print("Cache-Control: no-store\r\nConnection: close\r\n\r\n");

  // If WAV is enabled, send a minimal header with unknown size (streaming)
  if (STREAM_WAV_ENABLE) {
    const uint32_t sampleRate = (uint32_t)SAMPLE_RATE_HZ;
    const uint16_t channels = 1;
    const uint16_t bitsPerSample = 16;
    const uint32_t byteRate = sampleRate * channels * (bitsPerSample / 8);
    const uint16_t blockAlign = channels * (bitsPerSample / 8);
    const uint32_t riffSize = 0xFFFFFFFF;   // unknown/streaming
    const uint32_t dataSize = 0xFFFFFFFF;   // unknown/streaming

    uint8_t hdr[44];
    // RIFF chunk descriptor
    hdr[0]='R'; hdr[1]='I'; hdr[2]='F'; hdr[3]='F';
    hdr[4]= (uint8_t)(riffSize & 0xFF);
    hdr[5]= (uint8_t)((riffSize >> 8) & 0xFF);
    hdr[6]= (uint8_t)((riffSize >> 16) & 0xFF);
    hdr[7]= (uint8_t)((riffSize >> 24) & 0xFF);
    hdr[8]='W'; hdr[9]='A'; hdr[10]='V'; hdr[11]='E';
    // fmt subchunk
    hdr[12]='f'; hdr[13]='m'; hdr[14]='t'; hdr[15]=' ';
    const uint32_t subchunk1Size = 16; // PCM
    hdr[16]= (uint8_t)(subchunk1Size & 0xFF);
    hdr[17]= (uint8_t)((subchunk1Size >> 8) & 0xFF);
    hdr[18]= (uint8_t)((subchunk1Size >> 16) & 0xFF);
    hdr[19]= (uint8_t)((subchunk1Size >> 24) & 0xFF);
    const uint16_t audioFormat = 1; // PCM
    hdr[20]= (uint8_t)(audioFormat & 0xFF);
    hdr[21]= (uint8_t)((audioFormat >> 8) & 0xFF);
    hdr[22]= (uint8_t)(channels & 0xFF);
    hdr[23]= (uint8_t)((channels >> 8) & 0xFF);
    hdr[24]= (uint8_t)(sampleRate & 0xFF);
    hdr[25]= (uint8_t)((sampleRate >> 8) & 0xFF);
    hdr[26]= (uint8_t)((sampleRate >> 16) & 0xFF);
    hdr[27]= (uint8_t)((sampleRate >> 24) & 0xFF);
    hdr[28]= (uint8_t)(byteRate & 0xFF);
    hdr[29]= (uint8_t)((byteRate >> 8) & 0xFF);
    hdr[30]= (uint8_t)((byteRate >> 16) & 0xFF);
    hdr[31]= (uint8_t)((byteRate >> 24) & 0xFF);
    hdr[32]= (uint8_t)(blockAlign & 0xFF);
    hdr[33]= (uint8_t)((blockAlign >> 8) & 0xFF);
    hdr[34]= (uint8_t)(bitsPerSample & 0xFF);
    hdr[35]= (uint8_t)((bitsPerSample >> 8) & 0xFF);
    // data subchunk
    hdr[36]='d'; hdr[37]='a'; hdr[38]='t'; hdr[39]='a';
    hdr[40]= (uint8_t)(dataSize & 0xFF);
    hdr[41]= (uint8_t)((dataSize >> 8) & 0xFF);
    hdr[42]= (uint8_t)((dataSize >> 16) & 0xFF);
    hdr[43]= (uint8_t)((dataSize >> 24) & 0xFF);
    client.write(hdr, sizeof(hdr));
  }

  // Streaming loop: pull from ring buffer, optional HPF, write
  while (client.connected()) {
    size_t item_size = 0;
    int16_t* chunk = (int16_t*)xRingbufferReceive(g_ringbuf, &item_size, pdMS_TO_TICKS(1000));
    if (!chunk) {
      // No data available; keep trying
      yield();
      continue;
    }

    size_t frames = item_size / sizeof(int16_t);

    if (g_hpf_enabled) {
      for (size_t i = 0; i < frames; ++i) {
        int16_t x = chunk[i];
        int32_t yi = (int32_t)x - (int32_t)g_hpf_prev_x_i16 + (int32_t)((g_hpf_a_q15 * g_hpf_prev_y_i32) >> 15);
        g_hpf_prev_x_i16 = x;
        g_hpf_prev_y_i32 = yi;
        if (yi > 32767) yi = 32767;
        if (yi < -32768) yi = -32768;
        chunk[i] = (int16_t)yi;
      }
    }

    const uint8_t* p = reinterpret_cast<const uint8_t*>(chunk);
    size_t to_write = item_size;
    while (to_write > 0 && client.connected()) {
      size_t n = client.write(p, to_write);
      if (n == 0) { delay(1); }
      else { p += n; to_write -= n; }
      yield();
    }

    vRingbufferReturnItem(g_ringbuf, (void*)chunk);
    yield();
  }

  client.stop();
}

// ------------------- Time helpers -------------------
static inline bool timeIsValid() {
  time_t now = time(nullptr);
  return now > 1577836800; // 2020-01-01
}

static void formatIso8601UTC(time_t t, char* out, size_t out_sz) {
  if (t <= 0) { snprintf(out, out_sz, "null"); return; }
  struct tm tm_utc;
  gmtime_r(&t, &tm_utc);
  // 2023-01-02T03:04:05Z
  snprintf(out, out_sz, "%04d-%02d-%02dT%02d:%02d:%02dZ",
           tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday,
           tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec);
}

static uint32_t ymdFromUtc(time_t t) {
  struct tm tm_utc; gmtime_r(&t, &tm_utc);
  return (uint32_t)(tm_utc.tm_year + 1900) * 10000u + (uint32_t)(tm_utc.tm_mon + 1) * 100u + (uint32_t)tm_utc.tm_mday;
}

// ------------------- NOAA solar calc -------------------
static double deg2rad(double d) { return d * (PI / 180.0); }
static double rad2deg(double r) { return r * (180.0 / PI); }

static double clamp(double x, double a, double b) { if (x < a) return a; if (x > b) return b; return x; }

// Julian Day at 0h UTC for given Y-M-D
static double jdFromDateUTC(int y, int m, int d) {
  if (m <= 2) { y -= 1; m += 12; }
  int A = y / 100;
  int B = 2 - A + A / 4; // Gregorian calendar correction
  double JD = floor(365.25 * (y + 4716)) + floor(30.6001 * (m + 1)) + d + B - 1524.5;
  return JD;
}

static time_t epochFromJD(double jd) {
  // Unix epoch starts at JD 2440587.5
  double days = jd - 2440587.5;
  double secs = days * 86400.0;
  if (secs < 0) return 0;
  return (time_t)(secs + 0.5);
}

struct CivilTimes { time_t dawn; time_t dusk; bool valid; };

static CivilTimes computeCivilTimesUTC_forDay(int y, int m, int d, double lat_deg, double lon_deg) {
  CivilTimes out; out.dawn = 0; out.dusk = 0; out.valid = true;
  const double J2000 = 2451545.0;
  const double h0 = deg2rad(-6.0); // civil twilight center of Sun at -6°
  const double phi = deg2rad(lat_deg);

  // NOAA/USNO derivation expects Lw = -longitude (degrees, west negative)
  const double Lw = -lon_deg;

  // Julian Day at 0h UTC for the requested date
  double JD = jdFromDateUTC(y, m, d);

  // Approximate solar noon (transit) seed
  // n is the integer number of days since J2000 adjusted by longitude
  double n = round((JD - J2000 - 0.0009) - (Lw / 360.0));
  double Jstar = J2000 + 0.0009 + (Lw / 360.0) + n;

  // Solar mean anomaly (radians)
  double M = deg2rad(357.5291 + 0.98560028 * (Jstar - J2000));

  // Equation of center (radians)
  double C = deg2rad(1.9148) * sin(M) + deg2rad(0.0200) * sin(2.0 * M) + deg2rad(0.0003) * sin(3.0 * M);

  // Ecliptic longitude of the Sun (radians)
  double lambda = M + C + deg2rad(102.9372) + PI;

  // Solar declination (radians)
  double delta = asin(sin(lambda) * sin(deg2rad(23.44)));

  // Solar transit (Julian day)
  double Jtransit = Jstar + 0.0053 * sin(M) - 0.0069 * sin(2.0 * lambda);

  // Hour angle for given altitude (civil twilight)
  double cosH0 = (sin(h0) - sin(phi) * sin(delta)) / (cos(phi) * cos(delta));

  // Explicit polar day/night handling:
  //  - cosH0 > 1: Sun always below h0 → no dawn/dusk
  //  - cosH0 < -1: Sun always above h0 → no dusk/dawn
  if (cosH0 > 1.0 || cosH0 < -1.0) {
    out.valid = false;
    out.dawn = 0;
    out.dusk = 0;
    return out;
  }

  double H0 = acos(cosH0); // radians

  // Rise and set (Julian day)
  double Jrise = Jtransit - H0 / (2.0 * PI);
  double Jset  = Jtransit + H0 / (2.0 * PI);

  // Convert to Unix epoch
  time_t rise = epochFromJD(Jrise);
  time_t set  = epochFromJD(Jset);

  // Anchor results into the requested UTC date window [JD, JD+1)
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
  formatIso8601UTC(out.dawn, dawn_iso, sizeof(dawn_iso));
  formatIso8601UTC(out.dusk, dusk_iso, sizeof(dusk_iso));
  Serial.printf("[D] Civil times UTC for %04d-%02d-%02d at lat=%.5f lon=%.5f -> dawn=%s dusk=%s\n",
                y, m, d, (float)lat_deg, (float)lon_deg, dawn_iso, dusk_iso);
#endif
  return out;
}

static void computeTodayTomorrow(double lat, double lon, time_t now, time_t* tdawn, time_t* tdusk, time_t* mdawn, time_t* mdusk) {
  struct tm tm_utc; gmtime_r(&now, &tm_utc);
  int y = tm_utc.tm_year + 1900;
  int m = tm_utc.tm_mon + 1;
  int d = tm_utc.tm_mday;
  CivilTimes t = computeCivilTimesUTC_forDay(y, m, d, lat, lon);
  // tomorrow
  time_t tmp = now + 86400;
  struct tm tm2; gmtime_r(&tmp, &tm2);
  CivilTimes t2 = computeCivilTimesUTC_forDay(tm2.tm_year + 1900, tm2.tm_mon + 1, tm2.tm_mday, lat, lon);
  if (tdawn) *tdawn = t.dawn;
  if (tdusk) *tdusk = t.dusk;
  if (mdawn) *mdawn = t2.dawn;
  if (mdusk) *mdusk = t2.dusk;
}

static time_t nextCivilDawnAfter(time_t now) {
  for (int k = 0; k < 4; ++k) {
    time_t t = now + (time_t)k * 86400;
    struct tm tm_utc; gmtime_r(&t, &tm_utc);
    CivilTimes c = computeCivilTimesUTC_forDay(tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday, kLat, kLon);
    if (!c.valid) continue;
    if (c.dawn > now) return c.dawn;
  }
  return 0;
}

// ------------------- Preferences helpers -------------------
static void pushCsvEpochRolling(const char* key, time_t value) {
  if (value <= 0) return;
  if (!g_prefs_inited) return;
  char buf[64];
  String cur;
  if (g_prefs.isKey(key)) cur = g_prefs.getString(key, ""); else cur = "";
  size_t len = cur.length();
  char tmp[64]; tmp[0] = '\0';
  if (len > 0 && len < sizeof(tmp)) strncpy(tmp, cur.c_str(), sizeof(tmp));
  // parse up to 3 comma-separated values
  time_t vals[4] = {0,0,0,0}; int count = 0;
  if (tmp[0]) {
    char* save; char* tok = strtok_r(tmp, ",", &save);
    while (tok && count < 3) { vals[count++] = (time_t)strtoll(tok, nullptr, 10); tok = strtok_r(nullptr, ",", &save); }
  }
  // append new at end then keep last 3
  vals[count++] = value;
  if (count > 3) {
    // keep last 3
    vals[0] = vals[count-3];
    vals[1] = vals[count-2];
    vals[2] = vals[count-1];
    count = 3;
  }
  // write back
  int n = snprintf(buf, sizeof(buf), (count==3?"%lld,%lld,%lld": (count==2?"%lld,%lld":"%lld")),
                   (long long)vals[0], (count>=2?(long long)vals[1]:0LL), (count>=3?(long long)vals[2]:0LL));
  if (n > 0) g_prefs.putString(key, buf);
}

static void setCsvEpochList(const char* key, time_t a, time_t b, time_t c) {
  if (!g_prefs_inited) return;
  char buf[64];
  if (a && b && c) snprintf(buf, sizeof(buf), "%lld,%lld,%lld", (long long)a, (long long)b, (long long)c);
  else if (a && b) snprintf(buf, sizeof(buf), "%lld,%lld", (long long)a, (long long)b);
  else if (a) snprintf(buf, sizeof(buf), "%lld", (long long)a);
  else buf[0] = 0;
  g_prefs.putString(key, buf);
}

// ------------------- Sleep control -------------------
static void gracefulShutdown() {
  // Stop I2S task
  if (g_i2s_task) { vTaskDelete(g_i2s_task); g_i2s_task = nullptr; }
  if (g_ringbuf) { vRingbufferDelete(g_ringbuf); g_ringbuf = nullptr; }
  if (g_i2s_ok) { i2s_driver_uninstall(I2S_PORT); g_i2s_ok = false; }
  // WiFi off
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
}

static void deepSleepUntil(time_t target) {
  if (target <= 0) return;
  time_t now = time(nullptr);
  int64_t sec = (int64_t)target - (int64_t)now;
  if (sec < 5) sec = 5; // minimum
  LOGI("[SLEEP] Deep sleeping for %lld sec until %ld\n", (long long)sec, (long)target);
  esp_sleep_enable_timer_wakeup((uint64_t)sec * 1000000ULL);
  Serial.flush(); delay(50);
  gracefulShutdown();
  delay(200);
  esp_deep_sleep_start();
}

static void refreshNextSleeps(time_t today_dusk, time_t tomorrow_dusk) {
  // Compute dusk for +2 days for next_three_sleeps
  time_t d2 = tomorrow_dusk;
  if (timeIsValid() && tomorrow_dusk) {
    time_t t = tomorrow_dusk + 86400; struct tm tm2; gmtime_r(&t, &tm2);
    CivilTimes t2 = computeCivilTimesUTC_forDay(tm2.tm_year + 1900, tm2.tm_mon + 1, tm2.tm_mday, kLat, kLon);
    d2 = t2.dusk;
  }
  setCsvEpochList(PREF_KEY_NEXT_SLEEPS, today_dusk, tomorrow_dusk, d2);
}

static void ensureSchedule(time_t now) {
  uint32_t ymd = ymdFromUtc(now);
  if (g_last_compute_ymd != ymd || g_today_dawn_utc == 0 || g_today_dusk_utc == 0) {
    computeTodayTomorrow(kLat, kLon, now, &g_today_dawn_utc, &g_today_dusk_utc, &g_tomorrow_dawn_utc, &g_tomorrow_dusk_utc);
    g_last_compute_ymd = ymd;
    refreshNextSleeps(g_today_dusk_utc, g_tomorrow_dusk_utc);
    LOGI("[SCHED] Recomputed dawn/dusk. today: %ld/%ld, tomorrow: %ld/%ld\n", (long)g_today_dawn_utc, (long)g_today_dusk_utc, (long)g_tomorrow_dawn_utc, (long)g_tomorrow_dusk_utc);
#if LOG_LEVEL >= 3
    char now_iso[24], td_iso[24], ts_iso[24], nd_iso[24], ns_iso[24];
    formatIso8601UTC(now, now_iso, sizeof(now_iso));
    formatIso8601UTC(g_today_dawn_utc, td_iso, sizeof(td_iso));
    formatIso8601UTC(g_today_dusk_utc, ts_iso, sizeof(ts_iso));
    formatIso8601UTC(g_tomorrow_dawn_utc, nd_iso, sizeof(nd_iso));
    formatIso8601UTC(g_tomorrow_dusk_utc, ns_iso, sizeof(ns_iso));
    Serial.printf("[D] now=%s lat=%.6f lon=%.6f today.dawn=%s today.dusk=%s tomorrow.dawn=%s tomorrow.dusk=%s\n",
                  now_iso, (float)kLat, (float)kLon, td_iso, ts_iso, nd_iso, ns_iso);
#endif
  }
}

static void trySleepIfNight(time_t now) {
  if (!timeIsValid()) return;
  // Guard: avoid sleeping within first 20s after boot to allow NTP / stability
  if (millis() - g_boot_ms < 20000) return;
  ensureSchedule(now);
  if (now >= g_today_dusk_utc || now < g_today_dawn_utc) {
    time_t nd = nextCivilDawnAfter(now);
    if (nd > now) deepSleepUntil(nd);
  }
}

static bool waitForNtp(uint32_t timeout_ms) {
  uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    if (timeIsValid()) return true;
    delay(200);
  }
  return timeIsValid();
}

static void maybeSyncNtp() {
  time_t now = time(nullptr);
  if (!timeIsValid() || (now - g_last_ntp_sync_utc) > 86400) {
    g_last_ntp_check_utc = now;
    LOGI("[NTP] Sync starting...\n");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    if (waitForNtp(15000)) {
      g_last_ntp_sync_utc = time(nullptr);
      LOGI("[NTP] Sync ok: %ld\n", (long)g_last_ntp_sync_utc);
    } else {
      LOGW("[NTP] Sync failed, will retry later\n");
    }
  }
}

static void handleUptime() {
  // Legacy uptime fields
  unsigned long total = millis() / 1000UL;
  unsigned long d = total / 86400UL; total %= 86400UL;
  unsigned long h = total / 3600UL;  total %= 3600UL;
  unsigned long m = total / 60UL;    unsigned long s = total % 60UL;

  char human[64];
  size_t pos = 0;
  if (d > 0) pos += snprintf(human + pos, sizeof(human) - pos, "%lud ", d);
  if (d > 0 || h > 0) pos += snprintf(human + pos, sizeof(human) - pos, "%luh ", h);
  if (d > 0 || h > 0 || m > 0) pos += snprintf(human + pos, sizeof(human) - pos, "%lum ", m);
  snprintf(human + pos, sizeof(human) - pos, "%lus", s);

  // Schedule details
  time_t now = time(nullptr);
  char now_iso[24]; formatIso8601UTC(now, now_iso, sizeof(now_iso));
  ensureSchedule(now);

  char tdawn_iso[24]; char tdusk_iso[24];
  char mdawn_iso[24]; char mdusk_iso[24];
  formatIso8601UTC(g_today_dawn_utc, tdawn_iso, sizeof(tdawn_iso));
  formatIso8601UTC(g_today_dusk_utc, tdusk_iso, sizeof(tdusk_iso));
  formatIso8601UTC(g_tomorrow_dawn_utc, mdawn_iso, sizeof(mdawn_iso));
  formatIso8601UTC(g_tomorrow_dusk_utc, mdusk_iso, sizeof(mdusk_iso));

  const char* mode = "unknown";
  if (timeIsValid()) {
    if (now >= g_today_dawn_utc && now < g_today_dusk_utc) mode = "day"; else mode = "night";
  }

  char next_type[28] = "unknown"; time_t next_at = 0; uint32_t seconds_until = 0;
  if (timeIsValid()) {
    if (strcmp(mode, "day") == 0) { strcpy(next_type, "sleep_at_civil_dusk"); next_at = g_today_dusk_utc; }
    else {
      strcpy(next_type, "wake_at_civil_dawn");
      next_at = (now < g_today_dawn_utc) ? g_today_dawn_utc : g_tomorrow_dawn_utc;
    }
    if (next_at > now) seconds_until = (uint32_t)(next_at - now);
  }
  char next_at_iso[24]; formatIso8601UTC(next_at, next_at_iso, sizeof(next_at_iso));

  // Preferences arrays
  if (!g_prefs_inited) { g_prefs.begin(PREF_NS, false); g_prefs_inited = true; }
  String wakes_csv = g_prefs.isKey(PREF_KEY_LAST_WAKES) ? g_prefs.getString(PREF_KEY_LAST_WAKES, "") : String("");
  String sleeps_csv = g_prefs.isKey(PREF_KEY_NEXT_SLEEPS) ? g_prefs.getString(PREF_KEY_NEXT_SLEEPS, "") : String("");

  // Compose JSON
  char buf[1024];
  int n = 0;
  n += snprintf(buf + n, sizeof(buf) - n,
                "{\"uptime\": %lu, \"uptime_human\": \"%s\", \"days\": %lu, \"hours\": %lu, \"minutes\": %lu, \"seconds\": %lu, ",
                (millis() / 1000UL), human, d, h, m, s);

  n += snprintf(buf + n, sizeof(buf) - n,
                "\"now_utc\": \"%s\", ", now_iso);
  char last_check_iso[24]; formatIso8601UTC(g_last_ntp_check_utc, last_check_iso, sizeof(last_check_iso));
  n += snprintf(buf + n, sizeof(buf) - n,
                "\"last_ntp_check_utc\": \"%s\", ", last_check_iso);
  n += snprintf(buf + n, sizeof(buf) - n,
                "\"today\": {\"civil_dawn_utc\": \"%s\", \"civil_dusk_utc\": \"%s\"}, ", tdawn_iso, tdusk_iso);

  n += snprintf(buf + n, sizeof(buf) - n,
                "\"boot_count\": %lu, \"schedule_basis\": \"civil_twilight_-6deg\", \"location\": {\"lat\": %.5f, \"lon\": %.5f}, ",
                (unsigned long)g_boot_count, (float)kLat, (float)kLon);
  n += snprintf(buf + n, sizeof(buf) - n,
                "\"tomorrow\": {\"civil_dawn_utc\": \"%s\", \"civil_dusk_utc\": \"%s\"}, \"mode\": \"%s\", ", mdawn_iso, mdusk_iso, mode);
  n += snprintf(buf + n, sizeof(buf) - n,
                "\"next_event\": {\"type\": \"%s\", \"at_utc\": \"%s\", \"seconds_until\": %u}, ", next_type, next_at_iso, seconds_until);

  // Arrays: convert CSV to ISO array for output
  auto appendIsoArray = [&](const char* key, const String& csv) {
    n += snprintf(buf + n, sizeof(buf) - n, "\"%s\":[", key);
    int count = 0;
    if (csv.length() > 0) {
      char tmp[64]; strncpy(tmp, csv.c_str(), sizeof(tmp)); tmp[sizeof(tmp)-1] = 0;
      char* save; char* tok = strtok_r(tmp, ",", &save);
      while (tok && count < 3) {
        time_t t = (time_t)strtoll(tok, nullptr, 10);
        char iso[24]; formatIso8601UTC(t, iso, sizeof(iso));
        n += snprintf(buf + n, sizeof(buf) - n, "%s\"%s\"", (count?",":""), iso);
        ++count; tok = strtok_r(nullptr, ",", &save);
      }
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]");
  };

  appendIsoArray("last_three_wakes_utc", wakes_csv);
  n += snprintf(buf + n, sizeof(buf) - n, ", ");
  appendIsoArray("next_three_sleeps_utc", sleeps_csv);
  n += snprintf(buf + n, sizeof(buf) - n, "}");

  if (n <= 0) {
    server.send(500, "application/json", "{\"error\":\"formatting\"}");
    return;
  }
  server.send(200, "application/json", buf);
}

static void handleUptime_LegacyOnly() {
  unsigned long total = millis() / 1000UL;
  unsigned long d = total / 86400UL; total %= 86400UL;
  unsigned long h = total / 3600UL;  total %= 3600UL;
  unsigned long m = total / 60UL;    unsigned long s = total % 60UL;

  char human[64];
  size_t pos = 0;
  if (d > 0) pos += snprintf(human + pos, sizeof(human) - pos, "%lud ", d);
  if (d > 0 || h > 0) pos += snprintf(human + pos, sizeof(human) - pos, "%luh ", h);
  if (d > 0 || h > 0 || m > 0) pos += snprintf(human + pos, sizeof(human) - pos, "%lum ", m);
  snprintf(human + pos, sizeof(human) - pos, "%lus", s);

  char buf[192];
  int n = snprintf(buf, sizeof(buf),
                   "{\"uptime\": %lu, \"uptime_human\": \"%s\", \"days\": %lu, \"hours\": %lu, \"minutes\": %lu, \"seconds\": %lu}",
                   (millis() / 1000UL), human, d, h, m, s);
  
  if (n < 0) {
    server.send(500, "application/json", "{\"error\":\"formatting\"}");
    return;
  }
  server.send(200, "application/json", buf);
}

// New endpoint that serves non-uptime status and scheduling data
static void handleStatus() {
  // Ensure schedule/time context is up to date
  time_t now = time(nullptr);
  char now_iso[24];
  formatIso8601UTC(now, now_iso, sizeof(now_iso));
  ensureSchedule(now);

  char tdawn_iso[24]; char tdusk_iso[24];
  char mdawn_iso[24]; char mdusk_iso[24];
  formatIso8601UTC(g_today_dawn_utc, tdawn_iso, sizeof(tdawn_iso));
  formatIso8601UTC(g_today_dusk_utc, tdusk_iso, sizeof(tdusk_iso));
  formatIso8601UTC(g_tomorrow_dawn_utc, mdawn_iso, sizeof(mdawn_iso));
  formatIso8601UTC(g_tomorrow_dusk_utc, mdusk_iso, sizeof(mdusk_iso));

  const char* mode = "unknown";
  if (timeIsValid()) {
    if (now >= g_today_dawn_utc && now < g_today_dusk_utc) mode = "day"; else mode = "night";
  }

  char next_type[28] = "unknown"; time_t next_at = 0; uint32_t seconds_until = 0;
  if (timeIsValid()) {
    if (strcmp(mode, "day") == 0) { strcpy(next_type, "sleep_at_civil_dusk"); next_at = g_today_dusk_utc; }
    else {
      strcpy(next_type, "wake_at_civil_dawn");
      next_at = (now < g_today_dawn_utc) ? g_today_dawn_utc : g_tomorrow_dawn_utc;
    }
    if (next_at > now) seconds_until = (uint32_t)(next_at - now);
  }
  char next_at_iso[24]; formatIso8601UTC(next_at, next_at_iso, sizeof(next_at_iso));

  // Preferences arrays
  if (!g_prefs_inited) { g_prefs.begin(PREF_NS, false); g_prefs_inited = true; }
  String wakes_csv = g_prefs.isKey(PREF_KEY_LAST_WAKES) ? g_prefs.getString(PREF_KEY_LAST_WAKES, "") : String("");
  String sleeps_csv = g_prefs.isKey(PREF_KEY_NEXT_SLEEPS) ? g_prefs.getString(PREF_KEY_NEXT_SLEEPS, "") : String("");

  // Compose JSON (no uptime fields)
  char buf[1024];
  int n = 0;
  n += snprintf(buf + n, sizeof(buf) - n, "{\"now_utc\": \"%s\", ", now_iso);
  char last_check_iso[24]; formatIso8601UTC(g_last_ntp_check_utc, last_check_iso, sizeof(last_check_iso));
  n += snprintf(buf + n, sizeof(buf) - n, "\"last_ntp_check_utc\": \"%s\", ", last_check_iso);

  n += snprintf(buf + n, sizeof(buf) - n,
                "\"today\": {\"civil_dawn_utc\": \"%s\", \"civil_dusk_utc\": \"%s\"}, ", tdawn_iso, tdusk_iso);

  n += snprintf(buf + n, sizeof(buf) - n,
                "\"boot_count\": %lu, \"schedule_basis\": \"civil_twilight_-6deg\", \"location\": {\"lat\": %.5f, \"lon\": %.5f}, ",
                (unsigned long)g_boot_count, (float)kLat, (float)kLon);

  n += snprintf(buf + n, sizeof(buf) - n,
                "\"tomorrow\": {\"civil_dawn_utc\": \"%s\", \"civil_dusk_utc\": \"%s\"}, \"mode\": \"%s\", ", mdawn_iso, mdusk_iso, mode);

  n += snprintf(buf + n, sizeof(buf) - n,
                "\"next_event\": {\"type\": \"%s\", \"at_utc\": \"%s\", \"seconds_until\": %u}, ", next_type, next_at_iso, seconds_until);

  auto appendIsoArray = [&](const char* key, const String& csv) {
    n += snprintf(buf + n, sizeof(buf) - n, "\"%s\":[", key);
    int count = 0;
    if (csv.length() > 0) {
      char tmp[64]; strncpy(tmp, csv.c_str(), sizeof(tmp)); tmp[sizeof(tmp)-1] = 0;
      char* save; char* tok = strtok_r(tmp, ",", &save);
      while (tok && count < 3) {
        time_t t = (time_t)strtoll(tok, nullptr, 10);
        char iso[24]; formatIso8601UTC(t, iso, sizeof(iso));
        n += snprintf(buf + n, sizeof(buf) - n, "%s\"%s\"", (count?",":""), iso);
        ++count; tok = strtok_r(nullptr, ",", &save);
      }
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]");
  };

  appendIsoArray("last_three_wakes_utc", wakes_csv);
  n += snprintf(buf + n, sizeof(buf) - n, ", ");
  appendIsoArray("next_three_sleeps_utc", sleeps_csv);
  n += snprintf(buf + n, sizeof(buf) - n, "}");

  if (n <= 0) {
    server.send(500, "application/json", "{\"error\":\"formatting\"}");
    return;
  }
  server.send(200, "application/json", buf);
}

static void i2sProducerTask(void* arg) {
const size_t frames_per_chunk = CHUNK_FRAMES;
  int32_t* in32 = (int32_t*)malloc(frames_per_chunk * BYTES_PER_SAMPLE_IN);
  int16_t* out16 = (int16_t*)malloc(frames_per_chunk * sizeof(int16_t));
  if (!in32 || !out16) {
    LOGE("[RB] buffer alloc failed; stopping producer\n");
    if (in32) free(in32);
    if (out16) free(out16);
    vTaskDelete(nullptr);
    return;
  }

  for (;;) {
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_PORT, (void*)in32, frames_per_chunk * BYTES_PER_SAMPLE_IN, &bytes_read, portMAX_DELAY);
    if (err != ESP_OK || bytes_read == 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    size_t frames = bytes_read / BYTES_PER_SAMPLE_IN;
    for (size_t i = 0; i < frames; ++i) {
      int32_t s = in32[i];
      s >>= CONVERT_SHIFT;
      if (s > 32767) s = 32767;
      if (s < -32768) s = -32768;
      out16[i] = (int16_t)s;
    }

    // Push into ring buffer; drop if full to avoid blocking I2S
    size_t bytes = frames * sizeof(int16_t);
    BaseType_t ok = xRingbufferSend(g_ringbuf, out16, bytes, 0);
    if (ok != pdTRUE) {
      // overrun: drop
    }
  }
}

static bool initI2SMic() {
  i2s_config_t i2s_config = {};
  i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  i2s_config.sample_rate = SAMPLE_RATE_HZ;
  i2s_config.bits_per_sample = (i2s_bits_per_sample_t)BITS_PER_SAMPLE;
  i2s_config.channel_format = I2S_CHAN_FMT; // mono via only-left (or only-right)
  i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
  i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2s_config.dma_buf_count = DMA_BUF_COUNT;
  i2s_config.dma_buf_len = DMA_BUF_LEN;
  i2s_config.use_apll = false;
  i2s_config.tx_desc_auto_clear = false;
  i2s_config.fixed_mclk = 0;

  if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK) {
    LOGE("[I2S] driver install failed\n");
    return false;
  }

  i2s_pin_config_t pin_config = {};
  pin_config.bck_io_num = PIN_I2S_SCK;
  pin_config.ws_io_num = PIN_I2S_WS;
  pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  pin_config.data_in_num = PIN_I2S_SD;

  if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
    LOGE("[I2S] set pin failed\n");
    return false;
  }

  // Ensure mono format alignment
  i2s_zero_dma_buffer(I2S_PORT);

  // Some mics need left-justified channel selection; set if needed:
  i2s_set_clk(I2S_PORT, SAMPLE_RATE_HZ, (i2s_bits_per_sample_t)BITS_PER_SAMPLE, I2S_CHANNEL_MONO);

  LOGI("[I2S] init ok: %d Hz, %d-bit, mono, WS=%d, SCK=%d, SD=%d\n",
       SAMPLE_RATE_HZ, BITS_PER_SAMPLE, PIN_I2S_WS, PIN_I2S_SCK, PIN_I2S_SD);
  return true;
}

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
  // Ensure clean Wi‑Fi init after deep sleep / resets
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  delay(200);
  // Extra safety: stop/deinit underlying driver if previously initialized
  esp_wifi_stop();
  esp_wifi_deinit();
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (String(WIFI_SSID) == "YOUR_SSID" || String(WIFI_PASS) == "YOUR_PASSWORD") {
    LOGW("WIFI_SSID/WIFI_PASS are placeholders.\n");
    LOGW("Set them in platformio.ini build_flags, then rebuild/flash.\n");
  }

  LOGI("Connecting to Wi‑Fi SSID: '%s'\n", ssid);

  // Skip pre-scan to reduce init complexity and speed up first connection

  // Set TX power from build flag, mapped to closest supported step
  wifi_power_t txp = mapTxPowerDbm((int)WIFI_TX_POWER_DBM);
  WiFi.setTxPower(txp);
  LOGI("[WiFi] TX power target=%d dBm (mapped enum=%d)\n", (int)WIFI_TX_POWER_DBM, (int)txp);
  WiFi.begin(ssid, pass);

  // Block until connected
  uint32_t dot = 0;
  uint32_t lastDiag = millis();
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

void setup() {
  Serial.begin(115200);
  delay(200);
  LOG_NL();
LOGI("Booting ESP32 Audio Streamer (Step 1: Wi‑Fi + HTTP)\n");
  g_boot_ms = millis();
  g_boot_count++;
  if (!g_prefs_inited) { g_prefs.begin(PREF_NS, false); g_prefs_inited = true; }
  setenv("TZ", "UTC0", 1); tzset();

  // Optional: brownout workaround (can be disabled via build flag)
#if ENABLE_BROWNOUT_DISABLE
  LOGI("[PMIC] Brownout detector disabled (workaround enabled)\n");
  disableBrownout();
#else
  LOGI("[PMIC] Brownout workaround disabled (leaving detector enabled)\n");
#endif

  // Bring up Wi‑Fi first to obtain valid time via NTP
  connectWiFiBlocking();

  // Initial NTP sync and schedule (post Wi‑Fi)
  maybeSyncNtp();
  time_t now = time(nullptr);
  if (timeIsValid()) {
    ensureSchedule(now);
    if (now >= g_today_dawn_utc && now < g_today_dusk_utc) {
      // Daytime: record wake and continue
      pushCsvEpochRolling(PREF_KEY_LAST_WAKES, now);
      refreshNextSleeps(g_today_dusk_utc, g_tomorrow_dusk_utc);
      g_last_mode = 1;
    } else {
      // Nighttime after valid NTP: choose next future civil dawn and sleep then
      time_t nd = nextCivilDawnAfter(now);
      if (nd > now) {
        LOGI("[MODE] Night after NTP; scheduling deep sleep until next dawn (%ld)\n", (long)nd);
        deepSleepUntil(nd);
      } else {
        LOGW("[MODE] Night after NTP but could not find future dawn; staying awake\n");
      }
    }
  } else {
    LOGW("[NTP] Time invalid at boot; will retry in loop (no sleep decisions yet)\n");
  }

  // I2S mic
  g_i2s_ok = initI2SMic();
  if (!g_i2s_ok) {
    LOGE("[I2S] init failed; /stream will 503\n");
  }

  // Ring buffer and producer task
  if (g_i2s_ok) {
    g_ringbuf = xRingbufferCreate(RINGBUF_CAPACITY_BYTES, RINGBUF_TYPE_BYTEBUF);
    if (g_ringbuf) {
      g_rb_ok = true;
      xTaskCreatePinnedToCore(i2sProducerTask, "i2s_producer", 6144, nullptr, 5, &g_i2s_task, 0);
      LOGI("[RB] created %u bytes, producer task started\n", (unsigned)RINGBUF_CAPACITY_BYTES);
    } else {
      LOGE("[RB] create failed\n");
    }
  }

  // Configure HPF coefficient from cut-off and sample rate
  if (g_i2s_ok) {
    const float fs = (float)SAMPLE_RATE_HZ;
    const float fc = (float)HPF_CUTOFF_HZ;
    g_hpf_R = expf(-2.0f * PI_F * fc / fs);
    // Convert R to Q15 once for fast integer processing
    float aq = g_hpf_R * 32768.0f;
    if (aq < 0.0f) aq = 0.0f;
    if (aq > 32767.0f) aq = 32767.0f;
    g_hpf_a_q15 = (int32_t)lrintf(aq);
    LOGI("[HPF] %s, fc=%d Hz, R=%.6f (a_q15=%ld)\n", g_hpf_enabled ? "ENABLED" : "disabled", (int)HPF_CUTOFF_HZ, g_hpf_R, (long)g_hpf_a_q15);
  }

// HTTP routes
  server.on("/", HTTP_GET, handleRoot);
  // Expose uptime-only fields on /uptime, and move scheduling/status to /status
  server.on("/uptime", HTTP_GET, handleUptime_LegacyOnly);
  server.on("/status", HTTP_GET, handleStatus);
  if (g_i2s_ok && g_rb_ok) {
    server.on("/stream", HTTP_GET, handleStream);
  } else {
    server.on("/stream", HTTP_GET, [](){ server.send(503, "text/plain", "I2S not initialized"); });
  }
  server.begin();
  LOGI("HTTP server started on :%d\n", (int)SERVER_PORT);
}

void loop() {
  server.handleClient();

  // Periodic NTP sync while awake
  uint32_t now_ms = millis();
  if (now_ms - g_next_ntp_retry_ms > 60000) { // every 60s
    g_next_ntp_retry_ms = now_ms;
    maybeSyncNtp();
  }

  // Recompute schedule daily and sleep at dusk
  time_t nowt = time(nullptr);
  if (timeIsValid()) {
    ensureSchedule(nowt);
    trySleepIfNight(nowt);
  }

  delay(2);
}
