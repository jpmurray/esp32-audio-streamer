#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/i2s.h"
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

#ifndef SERVER_PORT
#define SERVER_PORT 80
#endif
// Stream format toggle (set via extra configs/local_env.ini)
#ifndef STREAM_WAV_ENABLE
#define STREAM_WAV_ENABLE 0   // 0 = raw audio/L16, 1 = audio/x-wav with header
#endif

static WebServer server(SERVER_PORT);
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

static void connectWiFiBlocking() {
  const char* ssid = WIFI_SSID;
  const char* pass = WIFI_PASS;
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.disconnect(true, true);
  delay(200);

  if (String(WIFI_SSID) == "YOUR_SSID" || String(WIFI_PASS) == "YOUR_PASSWORD") {
    LOGW("WIFI_SSID/WIFI_PASS are placeholders.\n");
    LOGW("Set them in platformio.ini build_flags, then rebuild/flash.\n");
  }

  LOGI("Connecting to Wi‑Fi SSID: '%s'\n", ssid);

  // Optional: quick scan to verify SSID is visible
  LOGI("Scanning for networks...\n");
  int n = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);
  if (n <= 0) {
    LOGW("[WiFi] No networks found\n");
  } else {
    bool seen = false;
    for (int i = 0; i < n; ++i) {
      String s = WiFi.SSID(i);
      int32_t rssi = WiFi.RSSI(i);
      if (s == ssid) {
        seen = true;
        LOGI("[WiFi] Found target SSID '%s' RSSI=%d dBm\n", s.c_str(), (int)rssi);
      }
    }
    if (!seen) LOGW("[WiFi] Target SSID not seen in scan (may still connect)\n");
  }

  // Use a stronger TX power (requires decent USB power) for robust association
  WiFi.setTxPower(WIFI_POWER_15dBm);
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

  // Mitigate repeated resets due to brownout on marginal USB power
  disableBrownout();

  connectWiFiBlocking();

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
  delay(2);
}
