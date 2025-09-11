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

#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YOUR_PASSWORD"
#endif

#ifndef SERVER_PORT
#define SERVER_PORT 80
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

static const char INDEX_HTML[] PROGMEM = R"HTML(
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
)HTML";

static void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

static void handleStream() {
  if (!g_rb_ok) { server.send(503, "text/plain", "Ring buffer not ready"); return; }

  // Start a raw PCM stream: HTTP/1.1 without Content-Length; keep connection open
  WiFiClient client = server.client();
  client.setNoDelay(true);
  client.print("HTTP/1.1 200 OK\r\n");
  client.print("Content-Type: audio/L16; rate=");
  client.print(SAMPLE_RATE_HZ);
  client.print("; channels=1\r\n");
  client.print("Cache-Control: no-store\r\nConnection: close\r\n\r\n");

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
    Serial.println("[RB] buffer alloc failed; stopping producer");
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
    Serial.println("[I2S] driver install failed");
    return false;
  }

  i2s_pin_config_t pin_config = {};
  pin_config.bck_io_num = PIN_I2S_SCK;
  pin_config.ws_io_num = PIN_I2S_WS;
  pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  pin_config.data_in_num = PIN_I2S_SD;

  if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
    Serial.println("[I2S] set pin failed");
    return false;
  }

  // Ensure mono format alignment
  i2s_zero_dma_buffer(I2S_PORT);

  // Some mics need left-justified channel selection; set if needed:
  i2s_set_clk(I2S_PORT, SAMPLE_RATE_HZ, (i2s_bits_per_sample_t)BITS_PER_SAMPLE, I2S_CHANNEL_MONO);

  Serial.printf("[I2S] init ok: %d Hz, %d-bit, mono, WS=%d, SCK=%d, SD=%d\n",
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
    Serial.println();
    Serial.println("[WARN] WIFI_SSID/WIFI_PASS are placeholders.");
    Serial.println("       Set them in platformio.ini build_flags, then rebuild/flash.");
  }

  Serial.printf("Connecting to Wi‑Fi SSID: '%s'\n", ssid);

  // Optional: quick scan to verify SSID is visible
  Serial.println("Scanning for networks...");
  int n = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);
  if (n <= 0) {
    Serial.println("[WiFi] No networks found");
  } else {
    bool seen = false;
    for (int i = 0; i < n; ++i) {
      String s = WiFi.SSID(i);
      int32_t rssi = WiFi.RSSI(i);
      if (s == ssid) {
        seen = true;
        Serial.printf("[WiFi] Found target SSID '%s' RSSI=%d dBm\n", s.c_str(), (int)rssi);
      }
    }
    if (!seen) Serial.println("[WiFi] Target SSID not seen in scan (may still connect)");
  }

  // Use a stronger TX power (requires decent USB power) for robust association
  WiFi.setTxPower(WIFI_POWER_15dBm);
  WiFi.begin(ssid, pass);

  // Block until connected
  uint32_t dot = 0;
  uint32_t lastDiag = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print('.');
    if ((++dot % 40) == 0) Serial.println();
    if (millis() - lastDiag > 10000) {
      Serial.println("\n[WiFi] Still connecting... printing diagnostics");
      WiFi.printDiag(Serial);
      lastDiag = millis();
    }
  }
  Serial.println();
  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());
}

static void disableBrownout() {
#ifdef RTC_CNTL_BROWN_OUT_REG
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Booting ESP32 Audio Streamer (Step 1: Wi‑Fi + HTTP)");

  // Mitigate repeated resets due to brownout on marginal USB power
  disableBrownout();

  connectWiFiBlocking();

  // I2S mic
  g_i2s_ok = initI2SMic();
  if (!g_i2s_ok) {
    Serial.println("[I2S] init failed; /stream will 503");
  }

  // Ring buffer and producer task
  if (g_i2s_ok) {
    g_ringbuf = xRingbufferCreate(RINGBUF_CAPACITY_BYTES, RINGBUF_TYPE_BYTEBUF);
    if (g_ringbuf) {
      g_rb_ok = true;
      xTaskCreatePinnedToCore(i2sProducerTask, "i2s_producer", 6144, nullptr, 5, &g_i2s_task, 0);
      Serial.printf("[RB] created %u bytes, producer task started\n", (unsigned)RINGBUF_CAPACITY_BYTES);
    } else {
      Serial.println("[RB] create failed");
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
    Serial.printf("[HPF] %s, fc=%d Hz, R=%.6f (a_q15=%ld)\n", g_hpf_enabled ? "ENABLED" : "disabled", (int)HPF_CUTOFF_HZ, g_hpf_R, (long)g_hpf_a_q15);
  }

  // HTTP routes
  server.on("/", HTTP_GET, handleRoot);
  if (g_i2s_ok && g_rb_ok) {
    server.on("/stream", HTTP_GET, handleStream);
  } else {
    server.on("/stream", HTTP_GET, [](){ server.send(503, "text/plain", "I2S not initialized"); });
  }
  server.begin();
  Serial.println("HTTP server started on :80");
}

void loop() {
  server.handleClient();
  delay(2);
}