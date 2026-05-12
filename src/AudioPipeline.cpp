// AudioPipeline.cpp — I2S microphone, ring buffer, and HPF DSP pipeline.

#include "AudioPipeline.h"
#include "AppState.h"
#include "RuntimeSettings.h"

#include <Arduino.h>
#include <math.h>
#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "LogBuffer.h"  // centralized LOGE/LOGW/LOGI/LOGD(module, fmt, ...)

// ------------------------------------------------------------
// Compile-time configuration (mirrors main.cpp defaults)
// ------------------------------------------------------------
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
// Default ring buffer: 256 KB — large enough for ~1.3 s at 48 kHz / 16-bit mono
// or ~2.7 s at 24 kHz / 16-bit mono.  The expanded buffer absorbs Wi-Fi hiccups
// and reduces drop counts that cause BirdNET-Go to restart the stream.
#define RB_CAPACITY_BYTES (256 * 1024)
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
#ifndef DMA_BUF_COUNT_CFG
#define DMA_BUF_COUNT_CFG 4
#endif

// ------------------------------------------------------------
// Internal constants
// ------------------------------------------------------------
static const i2s_port_t I2S_PORT        = (i2s_port_t)I2S_PORT_NUM;
static const int BITS_PER_SAMPLE        = 32;
static const int BYTES_PER_SAMPLE_IN    = BITS_PER_SAMPLE / 8;
static const int DMA_BUF_COUNT          = DMA_BUF_COUNT_CFG;
static const int DMA_BUF_LEN            = 1024;
static const size_t RINGBUF_CAPACITY    = RB_CAPACITY_BYTES;

#if USE_RIGHT_CHANNEL
static const i2s_channel_fmt_t I2S_CHAN_FMT = I2S_CHANNEL_FMT_ONLY_RIGHT;
#else
static const i2s_channel_fmt_t I2S_CHAN_FMT = I2S_CHANNEL_FMT_ONLY_LEFT;
#endif

// ------------------------------------------------------------
// HPF state (module-private)
// ------------------------------------------------------------
static bool    s_hpf_enabled    = (HPF_ENABLE != 0);
static int16_t s_hpf_prev_x_i16 = 0;
static int32_t s_hpf_prev_y_i32 = 0;
static int32_t s_hpf_a_q15      = 0;

// ------------------------------------------------------------
// Audio metrics (module-private, updated by producer task)
// ------------------------------------------------------------
static volatile int16_t  s_peak_level        = 0;
static volatile int16_t  s_peak_hold         = 0;
static volatile uint32_t s_clip_count        = 0;
static volatile bool     s_clipped_last_block = false;
static volatile uint32_t s_i2s_error_count   = 0;
static volatile uint32_t s_rb_drop_count     = 0;

// Rate-limit RB drop warnings to avoid flooding the log (one message per 30 s).
static volatile uint32_t s_last_rb_drop_log_ms = 0;
#ifndef RB_DROP_LOG_INTERVAL_MS
#define RB_DROP_LOG_INTERVAL_MS 30000
#endif

// ------------------------------------------------------------
// I2S producer task
// ------------------------------------------------------------
// Active convert_shift and sample rate used by the running producer task.
// Set once during audioPipeline_init(); read-only after that.
static volatile int s_active_convert_shift = CONVERT_SHIFT;
static volatile int s_active_sample_rate_hz = SAMPLE_RATE_HZ;
static volatile size_t s_active_ringbuf_capacity = 0;

static void i2sProducerTask(void* /*arg*/) {
    const size_t frames_per_chunk = CHUNK_FRAMES;
    const int convert_shift = s_active_convert_shift;
    int32_t* in32  = (int32_t*)malloc(frames_per_chunk * BYTES_PER_SAMPLE_IN);
    int16_t* out16 = (int16_t*)malloc(frames_per_chunk * sizeof(int16_t));
    if (!in32 || !out16) {
        LOGE("AP", "[RB] buffer alloc failed; stopping producer\n");
        if (in32)  free(in32);
        if (out16) free(out16);
        vTaskDelete(nullptr);
        return;
    }

    for (;;) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_read(I2S_PORT, (void*)in32,
                                 frames_per_chunk * BYTES_PER_SAMPLE_IN,
                                 &bytes_read, portMAX_DELAY);
        if (err != ESP_OK || bytes_read == 0) {
            s_i2s_error_count++;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        size_t frames = bytes_read / BYTES_PER_SAMPLE_IN;
        for (size_t i = 0; i < frames; ++i) {
            int32_t s = in32[i];
            s >>= convert_shift;
            if (s >  32767) s =  32767;
            if (s < -32768) s = -32768;
            out16[i] = (int16_t)s;
        }

        // --- Metrics: scan converted PCM-16 samples (post-convert, pre-HPF).
        // HPF is applied later in StreamServer before transmission; metrics
        // therefore reflect the raw mic signal after bit-shift conversion.
        bool clipped = false;
        for (size_t i = 0; i < frames; ++i) {
            int16_t abs_s = out16[i] < 0 ? (out16[i] == -32768 ? 32767 : -out16[i]) : out16[i];
            if (abs_s > s_peak_level)  s_peak_level = abs_s;
            if (abs_s > s_peak_hold)   s_peak_hold  = abs_s;
            if (abs_s == 32767) {
                s_clip_count++;
                clipped = true;
            }
        }
        s_clipped_last_block = clipped;

        size_t bytes = frames * sizeof(int16_t);
        BaseType_t sent = xRingbufferSend(g_ringbuf, out16, bytes, 0);
        if (sent != pdTRUE) {
            s_rb_drop_count++;
            // Log first drop and then at most once per RB_DROP_LOG_INTERVAL_MS
            // to avoid flooding the log during long stalls.
            uint32_t now_drop = (uint32_t)millis();
            if (s_last_rb_drop_log_ms == 0 ||
                (now_drop - s_last_rb_drop_log_ms) >= (uint32_t)RB_DROP_LOG_INTERVAL_MS) {
                s_last_rb_drop_log_ms = now_drop;
                LOGW("AP", "[RB] drop #%lu (ring buffer full)\n",
                     (unsigned long)s_rb_drop_count);
            }
        }
    }
}

// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------
bool audioPipeline_init() {
    // --- Resolve sample rate from runtime audio profile ---
    // If the runtime profile has been configured, honour it; otherwise fall back
    // to the compile-time SAMPLE_RATE_HZ constant.
    const int sample_rate_hz = audioProfile_sampleRateHz(g_runtime_settings.audio_profile);
    s_active_sample_rate_hz  = sample_rate_hz;
    s_active_convert_shift   = (int)g_runtime_settings.convert_shift;

    // --- I2S driver ---
    i2s_config_t cfg = {};
    cfg.mode                = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    cfg.sample_rate         = (uint32_t)sample_rate_hz;
    cfg.bits_per_sample     = (i2s_bits_per_sample_t)BITS_PER_SAMPLE;
    cfg.channel_format      = I2S_CHAN_FMT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags    = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count       = DMA_BUF_COUNT;
    cfg.dma_buf_len         = DMA_BUF_LEN;
    cfg.use_apll            = false;
    cfg.tx_desc_auto_clear  = false;
    cfg.fixed_mclk          = 0;

    if (i2s_driver_install(I2S_PORT, &cfg, 0, NULL) != ESP_OK) {
        LOGE("AP", "[I2S] driver install failed\n");
        return false;
    }

    i2s_pin_config_t pins = {};
    pins.bck_io_num    = PIN_I2S_SCK;
    pins.ws_io_num     = PIN_I2S_WS;
    pins.data_out_num  = I2S_PIN_NO_CHANGE;
    pins.data_in_num   = PIN_I2S_SD;

    if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK) {
        LOGE("AP", "[I2S] set pin failed\n");
        return false;
    }

    i2s_zero_dma_buffer(I2S_PORT);
    i2s_set_clk(I2S_PORT, (uint32_t)sample_rate_hz,
                (i2s_bits_per_sample_t)BITS_PER_SAMPLE, I2S_CHANNEL_MONO);

    LOGI("AP", "[I2S] init ok: %d Hz (%s), %d-bit, mono, WS=%d, SCK=%d, SD=%d\n",
         sample_rate_hz,
         audioProfile_name(g_runtime_settings.audio_profile),
         BITS_PER_SAMPLE, PIN_I2S_WS, PIN_I2S_SCK, PIN_I2S_SD);
    g_i2s_ok = true;

    // --- Ring buffer ---
    // Prefer the configured capacity, but fall back to smaller buffers on
    // RAM-constrained ESP32 variants instead of failing the whole pipeline.
    static const size_t kFallbackCaps[] = {
        RINGBUF_CAPACITY,
        192 * 1024,
        128 * 1024,
        96 * 1024,
        64 * 1024,
    };
    s_active_ringbuf_capacity = 0;
    for (size_t i = 0; i < (sizeof(kFallbackCaps) / sizeof(kFallbackCaps[0])); ++i) {
        size_t cap = kFallbackCaps[i];
        if (s_active_ringbuf_capacity == cap) continue;  // skip duplicates
        g_ringbuf = xRingbufferCreate(cap, RINGBUF_TYPE_BYTEBUF);
        if (g_ringbuf) {
            s_active_ringbuf_capacity = cap;
            break;
        }
    }
    if (!g_ringbuf) {
        LOGE("AP", "[RB] create failed at %u/%u/%u/%u/%u bytes\n",
             (unsigned)kFallbackCaps[0], (unsigned)kFallbackCaps[1],
             (unsigned)kFallbackCaps[2], (unsigned)kFallbackCaps[3],
             (unsigned)kFallbackCaps[4]);
        return false;
    }
    g_rb_ok = true;

    xTaskCreatePinnedToCore(i2sProducerTask, "i2s_producer",
                            6144, nullptr, 5, &g_i2s_task, 0);
    LOGI("AP", "[RB] created %u bytes, producer task started\n",
         (unsigned)s_active_ringbuf_capacity);

    // --- HPF coefficients ---
    const float fs = (float)sample_rate_hz;
    const float fc = (float)g_runtime_settings.hpf_cutoff_hz;
    float R = expf(-2.0f * PI_F * fc / fs);
    float aq = R * 32768.0f;
    if (aq < 0.0f)     aq = 0.0f;
    if (aq > 32767.0f) aq = 32767.0f;
    s_hpf_a_q15 = (int32_t)lrintf(aq);
    LOGI("AP", "[HPF] %s, fc=%d Hz, R=%.6f (a_q15=%ld)\n",
         s_hpf_enabled ? "ENABLED" : "disabled",
         (int)g_runtime_settings.hpf_cutoff_hz, R, (long)s_hpf_a_q15);

    LOGI("AP", "[AP] profile=%s sample_rate=%d convert_shift=%d\n",
         audioProfile_name(g_runtime_settings.audio_profile),
         s_active_sample_rate_hz, s_active_convert_shift);

    return true;
}

void audioPipeline_stop() {
    LOGI("AP", "[AP] stop: drops=%lu i2s_err=%lu clips=%lu\n",
         (unsigned long)s_rb_drop_count,
         (unsigned long)s_i2s_error_count,
         (unsigned long)s_clip_count);
    if (g_i2s_task) { vTaskDelete(g_i2s_task); g_i2s_task = nullptr; }
    if (g_ringbuf)  { vRingbufferDelete(g_ringbuf); g_ringbuf = nullptr; }
    if (g_i2s_ok)   { i2s_driver_uninstall(I2S_PORT); g_i2s_ok = false; }
    g_rb_ok = false;
    LOGI("AP", "[AP] teardown complete\n");
}

void audioPipeline_applyHPF(int16_t* buf, size_t frames) {
    if (!s_hpf_enabled) return;
    for (size_t i = 0; i < frames; ++i) {
        int16_t x  = buf[i];
        int32_t yi = (int32_t)x
                   - (int32_t)s_hpf_prev_x_i16
                   + (int32_t)((s_hpf_a_q15 * s_hpf_prev_y_i32) >> 15);
        s_hpf_prev_x_i16 = x;
        s_hpf_prev_y_i32 = yi;
        if (yi >  32767) yi =  32767;
        if (yi < -32768) yi = -32768;
        buf[i] = (int16_t)yi;
    }
}

AudioMetrics audioPipeline_getMetrics() {
    AudioMetrics m;
    m.peak_level         = s_peak_level;
    m.peak_hold          = s_peak_hold;
    m.clip_count         = s_clip_count;
    m.clipped_last_block = s_clipped_last_block;
    m.i2s_error_count    = s_i2s_error_count;
    m.rb_drop_count      = s_rb_drop_count;
    return m;
}

void audioPipeline_resetPeakHold() {
    s_peak_hold = 0;
}

int audioPipeline_getActiveConvertShift() {
    return s_active_convert_shift;
}

int audioPipeline_getActiveSampleRateHz() {
    return s_active_sample_rate_hz;
}

size_t audioPipeline_getRingBufCapacityBytes() {
    return s_active_ringbuf_capacity ? s_active_ringbuf_capacity : RINGBUF_CAPACITY;
}

void audioPipeline_setHpfConfig(bool enabled, int cutoff_hz) {
    s_hpf_enabled = enabled;
    // Recompute Q15 coefficient for new cutoff
    const float fs = (float)s_active_sample_rate_hz;
    const float fc = (float)cutoff_hz;
    float R = expf(-2.0f * PI_F * fc / fs);
    float aq = R * 32768.0f;
    if (aq < 0.0f)     aq = 0.0f;
    if (aq > 32767.0f) aq = 32767.0f;
    s_hpf_a_q15 = (int32_t)lrintf(aq);
    // Reset filter state to avoid transient glitch
    s_hpf_prev_x_i16 = 0;
    s_hpf_prev_y_i32 = 0;
    LOGI("AP", "[HPF] updated: %s, fc=%d Hz (a_q15=%ld)\n",
         s_hpf_enabled ? "ENABLED" : "disabled", cutoff_hz, (long)s_hpf_a_q15);
}
