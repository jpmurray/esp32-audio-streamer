// StreamServer.cpp — Dedicated HTTP server for live audio streaming.
// Runs in its own FreeRTOS task on STREAM_PORT (default 81) so the
// control-plane server on port 80 stays responsive during active streaming.

#include "StreamServer.h"
#include "AppState.h"
#include "AudioPipeline.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

// ------------------------------------------------------------
// Logging (mirrors main.cpp convention)
// ------------------------------------------------------------
#ifndef LOG_LEVEL
#define LOG_LEVEL 2
#endif

#if LOG_LEVEL >= 3
#define LOGD(fmt, ...) Serial.printf("[D][SS] " fmt, ##__VA_ARGS__)
#else
#define LOGD(...) do {} while (0)
#endif

#if LOG_LEVEL >= 2
#define LOGI(fmt, ...) Serial.printf("[I][SS] " fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) Serial.printf("[W][SS] " fmt, ##__VA_ARGS__)
#else
#define LOGI(...) do {} while (0)
#define LOGW(...) do {} while (0)
#endif

#if LOG_LEVEL >= 1
#define LOGE(fmt, ...) Serial.printf("[E][SS] " fmt, ##__VA_ARGS__)
#else
#define LOGE(...) do {} while (0)
#endif

// ------------------------------------------------------------
// Compile-time configuration
// ------------------------------------------------------------
#ifndef STREAM_PORT
#define STREAM_PORT 81
#endif
#ifndef STREAM_WAV_ENABLE
#define STREAM_WAV_ENABLE 0
#endif
#ifndef SAMPLE_RATE_HZ
#define SAMPLE_RATE_HZ 48000
#endif

// FreeRTOS task configuration
#ifndef STREAM_TASK_STACK_WORDS
#define STREAM_TASK_STACK_WORDS 8192
#endif
#ifndef STREAM_TASK_PRIORITY
#define STREAM_TASK_PRIORITY 5
#endif
// Pin stream task to core 0 so it doesn't starve the Arduino loop on core 1.
#ifndef STREAM_TASK_CORE
#define STREAM_TASK_CORE 0
#endif

// ------------------------------------------------------------
// Hardening tunables
// Consecutive zero-byte writes before we give up on a stalled client.
#ifndef STREAM_WRITE_STALL_LIMIT
#define STREAM_WRITE_STALL_LIMIT 200
#endif
// Max consecutive ring-buffer receive timeouts (each 1 s) before treating
// the session as idle/dead and exiting.
#ifndef STREAM_IDLE_TIMEOUT_COUNT
#define STREAM_IDLE_TIMEOUT_COUNT 5
#endif

// ------------------------------------------------------------
// Module state
// ------------------------------------------------------------
volatile bool     g_stream_active        = false;
volatile uint32_t g_stream_connect_count = 0;
volatile uint32_t g_stream_tx_bytes      = 0;
volatile uint32_t g_stream_write_stalls  = 0;
volatile uint32_t g_stream_timeout_count = 0;
volatile bool     g_stream_stop_requested = false;

static WebServer  s_stream_server(STREAM_PORT);
static TaskHandle_t s_stream_task = nullptr;

// ------------------------------------------------------------
// /stream handler
// ------------------------------------------------------------
static void handleStream() {
    if (!g_rb_ok) {
        s_stream_server.send(503, "text/plain", "Ring buffer not ready");
        return;
    }

    // Reset per-session transmit counters.
    g_stream_tx_bytes     = 0;
    g_stream_write_stalls = 0;

    g_stream_active = true;
    g_stream_stop_requested = false;
    ++g_stream_connect_count;

    WiFiClient client = s_stream_server.client();
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

    if (STREAM_WAV_ENABLE) {
        const uint32_t sampleRate    = (uint32_t)SAMPLE_RATE_HZ;
        const uint16_t channels      = 1;
        const uint16_t bitsPerSample = 16;
        const uint32_t byteRate      = sampleRate * channels * (bitsPerSample / 8);
        const uint16_t blockAlign    = channels * (bitsPerSample / 8);
        const uint32_t riffSize      = 0xFFFFFFFF;
        const uint32_t dataSize      = 0xFFFFFFFF;
        uint8_t hdr[44];
        hdr[0]='R'; hdr[1]='I'; hdr[2]='F'; hdr[3]='F';
        hdr[4]=(uint8_t)(riffSize&0xFF); hdr[5]=(uint8_t)((riffSize>>8)&0xFF);
        hdr[6]=(uint8_t)((riffSize>>16)&0xFF); hdr[7]=(uint8_t)((riffSize>>24)&0xFF);
        hdr[8]='W'; hdr[9]='A'; hdr[10]='V'; hdr[11]='E';
        hdr[12]='f'; hdr[13]='m'; hdr[14]='t'; hdr[15]=' ';
        const uint32_t sc1=16;
        hdr[16]=(uint8_t)(sc1&0xFF); hdr[17]=(uint8_t)((sc1>>8)&0xFF);
        hdr[18]=(uint8_t)((sc1>>16)&0xFF); hdr[19]=(uint8_t)((sc1>>24)&0xFF);
        const uint16_t audioFmt=1;
        hdr[20]=(uint8_t)(audioFmt&0xFF); hdr[21]=(uint8_t)((audioFmt>>8)&0xFF);
        hdr[22]=(uint8_t)(channels&0xFF); hdr[23]=(uint8_t)((channels>>8)&0xFF);
        hdr[24]=(uint8_t)(sampleRate&0xFF); hdr[25]=(uint8_t)((sampleRate>>8)&0xFF);
        hdr[26]=(uint8_t)((sampleRate>>16)&0xFF); hdr[27]=(uint8_t)((sampleRate>>24)&0xFF);
        hdr[28]=(uint8_t)(byteRate&0xFF); hdr[29]=(uint8_t)((byteRate>>8)&0xFF);
        hdr[30]=(uint8_t)((byteRate>>16)&0xFF); hdr[31]=(uint8_t)((byteRate>>24)&0xFF);
        hdr[32]=(uint8_t)(blockAlign&0xFF); hdr[33]=(uint8_t)((blockAlign>>8)&0xFF);
        hdr[34]=(uint8_t)(bitsPerSample&0xFF); hdr[35]=(uint8_t)((bitsPerSample>>8)&0xFF);
        hdr[36]='d'; hdr[37]='a'; hdr[38]='t'; hdr[39]='a';
        hdr[40]=(uint8_t)(dataSize&0xFF); hdr[41]=(uint8_t)((dataSize>>8)&0xFF);
        hdr[42]=(uint8_t)((dataSize>>16)&0xFF); hdr[43]=(uint8_t)((dataSize>>24)&0xFF);
        client.write(hdr, sizeof(hdr));
    }

    // ----------------------------------------------------------------
    // Main streaming loop
    // ----------------------------------------------------------------
    int idle_count = 0;  // consecutive ring-buffer receive timeouts

    while (client.connected() && !g_stream_stop_requested) {
        size_t item_size = 0;
        int16_t* chunk = (int16_t*)xRingbufferReceive(g_ringbuf, &item_size, pdMS_TO_TICKS(1000));
        if (!chunk) {
            // Ring buffer receive timed out — no audio produced yet or pipeline
            // is restarting.  Count consecutive idle windows and bail out if
            // the session has been idle too long.
            ++idle_count;
            if (idle_count >= STREAM_IDLE_TIMEOUT_COUNT) {
                LOGW("Stream idle timeout (%d s), closing client\n",
                     (int)STREAM_IDLE_TIMEOUT_COUNT);
                ++g_stream_timeout_count;
                break;
            }
            yield();
            continue;
        }
        idle_count = 0;  // got data — reset idle window

        size_t frames = item_size / sizeof(int16_t);
        audioPipeline_applyHPF(chunk, frames);

        const uint8_t* p = reinterpret_cast<const uint8_t*>(chunk);
        size_t to_write = item_size;
        int stall_count = 0;  // consecutive zero-byte writes this chunk

        while (to_write > 0 && client.connected() && !g_stream_stop_requested) {
            size_t n = client.write(p, to_write);
            if (n == 0) {
                ++stall_count;
                ++g_stream_write_stalls;
                if (stall_count >= STREAM_WRITE_STALL_LIMIT) {
                    LOGW("Stream write stalled (%d retries), closing client\n",
                         (int)STREAM_WRITE_STALL_LIMIT);
                    ++g_stream_timeout_count;
                    to_write = 0;  // force inner loop exit
                    // Signal outer loop to exit too by faking disconnect.
                    client.stop();
                    break;
                }
                delay(1);
            } else {
                stall_count = 0;
                p += n;
                to_write -= n;
                g_stream_tx_bytes += (uint32_t)n;
            }
            yield();
        }
        vRingbufferReturnItem(g_ringbuf, (void*)chunk);
        yield();
    }

    // ----------------------------------------------------------------
    // Session cleanup
    // ----------------------------------------------------------------
    client.stop();
    g_stream_active = false;
    g_stream_stop_requested = false;  // clear for next session
    LOGI("Stream session ended (tx_bytes=%lu write_stalls=%lu)\n",
         (unsigned long)g_stream_tx_bytes,
         (unsigned long)g_stream_write_stalls);
}

// ------------------------------------------------------------
// Stream server task
// ------------------------------------------------------------
void streamServer_taskBody(void* /*arg*/) {
    s_stream_server.on("/stream", HTTP_GET, handleStream);
    s_stream_server.begin();
    LOGI("Stream server started on :%d\n", (int)STREAM_PORT);

    for (;;) {
        s_stream_server.handleClient();
        // Small yield — handleStream() is blocking so this tight loop only
        // runs when no client is connected.
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ------------------------------------------------------------
// Public init (called from setup())
// ------------------------------------------------------------
bool streamServer_init() {
    if (!g_i2s_ok || !g_rb_ok) {
        LOGW("Audio pipeline not ready; stream server will respond with 503\n");
        // Still start the server so clients get a proper error instead of a
        // connection refused.
    }

    BaseType_t rc = xTaskCreatePinnedToCore(
        streamServer_taskBody,
        "stream_srv",
        STREAM_TASK_STACK_WORDS,
        nullptr,
        STREAM_TASK_PRIORITY,
        &s_stream_task,
        STREAM_TASK_CORE
    );

    if (rc != pdPASS) {
        LOGE("Failed to create stream server task (err %d)\n", (int)rc);
        return false;
    }
    LOGI("Stream server task created (core %d, prio %d)\n",
         (int)STREAM_TASK_CORE, (int)STREAM_TASK_PRIORITY);
    return true;
}
