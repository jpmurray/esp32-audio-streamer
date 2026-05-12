// StreamServer.cpp — Dedicated HTTP + RTSP/TCP server for live audio streaming.
// HTTP task runs on STREAM_PORT (default 81).
// RTSP task runs on RTSP_PORT (default 8554) and serves RTP/AVP/TCP L16 mono.
// Both share the same PCM ring buffer and single-client gate (g_stream_active).

#include "StreamServer.h"
#include "AppState.h"
#include "AudioPipeline.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "LogBuffer.h"  // centralized LOGE/LOGW/LOGI/LOGD(module, fmt, ...)

// RTSP_PORT default is defined in StreamServer.h via #define, but we need it
// here before that include pulls it in.  Guard against redefinition.
#ifndef RTSP_PORT
#define RTSP_PORT 8554
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

// FreeRTOS task configuration (HTTP stream task)
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

// FreeRTOS task configuration (RTSP task)
#ifndef RTSP_TASK_STACK_WORDS
#define RTSP_TASK_STACK_WORDS 8192
#endif
#ifndef RTSP_TASK_PRIORITY
#define RTSP_TASK_PRIORITY 5
#endif
#ifndef RTSP_TASK_CORE
#define RTSP_TASK_CORE 0
#endif

// RTSP hardening: consecutive ring-buf receive timeouts (each 1 s) before
// treating the RTSP streaming session as dead.
#ifndef RTSP_IDLE_TIMEOUT_COUNT
#define RTSP_IDLE_TIMEOUT_COUNT 30
#endif

// Max consecutive write-failure retries inside the RTSP RTP send loop.
#ifndef RTSP_WRITE_STALL_LIMIT
#define RTSP_WRITE_STALL_LIMIT 500
#endif

// Inactivity timeout (ms) before we close a connected-but-not-playing RTSP
// client (i.e. stuck after SETUP, never sent PLAY).
#ifndef RTSP_SETUP_TIMEOUT_MS
#define RTSP_SETUP_TIMEOUT_MS 30000
#endif

// ------------------------------------------------------------
// Hardening tunables
// Consecutive zero-byte writes before we give up on a stalled client.
// Raised from 200 to 500 to tolerate short TCP back-pressure bursts that occur
// when BirdNET-Go is busy processing a segment.
#ifndef STREAM_WRITE_STALL_LIMIT
#define STREAM_WRITE_STALL_LIMIT 500
#endif
// Max consecutive ring-buffer receive timeouts (each 1 s) before treating
// the session as idle/dead and exiting.  Raised from 5 to 30 so a 30-second
// gap (e.g. pipeline restart or heavy processing pause) does not drop the
// stream client prematurely.
#ifndef STREAM_IDLE_TIMEOUT_COUNT
#define STREAM_IDLE_TIMEOUT_COUNT 30
#endif

// ------------------------------------------------------------
// Module state
// ------------------------------------------------------------
volatile bool            g_stream_active           = false;
volatile StreamTransport g_stream_active_transport = STREAM_TRANSPORT_NONE;
volatile uint32_t        g_stream_connect_count    = 0;
volatile uint32_t        g_http_connect_count      = 0;
volatile uint32_t        g_stream_tx_bytes         = 0;
volatile uint32_t        g_stream_write_stalls     = 0;
volatile uint32_t        g_stream_timeout_count    = 0;
volatile bool            g_stream_stop_requested   = false;

// RTSP-specific state
volatile uint32_t g_rtsp_connect_count = 0;
volatile bool     g_rtsp_streaming     = false;

// Combined last-session diagnostics
volatile StreamDisconnectReason g_stream_last_disconnect_reason   = STREAM_DISC_NONE;
volatile uint32_t               g_stream_session_started_ms       = 0;
volatile uint32_t               g_stream_last_session_duration_ms = 0;
volatile uint32_t               g_stream_last_session_tx_bytes    = 0;
volatile StreamTransport        g_stream_last_transport           = STREAM_TRANSPORT_NONE;

// RTSP per-transport last-session diagnostics
volatile StreamDisconnectReason g_rtsp_last_disconnect_reason   = STREAM_DISC_NONE;
volatile uint32_t               g_rtsp_last_session_duration_ms = 0;
volatile uint32_t               g_rtsp_last_session_tx_bytes    = 0;

static WebServer  s_stream_server(STREAM_PORT);
static TaskHandle_t s_stream_task = nullptr;

// RTSP server (raw TCP, port 8554)
static WiFiServer   s_rtsp_server(RTSP_PORT);
static TaskHandle_t s_rtsp_task = nullptr;

// ------------------------------------------------------------
// Disconnect reason name helper
// ------------------------------------------------------------
const char* streamServer_transportName(StreamTransport t) {
    switch (t) {
        case STREAM_TRANSPORT_NONE: return "none";
        case STREAM_TRANSPORT_HTTP: return "http";
        case STREAM_TRANSPORT_RTSP: return "rtsp";
        default:                    return "unknown";
    }
}

const char* streamServer_disconnectReasonName(StreamDisconnectReason r) {
    switch (r) {
        case STREAM_DISC_NONE:              return "none";
        case STREAM_DISC_CLIENT_CLOSED:     return "client_closed";
        case STREAM_DISC_IDLE_TIMEOUT:      return "idle_timeout";
        case STREAM_DISC_WRITE_STALL:       return "write_stall";
        case STREAM_DISC_RESTART_REQUESTED: return "restart_requested";
        case STREAM_DISC_WIFI_LOST:         return "wifi_lost";
        case STREAM_DISC_AUDIO_UNAVAILABLE: return "audio_unavailable";
        default:                            return "unknown";
    }
}

// ------------------------------------------------------------
// Format metadata helper
// ------------------------------------------------------------
const char* streamServer_defaultFormatName() {
    return (STREAM_WAV_ENABLE) ? "wav" : "l16";
}

// ------------------------------------------------------------
// Performance helper
// ------------------------------------------------------------
uint32_t streamServer_getTaskHighWaterMark() {
    return s_stream_task ? (uint32_t)uxTaskGetStackHighWaterMark(s_stream_task) : 0;
}

uint32_t streamServer_getRtspTaskHighWaterMark() {
    return s_rtsp_task ? (uint32_t)uxTaskGetStackHighWaterMark(s_rtsp_task) : 0;
}

// ------------------------------------------------------------
// Shared ring-buffer helper
// ------------------------------------------------------------
static size_t drainQueuedAudioBytes(size_t max_items = 256) {
    if (!g_ringbuf) return 0;

    size_t drained = 0;
    for (size_t i = 0; i < max_items; ++i) {
        size_t item_size = 0;
        void* item = xRingbufferReceive(g_ringbuf, &item_size, 0);
        if (!item) break;
        drained += item_size;
        vRingbufferReturnItem(g_ringbuf, item);
        yield();
    }
    return drained;
}

// ------------------------------------------------------------
// Shared streaming handler
// ------------------------------------------------------------
static void handleStreamFormat(StreamResponseFormat fmt) {
    if (!g_rb_ok) {
        g_stream_last_disconnect_reason = STREAM_DISC_AUDIO_UNAVAILABLE;
        s_stream_server.send(503, "text/plain", "Ring buffer not ready");
        return;
    }

    // Resolve STREAM_FORMAT_DEFAULT to the concrete format.
    bool useWav = (fmt == STREAM_FORMAT_WAV) ||
                  (fmt == STREAM_FORMAT_DEFAULT && STREAM_WAV_ENABLE);

    // Reset per-session transmit counters.
    g_stream_tx_bytes     = 0;
    g_stream_write_stalls = 0;

    g_stream_active = true;
    g_stream_active_transport = STREAM_TRANSPORT_HTTP;
    g_stream_stop_requested = false;
    g_stream_session_started_ms = (uint32_t)millis();
    ++g_stream_connect_count;
    ++g_http_connect_count;

    WiFiClient client = s_stream_server.client();
    client.setNoDelay(true);

    // The producer keeps filling the ring buffer even when nobody is
    // connected.  Start each live session at the newest audio instead of
    // dumping stale backlog in a burst; browsers and FFmpeg expect live audio
    // to arrive roughly in real time.
    size_t drained = drainQueuedAudioBytes();
    if (drained > 0) {
        LOGI("SS", "Stream: drained %u stale bytes before HTTP session\n", (unsigned)drained);
    }

    client.print("HTTP/1.1 200 OK\r\n");
    if (useWav) {
        client.print("Content-Type: audio/x-wav\r\n");
    } else {
        client.print("Content-Type: audio/L16; rate=");
        client.print(audioPipeline_getActiveSampleRateHz());
        client.print("; channels=1\r\n");
    }
    client.print("Cache-Control: no-store\r\nConnection: close\r\n\r\n");

    if (useWav) {
        const uint32_t sampleRate    = (uint32_t)audioPipeline_getActiveSampleRateHz();
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
    int idle_count = 0;   // consecutive ring-buffer receive timeouts
    int stall_count = 0;  // consecutive zero-byte writes across the session
    StreamDisconnectReason disc_reason = STREAM_DISC_CLIENT_CLOSED;

    while (client.connected() && !g_stream_stop_requested) {
        // Check for Wi-Fi loss (distinct from client TCP disconnect).
        if (WiFi.status() != WL_CONNECTED) {
            LOGW("SS", "Stream: Wi-Fi lost mid-session, closing client\n");
            disc_reason = STREAM_DISC_WIFI_LOST;
            break;
        }
        size_t item_size = 0;
        int16_t* chunk = (int16_t*)xRingbufferReceive(g_ringbuf, &item_size, pdMS_TO_TICKS(1000));
        if (!chunk) {
            // Ring buffer receive timed out — no audio produced yet or pipeline
            // is restarting.  Count consecutive idle windows and bail out if
            // the session has been idle too long.
            ++idle_count;
            if (idle_count >= STREAM_IDLE_TIMEOUT_COUNT) {
                LOGW("SS", "Stream idle timeout (%d s), closing client\n",
                     (int)STREAM_IDLE_TIMEOUT_COUNT);
                ++g_stream_timeout_count;
                disc_reason = STREAM_DISC_IDLE_TIMEOUT;
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

        while (to_write > 0 && client.connected() && !g_stream_stop_requested) {
            size_t n = client.write(p, to_write);
            if (n == 0) {
                ++stall_count;
                ++g_stream_write_stalls;
                if (stall_count >= STREAM_WRITE_STALL_LIMIT) {
                    LOGW("SS", "Stream write stalled (%d retries), closing client\n",
                         (int)STREAM_WRITE_STALL_LIMIT);
                    ++g_stream_timeout_count;
                    disc_reason = STREAM_DISC_WRITE_STALL;
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
    // Resolve final disconnect reason before clearing the stop flag.
    if (g_stream_stop_requested && disc_reason == STREAM_DISC_CLIENT_CLOSED) {
        disc_reason = STREAM_DISC_RESTART_REQUESTED;
    }
    client.stop();
    uint32_t session_end_ms = (uint32_t)millis();
    g_stream_last_session_tx_bytes    = g_stream_tx_bytes;
    g_stream_last_session_duration_ms = session_end_ms - g_stream_session_started_ms;
    g_stream_last_disconnect_reason   = disc_reason;
    g_stream_last_transport           = STREAM_TRANSPORT_HTTP;
    g_stream_active = false;
    g_stream_active_transport = STREAM_TRANSPORT_NONE;
    g_stream_stop_requested = false;  // clear for next session
    LOGI("SS", "Stream session ended: reason=%s tx_bytes=%lu duration_ms=%lu write_stalls=%lu\n",
         streamServer_disconnectReasonName(disc_reason),
         (unsigned long)g_stream_tx_bytes,
         (unsigned long)g_stream_last_session_duration_ms,
         (unsigned long)g_stream_write_stalls);
}

// ------------------------------------------------------------
// Public stop API
// ------------------------------------------------------------
bool streamServer_requestStopAndWait(uint32_t timeout_ms) {
    if (!g_stream_active) return true;  // no active session
    if (!g_stream_stop_requested) {
        LOGI("SS", "Stream stop requested (timeout %lu ms)\n", (unsigned long)timeout_ms);
        g_stream_stop_requested = true;
    }
    uint32_t start = (uint32_t)millis();
    while (g_stream_active && ((uint32_t)millis() - start) < timeout_ms) {
        delay(20);
    }
    bool stopped = !g_stream_active;
    LOGI("SS", "Stream stop %s after %lu ms\n",
         stopped ? "confirmed" : "timed out",
         (unsigned long)((uint32_t)millis() - start));
    return stopped;
}

// Thin wrappers so WebServer can hold zero-arg function pointers.
static void handleStream()    { handleStreamFormat(STREAM_FORMAT_DEFAULT); }
static void handleStreamWav() { handleStreamFormat(STREAM_FORMAT_WAV);     }
static void handleStreamPcm() { handleStreamFormat(STREAM_FORMAT_L16);     }

// ------------------------------------------------------------
// Stream server task
// ------------------------------------------------------------
void streamServer_taskBody(void* /*arg*/) {
    s_stream_server.on("/stream",     HTTP_GET, handleStream);
    s_stream_server.on("/stream.wav", HTTP_GET, handleStreamWav);
    s_stream_server.on("/stream.pcm", HTTP_GET, handleStreamPcm);
    s_stream_server.begin();
    LOGI("SS", "Stream server started on :%d\n", (int)STREAM_PORT);

    for (;;) {
        s_stream_server.handleClient();
        // Small yield — handleStream() is blocking so this tight loop only
        // runs when no client is connected.
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ------------------------------------------------------------
// RTSP/TCP helpers
// ------------------------------------------------------------

// Write exactly `len` bytes; retry on partial writes.  Returns false on failure.
static bool rtsp_writeAll(WiFiClient& client, const uint8_t* buf, size_t len) {
    int stalls = 0;
    while (len > 0 && client.connected()) {
        size_t n = client.write(buf, len);
        if (n == 0) {
            if (++stalls >= RTSP_WRITE_STALL_LIMIT) return false;
            delay(1);
        } else {
            stalls = 0;
            buf += n;
            len -= n;
            g_stream_tx_bytes += (uint32_t)n;
        }
        yield();
    }
    return (len == 0);
}

static bool rtsp_headerNameMatches(const char* p, const char* name) {
    while (*name) {
        char a = *p++;
        char b = *name++;
        if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
        if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
        if (a != b) return false;
    }
    return *p == ':';
}

static String rtsp_extractHeader(const char* req, const char* name) {
    const char* line = req;
    while (line && *line) {
        const char* next = strstr(line, "\r\n");
        if (rtsp_headerNameMatches(line, name)) {
            const char* p = strchr(line, ':');
            if (!p) return String();
            ++p;
            while (*p == ' ' || *p == '\t') ++p;
            const char* end = next ? next : (line + strlen(line));
            String v;
            while (p < end) v += *p++;
            v.trim();
            return v;
        }
        if (!next) break;
        line = next + 2;
    }
    return String();
}

// Extract CSeq value from a raw RTSP request block.
static String rtsp_extractCSeq(const char* req) {
    String v = rtsp_extractHeader(req, "CSeq");
    return v.length() ? v : String("1");
}

static String rtsp_extractMethod(const char* req) {
    String m;
    const char* p = req;
    while (*p && *p != ' ' && *p != '\r' && *p != '\n' && m.length() < 16) {
        m += *p++;
    }
    return m;
}

// Send an RTSP RTP/AVP/TCP interleaved packet wrapping L16 PCM.
// The PCM data pointed to by `chunk` is byte-swapped in place (big-endian
// for network) then restored; caller must not use `chunk` after this call
// returns until the buffer is re-acquired from the ring buffer anyway.
static bool rtsp_sendRtpPacket(WiFiClient& client,
                                int16_t* chunk, size_t frames,
                                uint16_t& seqno, uint32_t& timestamp,
                                uint32_t ssrc) {
    const uint16_t payloadBytes = (uint16_t)(frames * sizeof(int16_t));
    const uint16_t rtpLen       = (uint16_t)(12 + payloadBytes);

    // RTSP interleaved framing: '$' channel(0) length(2 bytes big-endian)
    uint8_t framing[4];
    framing[0] = 0x24;
    framing[1] = 0x00;
    framing[2] = (uint8_t)((rtpLen >> 8) & 0xFF);
    framing[3] = (uint8_t)(rtpLen & 0xFF);

    // RTP header (RFC 3550)
    uint8_t hdr[12];
    hdr[0] = 0x80;           // V=2 P=0 X=0 CC=0
    hdr[1] = 96;             // M=0 PT=96 (dynamic, L16)
    hdr[2] = (uint8_t)((seqno >> 8) & 0xFF);
    hdr[3] = (uint8_t)(seqno & 0xFF);
    hdr[4] = (uint8_t)((timestamp >> 24) & 0xFF);
    hdr[5] = (uint8_t)((timestamp >> 16) & 0xFF);
    hdr[6] = (uint8_t)((timestamp >> 8) & 0xFF);
    hdr[7] = (uint8_t)(timestamp & 0xFF);
    hdr[8]  = (uint8_t)((ssrc >> 24) & 0xFF);
    hdr[9]  = (uint8_t)((ssrc >> 16) & 0xFF);
    hdr[10] = (uint8_t)((ssrc >> 8) & 0xFF);
    hdr[11] = (uint8_t)(ssrc & 0xFF);

    // L16 requires big-endian samples; swap in place.
    for (size_t i = 0; i < frames; ++i) {
        uint16_t s = (uint16_t)chunk[i];
        chunk[i] = (int16_t)((s << 8) | (s >> 8));
    }

    bool ok = rtsp_writeAll(client, framing, sizeof(framing)) &&
              rtsp_writeAll(client, hdr, sizeof(hdr)) &&
              rtsp_writeAll(client, (const uint8_t*)chunk, payloadBytes);

    seqno++;
    timestamp += (uint32_t)frames;
    return ok;
}

static bool rtsp_isMethodStart(uint8_t b) {
    return b == 'O' || b == 'D' || b == 'S' || b == 'P' || b == 'T' || b == 'G';
}

// Consume one RTSP interleaved binary frame sent by the client (usually RTCP on
// channel 1).  RTSP/TCP carries these as '$', channel, length_hi, length_lo,
// payload.  They are not RTSP text and must not be appended to parseBuf.
static bool rtsp_discardClientInterleavedFrame(WiFiClient& client) {
    if (client.available() < 4) return false;
    if (client.peek() != '$') return false;

    uint8_t hdr[4];
    for (size_t i = 0; i < sizeof(hdr); ++i) {
        int b = client.read();
        if (b < 0) return false;
        hdr[i] = (uint8_t)b;
    }
    uint16_t len = ((uint16_t)hdr[2] << 8) | hdr[3];
    uint16_t remaining = len;
    uint8_t scratch[64];
    uint32_t start = (uint32_t)millis();
    while (remaining > 0 && client.connected()) {
        int avail = client.available();
        if (avail <= 0) {
            if (((uint32_t)millis() - start) > 1000) return false;
            delay(1);
            continue;
        }
        size_t n = (size_t)avail;
        if (n > sizeof(scratch)) n = sizeof(scratch);
        if (n > remaining) n = remaining;
        int got = client.read(scratch, n);
        if (got <= 0) return false;
        remaining -= (uint16_t)got;
        yield();
    }
    LOGD("SS", "RTSP: discarded client interleaved frame ch=%u len=%u\n",
         (unsigned)hdr[1], (unsigned)len);
    return true;
}

// Handle one complete RTSP request block.  Returns true as long as the
// session should continue; false on TEARDOWN or unrecoverable error.
static bool rtsp_handleRequest(WiFiClient& client,
                               const char* req,
                               const char* deviceIp,
                               String& sessionId,
                               bool& playing) {
    String cseq = rtsp_extractCSeq(req);
    String method = rtsp_extractMethod(req);
    LOGI("SS", "RTSP request: %s CSeq=%s\n", method.c_str(), cseq.c_str());

    if (strncmp(req, "OPTIONS", 7) == 0) {
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: "); client.print(cseq); client.print("\r\n");
        client.print("Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN, GET_PARAMETER\r\n\r\n");

    } else if (strncmp(req, "DESCRIBE", 8) == 0) {
        uint32_t sr = (uint32_t)audioPipeline_getActiveSampleRateHz();
        // Build SDP
        String sdp;
        sdp.reserve(256);
        sdp += "v=0\r\n";
        sdp += "o=- 0 0 IN IP4 "; sdp += deviceIp; sdp += "\r\n";
        sdp += "s=ESP32 Audio Streamer ("; sdp += sr; sdp += " Hz L16 mono)\r\n";
        sdp += "c=IN IP4 "; sdp += deviceIp; sdp += "\r\n";
        sdp += "t=0 0\r\n";
        sdp += "m=audio 0 RTP/AVP 96\r\n";
        sdp += "a=rtpmap:96 L16/"; sdp += sr; sdp += "/1\r\n";
        sdp += "a=control:track1\r\n";

        String contentBase = "rtsp://";
        contentBase += deviceIp; contentBase += ":"; contentBase += RTSP_PORT; contentBase += "/audio/";

        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: "); client.print(cseq); client.print("\r\n");
        client.print("Content-Type: application/sdp\r\n");
        client.print("Content-Base: "); client.print(contentBase); client.print("\r\n");
        client.print("Content-Length: "); client.print((uint32_t)sdp.length()); client.print("\r\n\r\n");
        client.print(sdp);

    } else if (strncmp(req, "SETUP", 5) == 0) {
        String transport = rtsp_extractHeader(req, "Transport");
        LOGI("SS", "RTSP SETUP Transport: %s\n", transport.length() ? transport.c_str() : "(missing)");

        String transport_lc = transport;
        transport_lc.toLowerCase();
        const bool wantsTcp = transport_lc.indexOf("rtp/avp/tcp") >= 0 ||
                              transport_lc.indexOf("interleaved=") >= 0;
        if (!wantsTcp) {
            client.print("RTSP/1.0 461 Unsupported Transport\r\n");
            client.print("CSeq: "); client.print(cseq); client.print("\r\n");
            client.print("Connection: close\r\n\r\n");
            LOGW("SS", "RTSP SETUP rejected: client did not request RTP/AVP/TCP interleaved\n");
            return false;
        }

        // Generate a session ID for this client.
        char sid[12];
        snprintf(sid, sizeof(sid), "%09lu", (unsigned long)random(100000000L, 999999999L));
        sessionId = String(sid);

        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: "); client.print(cseq); client.print("\r\n");
        client.print("Session: "); client.print(sessionId); client.print(";timeout=60\r\n");
        client.print("Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n\r\n");

    } else if (strncmp(req, "PLAY", 4) == 0) {
        if (!sessionId.length()) {
            client.print("RTSP/1.0 454 Session Not Found\r\n");
            client.print("CSeq: "); client.print(cseq); client.print("\r\n\r\n");
            return false;
        }
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: "); client.print(cseq); client.print("\r\n");
        client.print("Session: "); client.print(sessionId); client.print("\r\n");
        client.print("Range: npt=0.000-\r\n");
        // RFC 2326 §12.33: RTP-Info is required in PLAY responses so clients
        // (e.g. VLC) know the initial RTP sequence number and timestamp.
        client.print("RTP-Info: url=rtsp://");
        client.print(deviceIp);
        client.print(":"); client.print(RTSP_PORT);
        client.print("/audio/track1;seq=0;rtptime=0\r\n\r\n");
        playing = true;
        g_rtsp_streaming = true;
        size_t drained = drainQueuedAudioBytes();
        if (drained > 0) {
            LOGI("SS", "RTSP: drained %u stale bytes before PLAY\n", (unsigned)drained);
        }
        LOGI("SS", "RTSP PLAY: client=%s session=%s\n",
             client.remoteIP().toString().c_str(), sessionId.c_str());

    } else if (strncmp(req, "TEARDOWN", 8) == 0) {
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: "); client.print(cseq); client.print("\r\n");
        client.print("Session: "); client.print(sessionId); client.print("\r\n\r\n");
        playing = false;
        g_rtsp_streaming = false;
        LOGI("SS", "RTSP TEARDOWN: client=%s\n", client.remoteIP().toString().c_str());
        return false;  // session done

    } else if (strncmp(req, "GET_PARAMETER", 13) == 0) {
        // Many clients send this as a keep-alive.
        client.print("RTSP/1.0 200 OK\r\n");
        client.print("CSeq: "); client.print(cseq); client.print("\r\n\r\n");

    } else {
        client.print("RTSP/1.0 501 Not Implemented\r\n");
        client.print("CSeq: "); client.print(cseq); client.print("\r\n\r\n");
        LOGW("SS", "RTSP unknown method from %s\n", client.remoteIP().toString().c_str());
    }

    return true;  // session continues
}

// ------------------------------------------------------------
// RTSP server task
// ------------------------------------------------------------
void streamServer_rtspTaskBody(void* /*arg*/) {
#if RTSP_PORT == 0
    LOGI("SS", "RTSP server disabled (RTSP_PORT=0)\n");
    vTaskDelete(nullptr);
    return;
#endif

    s_rtsp_server.begin();
    s_rtsp_server.setNoDelay(true);
    LOGI("SS", "RTSP server started on :%d\n", (int)RTSP_PORT);

    // Per-client RTSP parse buffer
    static uint8_t parseBuf[1024];

    for (;;) {
        // --------------------------------------------------------
        // Accept phase: wait for an incoming TCP connection.
        // --------------------------------------------------------
        WiFiClient client = s_rtsp_server.available();
        if (!client) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Single-client gate: if HTTP stream or another RTSP session is
        // already active, politely refuse by closing immediately.
        if (g_stream_active) {
            LOGW("SS", "RTSP: client rejected (stream already active)\n");
            client.stop();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (!g_rb_ok) {
            LOGW("SS", "RTSP: ring buffer not ready, dropping client\n");
            client.stop();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // --------------------------------------------------------
        // Session setup
        // --------------------------------------------------------
        client.setNoDelay(true);
        ++g_rtsp_connect_count;
        ++g_stream_connect_count;
        g_stream_active = true;
        g_stream_active_transport = STREAM_TRANSPORT_RTSP;
        g_stream_tx_bytes = 0;
        g_stream_write_stalls = 0;
        g_stream_session_started_ms = (uint32_t)millis();

        char deviceIp[16] = "";
        {
            IPAddress ip = WiFi.localIP();
            snprintf(deviceIp, sizeof(deviceIp), "%d.%d.%d.%d",
                     ip[0], ip[1], ip[2], ip[3]);
        }

        String sessionId;
        bool playing = false;

        // RTP state
        uint16_t rtpSeqno    = 0;
        uint32_t rtpTimestamp = 0;
        const uint32_t rtpSSRC = 0x45535033UL;  // "ESP3"

        int    parseBufPos  = 0;
        uint32_t lastActivity = (uint32_t)millis();

        String clientIp = client.remoteIP().toString();
        LOGI("SS", "RTSP client connected: %s\n", clientIp.c_str());

        // --------------------------------------------------------
        // Session loop: interleave RTSP control parsing + RTP audio
        // --------------------------------------------------------
        bool sessionOk = true;
        int idle_count = 0;

        while (sessionOk && client.connected() && !g_stream_stop_requested) {
            // Wi-Fi loss check
            if (WiFi.status() != WL_CONNECTED) {
                LOGW("SS", "RTSP: Wi-Fi lost mid-session\n");
                g_stream_last_disconnect_reason = STREAM_DISC_WIFI_LOST;
                break;
            }

            // ---- RTSP command parsing ----
            // RTSP/TCP can also carry client->server interleaved binary frames
            // (typically RTCP receiver reports).  Keep those out of the text
            // request parser or they can be mistaken for malformed RTSP.
            while (client.connected() && client.available() && parseBufPos < (int)sizeof(parseBuf) - 1) {
                int next = client.peek();
                if (next < 0) break;

                if (parseBufPos == 0 && next == '$') {
                    if (!rtsp_discardClientInterleavedFrame(client)) break;
                    lastActivity = (uint32_t)millis();
                    continue;
                }

                if (parseBufPos == 0 && !rtsp_isMethodStart((uint8_t)next)) {
                    // Drop noise defensively until a plausible RTSP method
                    // starts.  This also recovers from partial binary garbage.
                    (void)client.read();
                    lastActivity = (uint32_t)millis();
                    continue;
                }

                parseBuf[parseBufPos++] = (uint8_t)client.read();
                lastActivity = (uint32_t)millis();
            }

            // Look for end of RTSP header block (\r\n\r\n)
            parseBuf[parseBufPos] = '\0';
            char* endHdr = strstr((char*)parseBuf, "\r\n\r\n");
            if (endHdr) {
                *endHdr = '\0';
                sessionOk = rtsp_handleRequest(client, (const char*)parseBuf,
                                               deviceIp, sessionId, playing);
                // Consume the header from the buffer.
                int consumed = (int)(endHdr - (char*)parseBuf) + 4;
                if (consumed < parseBufPos) {
                    memmove(parseBuf, parseBuf + consumed, parseBufPos - consumed);
                    parseBufPos -= consumed;
                } else {
                    parseBufPos = 0;
                }
                lastActivity = (uint32_t)millis();
            }

            // Buffer overflow guard
            if (parseBufPos >= (int)sizeof(parseBuf) - 1) {
                LOGW("SS", "RTSP: parse buffer overflow, resetting\n");
                parseBufPos = 0;
            }

            // Inactivity timeout for non-playing clients (e.g. stuck at SETUP)
            if (!playing &&
                ((uint32_t)millis() - lastActivity) > (uint32_t)RTSP_SETUP_TIMEOUT_MS) {
                LOGW("SS", "RTSP: inactivity timeout before PLAY\n");
                g_stream_last_disconnect_reason = STREAM_DISC_IDLE_TIMEOUT;
                break;
            }

            // ---- RTP audio (only when client sent PLAY) ----
            if (!playing || !g_rb_ok) {
                yield();
                continue;
            }

            size_t item_size = 0;
            int16_t* chunk = (int16_t*)xRingbufferReceive(
                g_ringbuf, &item_size, pdMS_TO_TICKS(1000));

            if (!chunk) {
                ++idle_count;
                if (idle_count >= RTSP_IDLE_TIMEOUT_COUNT) {
                    LOGW("SS", "RTSP: idle timeout (%d s), closing client\n",
                         (int)RTSP_IDLE_TIMEOUT_COUNT);
                    ++g_stream_timeout_count;
                    g_stream_last_disconnect_reason = STREAM_DISC_IDLE_TIMEOUT;
                    sessionOk = false;
                }
                yield();
                continue;
            }
            idle_count = 0;

            size_t frames = item_size / sizeof(int16_t);
            audioPipeline_applyHPF(chunk, frames);

            bool writeOk = rtsp_sendRtpPacket(client, chunk, frames,
                                               rtpSeqno, rtpTimestamp, rtpSSRC);
            vRingbufferReturnItem(g_ringbuf, (void*)chunk);

            if (!writeOk) {
            LOGW("SS", "RTSP: write failed, closing client\n");
                ++g_stream_write_stalls;
                g_stream_last_disconnect_reason = STREAM_DISC_WRITE_STALL;
                sessionOk = false;
            }
            yield();
        }

        // --------------------------------------------------------
        // Session cleanup
        // --------------------------------------------------------
        if (g_stream_stop_requested &&
            g_stream_last_disconnect_reason == STREAM_DISC_NONE) {
            g_stream_last_disconnect_reason = STREAM_DISC_RESTART_REQUESTED;
        } else if (!client.connected() &&
                   g_stream_last_disconnect_reason == STREAM_DISC_NONE) {
            g_stream_last_disconnect_reason = STREAM_DISC_CLIENT_CLOSED;
        }

        client.stop();
        playing = false;
        g_rtsp_streaming = false;

        uint32_t session_end_ms = (uint32_t)millis();
        g_stream_last_session_tx_bytes    = g_stream_tx_bytes;
        g_stream_last_session_duration_ms = session_end_ms - g_stream_session_started_ms;
        g_rtsp_last_session_tx_bytes      = g_stream_tx_bytes;
        g_rtsp_last_session_duration_ms   = session_end_ms - g_stream_session_started_ms;
        g_rtsp_last_disconnect_reason     = g_stream_last_disconnect_reason;
        g_stream_last_transport           = STREAM_TRANSPORT_RTSP;
        g_stream_active = false;
        g_stream_active_transport = STREAM_TRANSPORT_NONE;
        g_stream_stop_requested = false;

        LOGI("SS", "RTSP session ended: client=%s reason=%s tx_bytes=%lu duration_ms=%lu\n",
             clientIp.c_str(),
             streamServer_disconnectReasonName(g_stream_last_disconnect_reason),
             (unsigned long)g_stream_tx_bytes,
             (unsigned long)g_stream_last_session_duration_ms);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ------------------------------------------------------------
// Public init (called from setup())
// ------------------------------------------------------------
bool streamServer_init() {
    if (!g_i2s_ok || !g_rb_ok) {
        LOGW("SS", "Audio pipeline not ready; stream server will respond with 503\n");
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
        LOGE("SS", "Failed to create stream server task (err %d)\n", (int)rc);
        return false;
    }
    LOGI("SS", "Stream server task created (core %d, prio %d)\n",
         (int)STREAM_TASK_CORE, (int)STREAM_TASK_PRIORITY);

#if RTSP_PORT != 0
    BaseType_t rcR = xTaskCreatePinnedToCore(
        streamServer_rtspTaskBody,
        "rtsp_srv",
        RTSP_TASK_STACK_WORDS,
        nullptr,
        RTSP_TASK_PRIORITY,
        &s_rtsp_task,
        RTSP_TASK_CORE
    );

    if (rcR != pdPASS) {
        LOGE("SS", "Failed to create RTSP server task (err %d)\n", (int)rcR);
        // Non-fatal: HTTP stream still works.
    } else {
        LOGI("SS", "RTSP server task created (core %d, prio %d)\n",
             (int)RTSP_TASK_CORE, (int)RTSP_TASK_PRIORITY);
    }
#endif

    return true;
}
