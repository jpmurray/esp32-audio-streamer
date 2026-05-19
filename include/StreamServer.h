#pragma once
// StreamServer.h — Dedicated audio stream server on a separate port.
// Owns the live /stream (HTTP) and /audio (RTSP/TCP) endpoints so the
// control-plane HTTP server on port 80 stays responsive during active streaming.

#include <stdint.h>
#include <stddef.h>

// ------------------------------------------------------------
// Stream response format
// ------------------------------------------------------------

enum StreamResponseFormat {
    STREAM_FORMAT_DEFAULT = 0,  // honours compile-time STREAM_WAV_ENABLE
    STREAM_FORMAT_WAV,          // always audio/x-wav + WAV header
    STREAM_FORMAT_L16,          // always audio/L16; rate=<hz>; channels=1
};

// Returns "wav" or "l16" depending on compile-time STREAM_WAV_ENABLE default.
const char* streamServer_defaultFormatName();

// ------------------------------------------------------------
// RTSP transport
// ------------------------------------------------------------

// RTSP/TCP server port (default 8554).  Compile-time override: -D RTSP_PORT=<n>.
// Set to 0 to disable the RTSP server entirely.
#ifndef RTSP_PORT
#define RTSP_PORT 8554
#endif

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------

// Call once from setup() after audioPipeline_init().
// Returns true if the server task was started successfully.
bool streamServer_init();

// Called by the stream server task; do not call from main code.
void streamServer_taskBody(void* /*arg*/);

// Called by the RTSP server task; do not call from main code.
void streamServer_rtspTaskBody(void* /*arg*/);

// ------------------------------------------------------------
// State (readable from other modules / status handler)
// ------------------------------------------------------------

// ------------------------------------------------------------
// Active transport tag
// ------------------------------------------------------------

enum StreamTransport : uint8_t {
    STREAM_TRANSPORT_NONE = 0,  // No active session
    STREAM_TRANSPORT_HTTP,      // HTTP chunked stream (port STREAM_PORT)
    STREAM_TRANSPORT_RTSP,      // RTSP/TCP stream (port RTSP_PORT)
};

// Human-readable string for a transport (never null).
const char* streamServer_transportName(StreamTransport t);

// True while at least one stream client is connected (HTTP or RTSP).
extern volatile bool g_stream_active;

// Which transport is currently serving audio (NONE when idle).
extern volatile StreamTransport g_stream_active_transport;

// Number of clients that have connected since boot (HTTP + RTSP combined).
extern volatile uint32_t g_stream_connect_count;

// Number of HTTP stream sessions started since boot.
extern volatile uint32_t g_http_connect_count;

// Number of RTSP sessions started since boot.
extern volatile uint32_t g_rtsp_connect_count;

// True while an RTSP client is actively streaming (between PLAY and TEARDOWN/disconnect).
extern volatile bool g_rtsp_streaming;

// ------------------------------------------------------------
// Transmit-side observability counters (reset on reconnect)
// ------------------------------------------------------------

// Total audio bytes written to the current (or last) client.
extern volatile uint32_t g_stream_tx_bytes;

// Number of zero-byte write stalls on the current (or last) session.
extern volatile uint32_t g_stream_write_stalls;

// Number of times the stream loop was kicked out due to stall/idle timeout.
extern volatile uint32_t g_stream_timeout_count;

// ------------------------------------------------------------
// Stream disconnect reason
// ------------------------------------------------------------

enum StreamDisconnectReason : uint8_t {
    STREAM_DISC_NONE = 0,           // No session has ended yet
    STREAM_DISC_CLIENT_CLOSED,      // Client closed the connection
    STREAM_DISC_IDLE_TIMEOUT,       // Ring-buffer starved for too long
    STREAM_DISC_WRITE_STALL,        // Consecutive zero-byte writes exceeded limit
    STREAM_DISC_RESTART_REQUESTED,  // Stop requested by control plane
    STREAM_DISC_WIFI_LOST,          // STA connection dropped mid-stream
    STREAM_DISC_AUDIO_UNAVAILABLE,  // Ring buffer not ready at session start
};

// Human-readable string for a disconnect reason (never null).
const char* streamServer_disconnectReasonName(StreamDisconnectReason r);

// ------------------------------------------------------------
// Stream lifecycle control
// ------------------------------------------------------------

// Set to true by HttpControl before audioPipeline_stop() to signal the
// stream loop to drain and exit cleanly.  Cleared by streamServer after
// the active client is dropped.  Checked by handleStream() so it exits
// promptly instead of racing the ring-buffer teardown.
extern volatile bool g_stream_stop_requested;

// Request the active stream session to stop and block until it exits or
// timeout_ms elapses.  Returns true if the session drained cleanly.
// Safe to call when no session is active (returns true immediately).
// Idempotent: a second call while stop is already pending is a no-op.
bool streamServer_requestStopAndWait(uint32_t timeout_ms);

// ------------------------------------------------------------
// Session diagnostics (read-only, updated at end of each session)
// ------------------------------------------------------------

// Reason the most recent stream session ended.  STREAM_DISC_NONE until the
// first session completes.  Updated by both HTTP and RTSP sessions.
extern volatile StreamDisconnectReason g_stream_last_disconnect_reason;

// millis() timestamp when the current (or last) stream session started.
extern volatile uint32_t g_stream_session_started_ms;

// Duration in ms of the last completed stream session (HTTP or RTSP).
extern volatile uint32_t g_stream_last_session_duration_ms;

// Audio bytes transmitted in the last completed stream session (HTTP or RTSP).
extern volatile uint32_t g_stream_last_session_tx_bytes;

// Transport that produced the most recent completed session.
extern volatile StreamTransport g_stream_last_transport;

// ------------------------------------------------------------
// Per-transport last-session diagnostics (RTSP)
// ------------------------------------------------------------

// Reason the most recent RTSP session ended.
extern volatile StreamDisconnectReason g_rtsp_last_disconnect_reason;

// Duration in ms of the last completed RTSP session.
extern volatile uint32_t g_rtsp_last_session_duration_ms;

// Audio bytes transmitted in the last completed RTSP session.
extern volatile uint32_t g_rtsp_last_session_tx_bytes;

// ------------------------------------------------------------
// Performance
// ------------------------------------------------------------

// Returns the stream task FreeRTOS stack high-water mark in bytes.
// Returns 0 if the task has not been created yet.
uint32_t streamServer_getTaskHighWaterMark();

// Returns the RTSP task FreeRTOS stack high-water mark in bytes.
// Returns 0 if the task has not been created yet.
uint32_t streamServer_getRtspTaskHighWaterMark();

// ------------------------------------------------------------
// Consumer-active query (used by AudioPipeline to gate ring-buffer sends)
// ------------------------------------------------------------

// Returns true when at least one audio consumer is actively receiving:
//   - HTTP stream is connected (g_stream_active + HTTP transport), or
//   - RTSP client has sent PLAY and is streaming (g_rtsp_streaming).
// False during idle / RTSP SETUP-but-not-playing / between sessions.
bool streamServer_audioConsumerActive();

// ------------------------------------------------------------
// Write-stall diagnostics (per-session + last-session)
// ------------------------------------------------------------

struct StreamWriteDiagnostics {
    uint32_t current_write_stalls;            // zero-byte write stalls in current session
    uint32_t current_max_consecutive_stalls;  // max run of consecutive stalls this session
    uint32_t last_session_write_stalls;       // stall count from the last completed session
    uint32_t last_session_max_consecutive_stalls; // max consecutive stalls in last session
    int      last_write_errno;                // best-effort errno after last zero-byte write
    uint32_t last_write_errno_ms;             // millis() when last_write_errno was captured
};

// Snapshot of current and last-session write-stall diagnostics.
void streamServer_getWriteDiagnostics(StreamWriteDiagnostics* out);
