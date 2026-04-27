#pragma once
// StreamServer.h — Dedicated audio stream server on a separate port.
// Owns the live /stream endpoint so the control-plane HTTP server on port 80
// stays responsive during active streaming.

#include <stdint.h>
#include <stddef.h>

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------

// Call once from setup() after audioPipeline_init().
// Returns true if the server task was started successfully.
bool streamServer_init();

// Called by the stream server task; do not call from main code.
void streamServer_taskBody(void* /*arg*/);

// ------------------------------------------------------------
// State (readable from other modules / status handler)
// ------------------------------------------------------------

// True while at least one stream client is connected.
extern volatile bool g_stream_active;

// Number of clients that have connected since boot.
extern volatile uint32_t g_stream_connect_count;

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
// Stream lifecycle control
// ------------------------------------------------------------

// Set to true by HttpControl before audioPipeline_stop() to signal the
// stream loop to drain and exit cleanly.  Cleared by streamServer after
// the active client is dropped.  Checked by handleStream() so it exits
// promptly instead of racing the ring-buffer teardown.
extern volatile bool g_stream_stop_requested;
