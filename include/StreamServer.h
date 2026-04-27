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
