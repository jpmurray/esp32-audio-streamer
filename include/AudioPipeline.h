#pragma once
// AudioPipeline.h — I2S microphone, ring buffer, and HPF DSP pipeline.

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// ------------------------------------------------------------
// Runtime audio metrics (updated by the I2S producer task)
// Metrics are measured on the converted PCM-16 samples BEFORE HPF is applied
// (HPF is applied later in StreamServer at transmission time).
// ------------------------------------------------------------

struct AudioMetrics {
    int16_t  peak_level;        // peak |sample| seen since boot
    int16_t  peak_hold;         // peak |sample| since last call to audioPipeline_resetPeakHold()
    uint32_t clip_count;        // cumulative samples at ±32767
    bool     clipped_last_block;// true if any clip occurred in the most-recent chunk
    uint32_t i2s_error_count;   // cumulative i2s_read() failures
    uint32_t rb_drop_count;     // cumulative xRingbufferSend() drops (ring buffer full)
    // Idle discard: samples converted but not sent to ring buffer (no active consumer).
    uint32_t idle_discard_count; // cumulative chunks discarded while no consumer was active
    uint32_t idle_discard_bytes; // cumulative PCM bytes discarded while no consumer was active
};

// Snapshot current metrics (thread-safe copy).
AudioMetrics audioPipeline_getMetrics();

// Reset the peak-hold accumulator only (other counters are not reset).
void audioPipeline_resetPeakHold();

// ------------------------------------------------------------
// Init / teardown
// ------------------------------------------------------------

// Initialise I2S driver, create ring buffer, start producer task, and
// compute HPF coefficients.  Returns true on full success.
bool audioPipeline_init();

// Stop producer task, delete ring buffer, uninstall I2S driver.
void audioPipeline_stop();

// ------------------------------------------------------------
// DSP — high-pass filter
// Apply in-place HPF to `frames` samples stored in `buf`.
// No-op when HPF is disabled.
// ------------------------------------------------------------
void audioPipeline_applyHPF(int16_t* buf, size_t frames);

// Update HPF enabled flag and recompute coefficients for new cutoff frequency.
// Safe to call at runtime; takes effect on the next audio chunk.
void audioPipeline_setHpfConfig(bool enabled, int cutoff_hz);

// Return the convert_shift value captured at the last audioPipeline_init() call.
// This is the value currently active in the running producer task.
int audioPipeline_getActiveConvertShift();

// Return the sample rate (Hz) captured at the last audioPipeline_init() call.
// Reflects the runtime audio profile; always 48000 or 24000.
int audioPipeline_getActiveSampleRateHz();

// Return the ring buffer capacity in bytes as configured at the last
// audioPipeline_init() call.  Reflects any RB_CAPACITY_BYTES override.
size_t audioPipeline_getRingBufCapacityBytes();
