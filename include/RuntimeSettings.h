
#pragma once
// RuntimeSettings.h — First-wave runtime settings persisted in Preferences.
// All settings have safe compile-time defaults; Preferences values override on boot.
//
// Namespace: "runtime"  (separate from "sched" used by AppState)
//
// Supported settings:
//   wifi_tx_power_dbm  — int8,   range [-1, 20]
//   hpf_enabled        — bool
//   hpf_cutoff_hz      — int16,  range [20, 8000]
//   convert_shift      — int8,   range [1, 31]   (restart-audio required)
//   audio_profile      — uint8,  0=quality_48k, 1=stability_24k (restart-audio required)
//
// Intentionally deferred (compile-time only for now):
//   stream_wav_enable  — controls HTTP Content-Type; unsafe to change mid-stream.
//   log_level          — compile-time macros; runtime change needs new log arch.

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ------------------------------------------------------------
// Audio profile enum
// ------------------------------------------------------------
// AUDIO_PROFILE_QUALITY_48K  — 48 kHz, optimised for BirdNET-Go quality mode.
//   Provides higher spectral resolution and is the recommended default.
//   Requires ~2x the ring-buffer bandwidth vs. 24 kHz.
// AUDIO_PROFILE_STABILITY_24K — 24 kHz, reduces bandwidth and DMA pressure.
//   Use on marginal Wi-Fi or when the 48 kHz stream drops frequently.
enum AudioProfile : uint8_t {
    AUDIO_PROFILE_QUALITY_48K   = 0,
    AUDIO_PROFILE_STABILITY_24K = 1,
};

// Friendly name strings for the above values.
inline const char* audioProfile_name(AudioProfile p) {
    switch (p) {
        case AUDIO_PROFILE_QUALITY_48K:   return "quality_48k";
        case AUDIO_PROFILE_STABILITY_24K: return "stability_24k";
        default:                          return "unknown";
    }
}

// Return the sample rate in Hz for a given profile.
inline int audioProfile_sampleRateHz(AudioProfile p) {
    return (p == AUDIO_PROFILE_STABILITY_24K) ? 24000 : 48000;
}

// ------------------------------------------------------------
// Settings struct
// ------------------------------------------------------------
struct RuntimeSettings {
    int8_t      wifi_tx_power_dbm;   // Wi-Fi TX power target in dBm
    bool        hpf_enabled;         // High-pass filter enable
    int16_t     hpf_cutoff_hz;       // HPF cutoff frequency in Hz
    int8_t      convert_shift;       // I2S 32-bit to 16-bit right-shift (restart-audio required)
    AudioProfile audio_profile;      // BirdNET audio profile (restart-audio required)
};

// Global instance (defined in RuntimeSettings.cpp)
extern RuntimeSettings g_runtime_settings;

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------

// Load from Preferences into g_runtime_settings.
// Falls back to compile-time defaults for any missing key.
// Must be called after g_prefs is open (i.e. after appState_init()).
void runtimeSettings_load();

// Persist current g_runtime_settings to Preferences.
void runtimeSettings_save();

// ------------------------------------------------------------
// Validation helpers (return true if value is in range)
// ------------------------------------------------------------
bool runtimeSettings_validateWifiTxPower(int v);
bool runtimeSettings_validateHpfCutoffHz(int v);
bool runtimeSettings_validateConvertShift(int v);
bool runtimeSettings_validateAudioProfile(int v);

// Apply the current Wi-Fi TX power setting to the active Wi-Fi driver.
void runtimeSettings_applyWifiTxPower();

// ------------------------------------------------------------
// Mutators — validate, update g_runtime_settings, persist, and
// apply where possible without a full pipeline restart.
// Returns true on success; fills errmsg (up to errmsg_sz bytes) on failure.
// ------------------------------------------------------------
bool runtimeSettings_setWifiTxPowerDbm(int v, char* errmsg, size_t errmsg_sz);
bool runtimeSettings_setHpfEnabled(bool v, char* errmsg, size_t errmsg_sz);
bool runtimeSettings_setHpfCutoffHz(int v, char* errmsg, size_t errmsg_sz);
// convert_shift: persists value and returns true; does NOT hot-apply — restart-audio required.
bool runtimeSettings_setConvertShift(int v, char* errmsg, size_t errmsg_sz);
// audio_profile: persists value and returns true; does NOT hot-apply — restart-audio required.
bool runtimeSettings_setAudioProfile(int v, char* errmsg, size_t errmsg_sz);
