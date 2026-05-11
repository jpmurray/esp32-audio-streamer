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
//
// Intentionally deferred (compile-time only for now):
//   stream_wav_enable  — controls HTTP Content-Type; unsafe to change mid-stream.
//   log_level          — compile-time macros; runtime change needs new log arch.

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ------------------------------------------------------------
// Settings struct
// ------------------------------------------------------------
struct RuntimeSettings {
    int8_t  wifi_tx_power_dbm;   // Wi-Fi TX power target in dBm
    bool    hpf_enabled;         // High-pass filter enable
    int16_t hpf_cutoff_hz;       // HPF cutoff frequency in Hz
    int8_t  convert_shift;       // I2S 32-bit to 16-bit right-shift (restart-audio required)
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
