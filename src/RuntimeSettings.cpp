// RuntimeSettings.cpp — Load/save/mutate first-wave runtime settings.

#include "RuntimeSettings.h"
#include "AppState.h"
#include "AudioPipeline.h"

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <math.h>
#include "esp_wifi.h"

// --------------------------------------------------------
// Logging
// --------------------------------------------------------
#ifndef LOG_LEVEL
#define LOG_LEVEL 2
#endif
#if LOG_LEVEL >= 2
#define LOGI(fmt, ...) Serial.printf("[I][RT] " fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) Serial.printf("[W][RT] " fmt, ##__VA_ARGS__)
#else
#define LOGI(...) do {} while (0)
#define LOGW(...) do {} while (0)
#endif
#if LOG_LEVEL >= 1
#define LOGE(fmt, ...) Serial.printf("[E][RT] " fmt, ##__VA_ARGS__)
#else
#define LOGE(...) do {} while (0)
#endif

// --------------------------------------------------------
// Compile-time defaults (mirrors AudioPipeline / main defaults)
// --------------------------------------------------------
#ifndef WIFI_TX_POWER_DBM
#define WIFI_TX_POWER_DBM 15
#endif
#ifndef HPF_ENABLE
#define HPF_ENABLE 1
#endif
#ifndef HPF_CUTOFF_HZ
#define HPF_CUTOFF_HZ 100
#endif
#ifndef CONVERT_SHIFT
#define CONVERT_SHIFT 11
#endif

// --------------------------------------------------------
// Preferences keys
// --------------------------------------------------------
static const char* const RT_NS               = "runtime";
static const char* const KEY_WIFI_TX_POWER   = "wifi_tx_dbm";
static const char* const KEY_HPF_ENABLED     = "hpf_en";
static const char* const KEY_HPF_CUTOFF_HZ   = "hpf_hz";
static const char* const KEY_CONVERT_SHIFT   = "conv_shift";
static const char* const KEY_AUDIO_PROFILE   = "audio_prof";

// --------------------------------------------------------
// Global instance
// --------------------------------------------------------
RuntimeSettings g_runtime_settings = {
    .wifi_tx_power_dbm = (int8_t)WIFI_TX_POWER_DBM,
    .hpf_enabled       = (HPF_ENABLE != 0),
    .hpf_cutoff_hz     = (int16_t)HPF_CUTOFF_HZ,
    .convert_shift     = (int8_t)CONVERT_SHIFT,
    .audio_profile     = AUDIO_PROFILE_QUALITY_48K,
};

// --------------------------------------------------------
// Validation
// --------------------------------------------------------
bool runtimeSettings_validateWifiTxPower(int v) {
    return v >= -1 && v <= 20;
}

bool runtimeSettings_validateHpfCutoffHz(int v) {
    return v >= 20 && v <= 8000;
}

bool runtimeSettings_validateConvertShift(int v) {
    return v >= 1 && v <= 31;
}

bool runtimeSettings_validateAudioProfile(int v) {
    return v == (int)AUDIO_PROFILE_QUALITY_48K || v == (int)AUDIO_PROFILE_STABILITY_24K;
}

// --------------------------------------------------------
// Lifecycle
// --------------------------------------------------------
void runtimeSettings_load() {
    if (!g_prefs_inited) {
        LOGW("Preferences not inited; using compile-time defaults\n");
        return;
    }

    Preferences rt;
    rt.begin(RT_NS, true /*read-only*/);

    if (rt.isKey(KEY_WIFI_TX_POWER)) {
        int v = (int)rt.getChar(KEY_WIFI_TX_POWER, (int8_t)WIFI_TX_POWER_DBM);
        if (runtimeSettings_validateWifiTxPower(v)) {
            g_runtime_settings.wifi_tx_power_dbm = (int8_t)v;
        } else {
            LOGW("Stored wifi_tx_power_dbm=%d out of range; using default\n", v);
        }
    }

    if (rt.isKey(KEY_HPF_ENABLED)) {
        g_runtime_settings.hpf_enabled = rt.getBool(KEY_HPF_ENABLED, (HPF_ENABLE != 0));
    }

    if (rt.isKey(KEY_HPF_CUTOFF_HZ)) {
        int v = (int)rt.getShort(KEY_HPF_CUTOFF_HZ, (int16_t)HPF_CUTOFF_HZ);
        if (runtimeSettings_validateHpfCutoffHz(v)) {
            g_runtime_settings.hpf_cutoff_hz = (int16_t)v;
        } else {
            LOGW("Stored hpf_cutoff_hz=%d out of range; using default\n", v);
        }
    }

    if (rt.isKey(KEY_CONVERT_SHIFT)) {
        int v = (int)rt.getChar(KEY_CONVERT_SHIFT, (int8_t)CONVERT_SHIFT);
        if (runtimeSettings_validateConvertShift(v)) {
            g_runtime_settings.convert_shift = (int8_t)v;
        } else {
            LOGW("Stored convert_shift=%d out of range; using default\n", v);
        }
    }

    if (rt.isKey(KEY_AUDIO_PROFILE)) {
        int v = (int)rt.getUChar(KEY_AUDIO_PROFILE, (uint8_t)AUDIO_PROFILE_QUALITY_48K);
        if (runtimeSettings_validateAudioProfile(v)) {
            g_runtime_settings.audio_profile = (AudioProfile)v;
        } else {
            LOGW("Stored audio_profile=%d invalid; using default (quality_48k)\n", v);
        }
    }

    rt.end();
    LOGI("Loaded: wifi_tx_dbm=%d hpf_en=%d hpf_hz=%d convert_shift=%d audio_profile=%s\n",
         (int)g_runtime_settings.wifi_tx_power_dbm,
         (int)g_runtime_settings.hpf_enabled,
         (int)g_runtime_settings.hpf_cutoff_hz,
         (int)g_runtime_settings.convert_shift,
         audioProfile_name(g_runtime_settings.audio_profile));
}

void runtimeSettings_save() {
    if (!g_prefs_inited) {
        LOGW("Preferences not inited; cannot save\n");
        return;
    }

    Preferences rt;
    rt.begin(RT_NS, false /*read-write*/);
    rt.putChar(KEY_WIFI_TX_POWER, (int8_t)g_runtime_settings.wifi_tx_power_dbm);
    rt.putBool(KEY_HPF_ENABLED,   g_runtime_settings.hpf_enabled);
    rt.putShort(KEY_HPF_CUTOFF_HZ, (int16_t)g_runtime_settings.hpf_cutoff_hz);
    rt.putChar(KEY_CONVERT_SHIFT,  (int8_t)g_runtime_settings.convert_shift);
    rt.putUChar(KEY_AUDIO_PROFILE, (uint8_t)g_runtime_settings.audio_profile);
    rt.end();
}

// --------------------------------------------------------
// Wi-Fi TX power application
// --------------------------------------------------------
static wifi_power_t mapDbmToEnum(int dbm) {
    if (dbm >= 20) return WIFI_POWER_19_5dBm;
    if (dbm >= 19) return WIFI_POWER_19dBm;
    if (dbm >= 18) return WIFI_POWER_18_5dBm;
    if (dbm >= 17) return WIFI_POWER_17dBm;
    if (dbm >= 15) return WIFI_POWER_15dBm;
    if (dbm >= 13) return WIFI_POWER_13dBm;
    if (dbm >= 11) return WIFI_POWER_11dBm;
    if (dbm >= 9)  return WIFI_POWER_8_5dBm;
    if (dbm >= 7)  return WIFI_POWER_7dBm;
    if (dbm >= 5)  return WIFI_POWER_5dBm;
    if (dbm >= 2)  return WIFI_POWER_2dBm;
    return WIFI_POWER_MINUS_1dBm;
}

void runtimeSettings_applyWifiTxPower() {
    wifi_power_t txp = mapDbmToEnum((int)g_runtime_settings.wifi_tx_power_dbm);
    WiFi.setTxPower(txp);
    LOGI("wifi_tx_power_dbm applied: %d (enum=%d)\n",
         (int)g_runtime_settings.wifi_tx_power_dbm, (int)txp);
}

// --------------------------------------------------------
// Mutators
// --------------------------------------------------------

bool runtimeSettings_setWifiTxPowerDbm(int v, char* errmsg, size_t errmsg_sz) {
    if (!runtimeSettings_validateWifiTxPower(v)) {
        snprintf(errmsg, errmsg_sz, "wifi_tx_power_dbm must be in [-1, 20], got %d", v);
        return false;
    }
    g_runtime_settings.wifi_tx_power_dbm = (int8_t)v;
    runtimeSettings_save();
    runtimeSettings_applyWifiTxPower();
    LOGI("wifi_tx_power_dbm set to %d\n", v);
    return true;
}

bool runtimeSettings_setHpfEnabled(bool v, char* errmsg, size_t errmsg_sz) {
    (void)errmsg; (void)errmsg_sz; // always valid
    g_runtime_settings.hpf_enabled = v;
    runtimeSettings_save();
    // The AudioPipeline reads this via audioPipeline_setHpf() — declared below.
    // We call audioPipeline_applyHpfConfig() to propagate without restarting.
    audioPipeline_setHpfConfig(v, g_runtime_settings.hpf_cutoff_hz);
    LOGI("hpf_enabled set to %d\n", (int)v);
    return true;
}

bool runtimeSettings_setHpfCutoffHz(int v, char* errmsg, size_t errmsg_sz) {
    if (!runtimeSettings_validateHpfCutoffHz(v)) {
        snprintf(errmsg, errmsg_sz, "hpf_cutoff_hz must be in [20, 8000], got %d", v);
        return false;
    }
    g_runtime_settings.hpf_cutoff_hz = (int16_t)v;
    runtimeSettings_save();
    audioPipeline_setHpfConfig(g_runtime_settings.hpf_enabled, v);
    LOGI("hpf_cutoff_hz set to %d\n", v);
    return true;
}

bool runtimeSettings_setConvertShift(int v, char* errmsg, size_t errmsg_sz) {
    if (!runtimeSettings_validateConvertShift(v)) {
        snprintf(errmsg, errmsg_sz, "convert_shift must be in [1, 31], got %d", v);
        return false;
    }
    g_runtime_settings.convert_shift = (int8_t)v;
    runtimeSettings_save();
    // NOTE: does NOT hot-apply — restart-audio is required for this to take effect.
    LOGI("convert_shift configured to %d (restart-audio required)\n", v);
    return true;
}

bool runtimeSettings_setAudioProfile(int v, char* errmsg, size_t errmsg_sz) {
    if (!runtimeSettings_validateAudioProfile(v)) {
        snprintf(errmsg, errmsg_sz,
                 "audio_profile must be 0 (quality_48k) or 1 (stability_24k), got %d", v);
        return false;
    }
    g_runtime_settings.audio_profile = (AudioProfile)v;
    runtimeSettings_save();
    // NOTE: does NOT hot-apply — restart-audio is required for this to take effect.
    LOGI("audio_profile configured to %s (restart-audio required)\n",
         audioProfile_name(g_runtime_settings.audio_profile));
    return true;
}
