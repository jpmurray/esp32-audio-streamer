// AppState.cpp — definitions for shared application state.

#include "AppState.h"

// ------------------------------------------------------------
// Run-time flags
// ------------------------------------------------------------
bool g_i2s_ok  = false;
bool g_rb_ok   = false;

// ------------------------------------------------------------
// Ring buffer / producer task
// ------------------------------------------------------------
RingbufHandle_t g_ringbuf  = nullptr;
TaskHandle_t    g_i2s_task = nullptr;

// ------------------------------------------------------------
// Preferences
// ------------------------------------------------------------
Preferences g_prefs;
bool        g_prefs_inited = false;

// ------------------------------------------------------------
// RTC-retained scheduling state
// ------------------------------------------------------------
RTC_DATA_ATTR uint32_t g_boot_count        = 0;
RTC_DATA_ATTR time_t   g_today_dawn_utc    = 0;
RTC_DATA_ATTR time_t   g_today_dusk_utc    = 0;
RTC_DATA_ATTR time_t   g_tomorrow_dawn_utc = 0;
RTC_DATA_ATTR time_t   g_tomorrow_dusk_utc = 0;
RTC_DATA_ATTR uint32_t g_last_compute_ymd  = 0;
RTC_DATA_ATTR time_t   g_last_ntp_sync_utc = 0;
RTC_DATA_ATTR time_t   g_last_ntp_check_utc = 0;
RTC_DATA_ATTR uint8_t  g_last_mode         = 0;

// Monotonic (not RTC)
uint32_t g_boot_ms          = 0;
uint32_t g_next_ntp_retry_ms = 0;

// ------------------------------------------------------------
// Init
// ------------------------------------------------------------
void appState_init() {
    g_boot_ms = millis();
    g_boot_count++;
    if (!g_prefs_inited) {
        g_prefs.begin(PREF_NS, false);
        g_prefs_inited = true;
    }
}
