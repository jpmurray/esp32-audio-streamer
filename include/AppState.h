#pragma once
// AppState.h — shared application state visible across modules.
// Prefer plain globals with a clear init function over a singleton class.

#include <Arduino.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"

// ------------------------------------------------------------
// Preferences namespace / keys (kept here so all modules agree)
// ------------------------------------------------------------
static const char* const PREF_NS            = "sched";
static const char* const PREF_KEY_LAST_WAKES  = "last_wakes";
static const char* const PREF_KEY_NEXT_SLEEPS = "next_sleeps";

// ------------------------------------------------------------
// Run-time flags
// ------------------------------------------------------------
extern bool g_i2s_ok;
extern bool g_rb_ok;

// ------------------------------------------------------------
// Ring buffer / I2S producer task
// ------------------------------------------------------------
extern RingbufHandle_t g_ringbuf;
extern TaskHandle_t    g_i2s_task;

// ------------------------------------------------------------
// Preferences handle
// ------------------------------------------------------------
extern Preferences g_prefs;
extern bool        g_prefs_inited;

// ------------------------------------------------------------
// Scheduling / RTC retained state
// (RTC_DATA_ATTR variables must be defined in exactly one .cpp)
// ------------------------------------------------------------
extern uint32_t g_boot_count;     // RTC_DATA_ATTR — defined in AppState.cpp
extern time_t   g_today_dawn_utc;
extern time_t   g_today_dusk_utc;
extern time_t   g_tomorrow_dawn_utc;
extern time_t   g_tomorrow_dusk_utc;
extern uint32_t g_last_compute_ymd;
extern time_t   g_last_ntp_sync_utc;
extern time_t   g_last_ntp_check_utc;
extern uint8_t  g_last_mode;

// Monotonic boot timestamp (not RTC)
extern uint32_t g_boot_ms;

// NTP retry scheduler (millis-based)
extern uint32_t g_next_ntp_retry_ms;

// ------------------------------------------------------------
// One-time init (call from setup() before anything else)
// ------------------------------------------------------------
void appState_init();
