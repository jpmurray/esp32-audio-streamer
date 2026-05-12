#pragma once
// Scheduler.h — NTP sync, civil dawn/dusk solar computation, and sleep control.

#include <Arduino.h>
#include <time.h>

// ------------------------------------------------------------
// Time helpers
// ------------------------------------------------------------
bool  scheduler_timeIsValid();
void  scheduler_formatIso8601UTC(time_t t, char* out, size_t out_sz);
void  scheduler_formatIso8601Local(time_t t, char* out, size_t out_sz);

// ------------------------------------------------------------
// NTP
// ------------------------------------------------------------
void scheduler_maybeSyncNtp();

// ------------------------------------------------------------
// Schedule maintenance
// Recomputes dawn/dusk if the UTC calendar date has changed.
// ------------------------------------------------------------
void scheduler_ensureSchedule(time_t now);

// ------------------------------------------------------------
// Sleep
// ------------------------------------------------------------
// Check whether it is night and, if so, deep-sleep until next dawn.
// Must be called after NTP is valid.
void scheduler_trySleepIfNight(time_t now);

// Unconditionally deep-sleep until `target` (UTC epoch).
void scheduler_deepSleepUntil(time_t target);

// Return the next civil dawn after `now` (searches up to 4 days ahead).
time_t scheduler_nextCivilDawnAfter(time_t now);

// ------------------------------------------------------------
// Maintenance inhibit (OTA / update protection)
// ------------------------------------------------------------
// Set or clear the maintenance inhibit flag.  While active, both
// scheduler_trySleepIfNight() and scheduler_deepSleepUntil() are no-ops.
// Only OtaManager should call this.
void scheduler_setMaintenanceInhibit(bool active);
bool scheduler_maintenanceInhibit();

// ------------------------------------------------------------
// Preferences helpers (rolling wake / sleep lists)
// ------------------------------------------------------------
void scheduler_pushCsvEpochRolling(const char* key, time_t value);
void scheduler_setCsvEpochList(const char* key, time_t a, time_t b, time_t c);
void scheduler_refreshNextSleeps(time_t today_dusk, time_t tomorrow_dusk);
