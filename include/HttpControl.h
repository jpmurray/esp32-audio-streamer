#pragma once
// HttpControl.h — /api/* route registration for the control-plane HTTP server.
//
// All mutating endpoints require header  X-ESP32MIC-CSRF: 1
// Provides:
//   GET  /api/status
//   GET  /api/audio_status
//   GET  /api/perf_status
//   GET  /api/logs
//   POST /api/set
//   POST /api/action/restart-audio
//   POST /api/action/time-sync
//   POST /api/action/reboot
//
// Legacy endpoints (GET /status, GET /uptime) remain in main.cpp unchanged.

#include <WebServer.h>

// Register all /api/* routes on `server`.
// Call once from setup(), after audioPipeline_init() and streamServer_init().
void httpControl_registerRoutes(WebServer& server);
