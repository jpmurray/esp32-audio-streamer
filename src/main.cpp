#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YOUR_PASSWORD"
#endif

static WebServer server(80);

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>ESP32 Audio Streamer</title>
    <style>
      body { font-family: -apple-system, system-ui, Roboto, Arial, sans-serif; margin: 2rem; }
      .card { max-width: 680px; padding: 1rem 1.25rem; border: 1px solid #ddd; border-radius: 8px; }
      code { background: #f6f8fa; padding: 0.2rem 0.4rem; border-radius: 4px; }
      .warn { color: #b35600; }
    </style>
  </head>
  <body>
    <h1>ESP32 Audio Streamer</h1>
    <div class="card">
      <p>Status: <strong>Server is running</strong></p>
      <p>
        Once audio streaming is enabled, this player will work:
      </p>
      <audio id="player" controls preload="none" src="/stream"></audio>
      <p class="warn">Note: /stream not implemented yet (next step).</p>
      <p>
        Raw PCM stream content type will be <code>audio/L16</code>, mono 16 kHz.
      </p>
    </div>
  </body>
</html>
)HTML";

static void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

static void handleStreamNotReady() {
  server.send(503, "text/plain", "Audio streaming not implemented yet. Check back after next update.");
}

static void connectWiFiBlocking() {
  WiFi.mode(WIFI_STA);

  if (String(WIFI_SSID) == "YOUR_SSID" || String(WIFI_PASS) == "YOUR_PASSWORD") {
    Serial.println();
    Serial.println("[WARN] WIFI_SSID/WIFI_PASS are placeholders.");
    Serial.println("       Set them in platformio.ini build_flags, then rebuild/flash.");
  }

  Serial.printf("Connecting to Wi‑Fi SSID: %s\n", WIFI_SSID);
  // Reduce TX power a bit to avoid brownout on weak USB ports
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Block until connected
  uint32_t dot = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print('.');
    if ((++dot % 40) == 0) Serial.println();
  }
  Serial.println();
  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());
}

static void disableBrownout() {
#ifdef RTC_CNTL_BROWN_OUT_REG
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Booting ESP32 Audio Streamer (Step 1: Wi‑Fi + HTTP)");

  // Mitigate repeated resets due to brownout on marginal USB power
  disableBrownout();

  connectWiFiBlocking();

  // HTTP routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/stream", HTTP_GET, handleStreamNotReady);
  server.begin();
  Serial.println("HTTP server started on :80");
}

void loop() {
  server.handleClient();
  delay(2);
}