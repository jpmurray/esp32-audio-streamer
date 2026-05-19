// NetworkManager.cpp — lightweight Wi-Fi onboarding/management foundation.

#include "NetworkManager.h"
#include "RuntimeSettings.h"
#include "LogBuffer.h"

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include "esp_wifi.h"

// ------------------------------------------------------------
// Logging: use centralized macros from LogBuffer.h.
// ------------------------------------------------------------

// ------------------------------------------------------------
// Compile-time defaults
// ------------------------------------------------------------
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YOUR_PASSWORD"
#endif
#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 20000
#endif
#ifndef WIFI_RECONNECT_INTERVAL_MS
#define WIFI_RECONNECT_INTERVAL_MS 30000
#endif
#ifndef WIFI_SETUP_AP_SSID
#define WIFI_SETUP_AP_SSID "ESP32-Audio-Setup"
#endif
#ifndef WIFI_SETUP_AP_UNIQUE_SUFFIX
#define WIFI_SETUP_AP_UNIQUE_SUFFIX 1
#endif
#ifndef WIFI_SETUP_AP_PASS
#define WIFI_SETUP_AP_PASS ""
#endif
#ifndef WIFI_SETUP_AP_CHANNEL
#define WIFI_SETUP_AP_CHANNEL 6
#endif
#ifndef WIFI_SETUP_AP_MAX_CLIENTS
#define WIFI_SETUP_AP_MAX_CLIENTS 4
#endif
#ifndef WIFI_SETUP_AP_SUCCESS_GRACE_MS
#define WIFI_SETUP_AP_SUCCESS_GRACE_MS 120000
#endif

// RSSI quality classification thresholds.
// RSSI <= WIFI_RSSI_UNSTABLE_DBM => unstable (streaming likely impaired)
// RSSI <= WIFI_RSSI_WARN_DBM     => weak (streaming may be affected)
// RSSI  > WIFI_RSSI_WARN_DBM     => good
#ifndef WIFI_RSSI_WARN_DBM
#define WIFI_RSSI_WARN_DBM     (-70)
#endif
#ifndef WIFI_RSSI_UNSTABLE_DBM
#define WIFI_RSSI_UNSTABLE_DBM (-75)
#endif

// Minimum interval between repeated weak/unstable RSSI log warnings (ms).
#ifndef WIFI_RSSI_WARN_LOG_INTERVAL_MS
#define WIFI_RSSI_WARN_LOG_INTERVAL_MS 60000
#endif

// ------------------------------------------------------------
// Preferences
// ------------------------------------------------------------
static const char* const WIFI_NS       = "wifi";
static const char* const KEY_WIFI_SSID = "ssid";
static const char* const KEY_WIFI_PASS = "pass";

// ------------------------------------------------------------
// Module state
// ------------------------------------------------------------
static String s_saved_ssid;
static String s_saved_pass;
static String s_ap_ssid;
static String s_last_error;

static bool     s_has_credentials = false;
static bool     s_sta_connecting = false;
static bool     s_setup_ap_active = false;
static bool     s_ap_stop_scheduled = false;
static bool     s_reconnect_pending = false;
static bool     s_disconnect_sta_pending = false;
static uint32_t s_connect_started_ms = 0;
static uint32_t s_reconnect_at_ms = 0;
static uint32_t s_next_auto_reconnect_ms = 0;
static uint32_t s_ap_stop_at_ms = 0;
static uint32_t s_disconnect_sta_at_ms = 0;

// RSSI observability state
static uint32_t s_rssi_warn_log_at_ms = 0;  // next allowed warn log timestamp

// ------------------------------------------------------------
// Internal helpers
// ------------------------------------------------------------
static void copyString(char* out, size_t out_sz, const String& value) {
    if (!out || out_sz == 0) return;
    strncpy(out, value.c_str(), out_sz - 1);
    out[out_sz - 1] = '\0';
}

static void copyCString(char* out, size_t out_sz, const char* value) {
    if (!out || out_sz == 0) return;
    if (!value) value = "";
    strncpy(out, value, out_sz - 1);
    out[out_sz - 1] = '\0';
}

static bool isControlChar(char c) {
    uint8_t b = (uint8_t)c;
    return b < 0x20 || b == 0x7f;
}

static bool validateSsid(const String& ssid, char* errmsg, size_t errmsg_sz) {
    if (ssid.length() == 0 || ssid.length() > 32) {
        snprintf(errmsg, errmsg_sz, "ssid must be 1..32 bytes");
        return false;
    }
    for (size_t i = 0; i < ssid.length(); ++i) {
        if (isControlChar(ssid[i])) {
            snprintf(errmsg, errmsg_sz, "ssid contains control characters");
            return false;
        }
    }
    return true;
}

static bool validatePassword(const String& password, char* errmsg, size_t errmsg_sz) {
    if (!(password.length() == 0 || (password.length() >= 8 && password.length() <= 64))) {
        snprintf(errmsg, errmsg_sz, "password must be empty or 8..64 bytes");
        return false;
    }
    for (size_t i = 0; i < password.length(); ++i) {
        if (isControlChar(password[i])) {
            snprintf(errmsg, errmsg_sz, "password contains control characters");
            return false;
        }
    }
    return true;
}

static bool isPlaceholderCredential(const String& ssid, const String& pass) {
    if (ssid.length() == 0) return true;
    if (ssid == "YOUR_SSID" || ssid == "Your SSID") return true;
    if (pass == "YOUR_PASSWORD" || pass == "Your Password") return true;
    return false;
}

static void loadCredentials() {
    Preferences prefs;
    prefs.begin(WIFI_NS, true /* read-only */);
    s_saved_ssid = prefs.getString(KEY_WIFI_SSID, "");
    s_saved_pass = prefs.getString(KEY_WIFI_PASS, "");
    prefs.end();
    s_has_credentials = s_saved_ssid.length() > 0;
}

static bool persistCredentials(const String& ssid, const String& pass, char* errmsg, size_t errmsg_sz) {
    Preferences prefs;
    if (!prefs.begin(WIFI_NS, false /* read-write */)) {
        snprintf(errmsg, errmsg_sz, "failed to open wifi preferences");
        return false;
    }
    prefs.putString(KEY_WIFI_SSID, ssid);
    prefs.putString(KEY_WIFI_PASS, pass);
    prefs.end();
    s_saved_ssid = ssid;
    s_saved_pass = pass;
    s_has_credentials = s_saved_ssid.length() > 0;
    return true;
}

static String makeSetupApSsid() {
    String ssid = String(WIFI_SETUP_AP_SSID);
#if WIFI_SETUP_AP_UNIQUE_SUFFIX
    uint64_t mac = ESP.getEfuseMac();
    char suffix[8];
    snprintf(suffix, sizeof(suffix), "-%06X", (unsigned)(mac & 0xFFFFFFu));
    ssid += suffix;
#endif
    if (ssid.length() > 39) ssid = ssid.substring(0, 39);
    return ssid;
}

// ------------------------------------------------------------
// RSSI quality helpers
// ------------------------------------------------------------
const char* networkManager_rssiQualityName(NetworkRssiQuality q) {
    switch (q) {
        case NETWORK_RSSI_GOOD:     return "good";
        case NETWORK_RSSI_WEAK:     return "weak";
        case NETWORK_RSSI_UNSTABLE: return "unstable";
        default:                    return "unknown";
    }
}

static NetworkRssiQuality classifyRssi(bool connected, int rssi) {
    if (!connected) return NETWORK_RSSI_UNKNOWN;
    if (rssi <= (int)WIFI_RSSI_UNSTABLE_DBM) return NETWORK_RSSI_UNSTABLE;
    if (rssi <= (int)WIFI_RSSI_WARN_DBM)     return NETWORK_RSSI_WEAK;
    return NETWORK_RSSI_GOOD;
}

static void setLastError(const char* msg) {
    s_last_error = msg ? msg : "";
    if (s_last_error.length() > 95) s_last_error = s_last_error.substring(0, 95);
}

static bool startSetupAp() {
    // If AP is already up (for example during post-onboarding grace), make
    // this an indefinite setup AP request and cancel any pending auto-stop.
    if (s_setup_ap_active) {
        s_ap_stop_scheduled = false;
        return true;
    }

    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    runtimeSettings_applyWifiTxPower();

    IPAddress ap_ip(192, 168, 4, 1);
    IPAddress ap_gw(192, 168, 4, 1);
    IPAddress ap_mask(255, 255, 255, 0);
    WiFi.softAPConfig(ap_ip, ap_gw, ap_mask);

    s_ap_ssid = makeSetupApSsid();
    String ap_pass = String(WIFI_SETUP_AP_PASS);
    const char* pass_arg = nullptr;
    if (ap_pass.length() >= 8) {
        pass_arg = ap_pass.c_str();
    } else if (ap_pass.length() > 0) {
        LOGW("NET", "setup AP password is shorter than 8 bytes; starting open AP\n");
    }

    bool ok = WiFi.softAP(s_ap_ssid.c_str(), pass_arg,
                          (int)WIFI_SETUP_AP_CHANNEL,
                          0,
                          (int)WIFI_SETUP_AP_MAX_CLIENTS);
    if (!ok) {
        setLastError("failed to start setup AP");
            LOGE("NET", "failed to start setup AP\n");
        return false;
    }

    s_setup_ap_active = true;
    s_ap_stop_scheduled = false;
    LOGI("NET", "setup AP active: SSID='%s' IP=%s ch=%d %s\n",
         s_ap_ssid.c_str(), WiFi.softAPIP().toString().c_str(),
         (int)WIFI_SETUP_AP_CHANNEL,
         pass_arg ? "WPA2" : "open");
    return true;
}

static void stopSetupApIfSafe() {
    if (!s_setup_ap_active || !networkManager_staConnected()) return;
    WiFi.softAPdisconnect(true);
    s_setup_ap_active = false;
    s_ap_stop_scheduled = false;
    s_ap_ssid = "";
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    runtimeSettings_applyWifiTxPower();
    LOGI("NET", "setup AP stopped after successful STA connection\n");
}

static void startStaConnect() {
    if (!s_has_credentials) return;

    WiFi.mode(s_setup_ap_active ? WIFI_AP_STA : WIFI_STA);
    WiFi.setSleep(false);
    runtimeSettings_applyWifiTxPower();

    LOGI("NET", "connecting STA to SSID='%s'\n", s_saved_ssid.c_str());
    WiFi.disconnect(false, false);
    delay(50);
    WiFi.begin(s_saved_ssid.c_str(), s_saved_pass.c_str());
    s_sta_connecting = true;
    s_connect_started_ms = millis();
}

static bool waitForStaConnected(uint32_t timeout_ms) {
    uint32_t start = millis();
    while ((millis() - start) < timeout_ms) {
        if (WiFi.status() == WL_CONNECTED) return true;
        delay(250);
        yield();
    }
    return WiFi.status() == WL_CONNECTED;
}

static bool connectStaBlocking(const String& ssid, const String& pass, uint32_t timeout_ms) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    runtimeSettings_applyWifiTxPower();
    LOGI("NET", "boot STA connect to SSID='%s' (timeout %lu ms)\n",
         ssid.c_str(), (unsigned long)timeout_ms);
    WiFi.begin(ssid.c_str(), pass.c_str());
    bool ok = waitForStaConnected(timeout_ms);
    if (ok) {
        s_sta_connecting = false;
        s_ap_stop_scheduled = false;
        setLastError("");
        LOGI("NET", "STA connected: IP=%s RSSI=%d\n",
             WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
    } else {
        setLastError("STA connect timed out");
        LOGW("NET", "STA connect timed out\n");
        WiFi.disconnect(false, false);
    }
    return ok;
}

static String jsonEscape(const char* s) {
    String out;
    if (!s) return out;
    out.reserve(strlen(s) + 8);
    for (const char* p = s; *p; ++p) {
        char c = *p;
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"':  out += "\\\""; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if ((uint8_t)c < 0x20) {
                    char esc[7];
                    snprintf(esc, sizeof(esc), "\\u%04x", (unsigned)c);
                    out += esc;
                } else {
                    out += c;
                }
                break;
        }
    }
    return out;
}

// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------
NetworkBootMode networkManager_begin() {
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_OFF);
    delay(100);

    loadCredentials();

    if (s_has_credentials) {
        if (connectStaBlocking(s_saved_ssid, s_saved_pass, (uint32_t)WIFI_CONNECT_TIMEOUT_MS)) {
            return NETWORK_BOOT_STA_CONNECTED;
        }
        startSetupAp();
        return NETWORK_BOOT_STA_FAILED_SETUP_AP;
    }

    String legacy_ssid = String(WIFI_SSID);
    String legacy_pass = String(WIFI_PASS);
    if (!isPlaceholderCredential(legacy_ssid, legacy_pass)) {
        LOGI("NET", "trying legacy compile-time Wi-Fi credentials\n");
        if (connectStaBlocking(legacy_ssid, legacy_pass, (uint32_t)WIFI_CONNECT_TIMEOUT_MS)) {
            char errmsg[96] = "";
            if (persistCredentials(legacy_ssid, legacy_pass, errmsg, sizeof(errmsg))) {
                LOGI("NET", "migrated legacy compile-time Wi-Fi SSID to NVS\n");
            } else {
                LOGW("NET", "could not persist legacy Wi-Fi credentials: %s\n", errmsg);
            }
            return NETWORK_BOOT_STA_CONNECTED;
        }
        startSetupAp();
        return NETWORK_BOOT_STA_FAILED_SETUP_AP;
    }

    LOGI("NET", "no saved Wi-Fi credentials; starting setup AP\n");
    startSetupAp();
    return NETWORK_BOOT_SETUP_AP;
}

void networkManager_loop() {
    uint32_t now = millis();

    if (s_disconnect_sta_pending && (int32_t)(now - s_disconnect_sta_at_ms) >= 0) {
        s_disconnect_sta_pending = false;
        LOGI("NET", "disconnecting STA after forget request\n");
        WiFi.disconnect(false, false);
    }

    if (s_reconnect_pending && (int32_t)(now - s_reconnect_at_ms) >= 0) {
        s_reconnect_pending = false;
        startStaConnect();
    }

    if (s_sta_connecting) {
        wl_status_t st = WiFi.status();
        if (st == WL_CONNECTED) {
            s_sta_connecting = false;
            s_next_auto_reconnect_ms = 0;
            setLastError("");
            LOGI("NET", "STA connected: SSID='%s' IP=%s RSSI=%d\n",
                 WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
            if (s_setup_ap_active) {
                s_ap_stop_scheduled = true;
                s_ap_stop_at_ms = now + (uint32_t)WIFI_SETUP_AP_SUCCESS_GRACE_MS;
                LOGI("NET", "setup AP will stop in %lu ms\n",
                     (unsigned long)WIFI_SETUP_AP_SUCCESS_GRACE_MS);
            }
        } else if ((now - s_connect_started_ms) > (uint32_t)WIFI_CONNECT_TIMEOUT_MS) {
            s_sta_connecting = false;
            s_next_auto_reconnect_ms = now + (uint32_t)WIFI_RECONNECT_INTERVAL_MS;
            setLastError("STA connect timed out");
            LOGW("NET", "STA connect timed out; setup AP remains available\n");
            WiFi.disconnect(false, false);
            startSetupAp();
        }
    }

    if (s_ap_stop_scheduled && (int32_t)(now - s_ap_stop_at_ms) >= 0) {
        stopSetupApIfSafe();
    }

    // Rate-limited RSSI quality warnings
    if (networkManager_staConnected()) {
        int rssi = (int)WiFi.RSSI();
        NetworkRssiQuality quality = classifyRssi(true, rssi);
        if (quality == NETWORK_RSSI_WEAK || quality == NETWORK_RSSI_UNSTABLE) {
            if (s_rssi_warn_log_at_ms == 0 || (int32_t)(now - s_rssi_warn_log_at_ms) >= 0) {
                s_rssi_warn_log_at_ms = now + (uint32_t)WIFI_RSSI_WARN_LOG_INTERVAL_MS;
                LOGW("NET", "RSSI=%d dBm (%s); streaming may be affected\n",
                     rssi, networkManager_rssiQualityName(quality));
            }
        } else {
            // Reset warning timer when signal improves so next degradation logs promptly.
            s_rssi_warn_log_at_ms = 0;
        }
    }

    if (!s_sta_connecting && s_has_credentials && WiFi.status() != WL_CONNECTED) {
        if (s_next_auto_reconnect_ms == 0) {
            s_next_auto_reconnect_ms = now + (uint32_t)WIFI_RECONNECT_INTERVAL_MS;
        } else if ((int32_t)(now - s_next_auto_reconnect_ms) >= 0) {
            LOGW("NET", "STA disconnected; retrying saved credentials\n");
            startStaConnect();
        }
    }
}

bool networkManager_staConnected() {
    return WiFi.status() == WL_CONNECTED;
}

bool networkManager_setupApActive() {
    return s_setup_ap_active;
}

bool networkManager_hasCredentials() {
    return s_has_credentials;
}

void networkManager_getStatus(NetworkStatusSnapshot* out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->has_credentials = s_has_credentials;
    out->sta_connected = networkManager_staConnected();
    out->sta_connecting = s_sta_connecting;
    out->setup_ap_active = s_setup_ap_active;
    out->ap_stop_scheduled = s_ap_stop_scheduled;
    out->wifi_status_code = (int)WiFi.status();
    out->rssi_dbm = out->sta_connected ? (int)WiFi.RSSI() : 0;
    out->rssi_quality = classifyRssi(out->sta_connected, out->rssi_dbm);
    out->rssi_streaming_warning = (out->rssi_quality == NETWORK_RSSI_WEAK ||
                                   out->rssi_quality == NETWORK_RSSI_UNSTABLE);
    out->rssi_warn_dbm     = (int)WIFI_RSSI_WARN_DBM;
    out->rssi_unstable_dbm = (int)WIFI_RSSI_UNSTABLE_DBM;

    copyString(out->saved_ssid, sizeof(out->saved_ssid), s_saved_ssid);
    copyString(out->connected_ssid, sizeof(out->connected_ssid),
               out->sta_connected ? WiFi.SSID() : String(""));
    copyString(out->sta_ip, sizeof(out->sta_ip),
               out->sta_connected ? WiFi.localIP().toString() : String(""));
    copyString(out->ap_ip, sizeof(out->ap_ip),
               s_setup_ap_active ? WiFi.softAPIP().toString() : String(""));
    copyString(out->ap_ssid, sizeof(out->ap_ssid),
               s_setup_ap_active ? s_ap_ssid : String(""));
    copyString(out->sta_mac, sizeof(out->sta_mac), WiFi.macAddress());
    copyString(out->ap_mac, sizeof(out->ap_mac), WiFi.softAPmacAddress());
    copyString(out->last_error, sizeof(out->last_error), s_last_error);
}

bool networkManager_saveCredentials(const String& ssid,
                                    const String& password,
                                    char* errmsg,
                                    size_t errmsg_sz) {
    if (!validateSsid(ssid, errmsg, errmsg_sz)) return false;
    if (!validatePassword(password, errmsg, errmsg_sz)) return false;
    if (!persistCredentials(ssid, password, errmsg, errmsg_sz)) return false;
    setLastError("");
    LOGI("NET", "saved Wi-Fi credentials for SSID='%s'\n", ssid.c_str());
    return true;
}

bool networkManager_forgetCredentials(char* errmsg, size_t errmsg_sz) {
    Preferences prefs;
    if (!prefs.begin(WIFI_NS, false /* read-write */)) {
        snprintf(errmsg, errmsg_sz, "failed to open wifi preferences");
        return false;
    }
    prefs.remove(KEY_WIFI_SSID);
    prefs.remove(KEY_WIFI_PASS);
    prefs.end();
    s_saved_ssid = "";
    s_saved_pass = "";
    s_has_credentials = false;
    s_sta_connecting = false;
    s_reconnect_pending = false;
    s_next_auto_reconnect_ms = 0;
    s_ap_stop_scheduled = false;
    setLastError("");
    LOGI("NET", "forgot saved Wi-Fi credentials\n");
    return true;
}

bool networkManager_requestReconnect(char* errmsg, size_t errmsg_sz) {
    if (!s_has_credentials) {
        snprintf(errmsg, errmsg_sz, "no saved Wi-Fi credentials");
        return false;
    }
    s_reconnect_pending = true;
    s_reconnect_at_ms = millis() + 500;
    LOGI("NET", "STA reconnect scheduled\n");
    return true;
}

bool networkManager_requestForgetAndStartAp(bool disconnectSta,
                                            char* errmsg,
                                            size_t errmsg_sz) {
    if (!networkManager_forgetCredentials(errmsg, errmsg_sz)) return false;
    if (!startSetupAp()) {
        snprintf(errmsg, errmsg_sz, "failed to start setup AP");
        return false;
    }
    if (disconnectSta) {
        s_disconnect_sta_pending = true;
        s_disconnect_sta_at_ms = millis() + 750;
    }
    return true;
}

const char* networkManager_primaryIpString(char* out, size_t out_sz) {
    if (!out || out_sz == 0) return "";
    out[0] = '\0';
    if (networkManager_staConnected()) {
        copyString(out, out_sz, WiFi.localIP().toString());
    } else if (s_setup_ap_active) {
        copyString(out, out_sz, WiFi.softAPIP().toString());
    }
    return out;
}

const char* networkManager_streamHostIpString(char* out, size_t out_sz) {
    if (!out || out_sz == 0) return "";
    out[0] = '\0';
    if (networkManager_staConnected()) {
        copyString(out, out_sz, WiFi.localIP().toString());
    }
    return out;
}

int networkManager_writeStatusJson(char* out, size_t out_sz) {
    if (!out || out_sz == 0) return -1;

    NetworkStatusSnapshot s;
    networkManager_getStatus(&s);

    String json;
    json.reserve(900);
    json += "{\"has_credentials\":"; json += s.has_credentials ? "true" : "false";
    json += ",\"sta_connected\":"; json += s.sta_connected ? "true" : "false";
    json += ",\"sta_connecting\":"; json += s.sta_connecting ? "true" : "false";
    json += ",\"setup_ap_active\":"; json += s.setup_ap_active ? "true" : "false";
    json += ",\"ap_stop_scheduled\":"; json += s.ap_stop_scheduled ? "true" : "false";
    json += ",\"wifi_status_code\":"; json += String(s.wifi_status_code);
    json += ",\"saved_ssid\":\""; json += jsonEscape(s.saved_ssid); json += "\"";
    json += ",\"connected_ssid\":\""; json += jsonEscape(s.connected_ssid); json += "\"";
    json += ",\"sta_ip\":\""; json += jsonEscape(s.sta_ip); json += "\"";
    json += ",\"ap_ip\":\""; json += jsonEscape(s.ap_ip); json += "\"";
    json += ",\"ap_ssid\":\""; json += jsonEscape(s.ap_ssid); json += "\"";
    if (s.sta_connected) {
        json += ",\"rssi_dbm\":"; json += String(s.rssi_dbm);
    } else {
        json += ",\"rssi_dbm\":null";
    }
    json += ",\"rssi_quality\":\"";
    json += networkManager_rssiQualityName(s.rssi_quality);
    json += "\"";
    json += ",\"rssi_streaming_warning\":"; json += s.rssi_streaming_warning ? "true" : "false";
    json += ",\"rssi_warn_dbm\":"; json += String(s.rssi_warn_dbm);
    json += ",\"rssi_unstable_dbm\":"; json += String(s.rssi_unstable_dbm);
    json += ",\"sta_mac\":\""; json += jsonEscape(s.sta_mac); json += "\"";
    json += ",\"ap_mac\":\""; json += jsonEscape(s.ap_mac); json += "\"";
    json += ",\"last_error\":\""; json += jsonEscape(s.last_error); json += "\"";
    json += "}";

    if (json.length() + 1 > out_sz) {
        out[0] = '\0';
        return -1;
    }
    copyCString(out, out_sz, json.c_str());
    return (int)json.length();
}
