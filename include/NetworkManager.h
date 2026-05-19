#pragma once
// NetworkManager.h — NVS-backed Wi-Fi onboarding and management.
// Owns saved STA credentials, bounded reconnect attempts, setup AP fallback,
// and compact status snapshots for HTTP control endpoints.

#include <Arduino.h>
#include <stddef.h>

// ------------------------------------------------------------
// Boot result
// ------------------------------------------------------------
enum NetworkBootMode {
    NETWORK_BOOT_STA_CONNECTED,
    NETWORK_BOOT_SETUP_AP,
    NETWORK_BOOT_STA_FAILED_SETUP_AP
};

// ------------------------------------------------------------
// RSSI quality classification
// ------------------------------------------------------------
enum NetworkRssiQuality {
    NETWORK_RSSI_UNKNOWN  = 0,
    NETWORK_RSSI_GOOD     = 1,
    NETWORK_RSSI_WEAK     = 2,
    NETWORK_RSSI_UNSTABLE = 3
};

// ------------------------------------------------------------
// Status snapshot
// ------------------------------------------------------------
struct NetworkStatusSnapshot {
    bool has_credentials;
    bool sta_connected;
    bool sta_connecting;
    bool setup_ap_active;
    bool ap_stop_scheduled;
    int  wifi_status_code;
    int  rssi_dbm;
    NetworkRssiQuality rssi_quality;
    bool rssi_streaming_warning;
    int  rssi_warn_dbm;
    int  rssi_unstable_dbm;
    char saved_ssid[33];
    char connected_ssid[33];
    char sta_ip[16];
    char ap_ip[16];
    char ap_ssid[40];
    char sta_mac[18];
    char ap_mac[18];
    char last_error[96];
};

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------
NetworkBootMode networkManager_begin();
void networkManager_loop();

bool networkManager_staConnected();
bool networkManager_setupApActive();
bool networkManager_hasCredentials();

void networkManager_getStatus(NetworkStatusSnapshot* out);

// ------------------------------------------------------------
// Credentials and connection actions
// ------------------------------------------------------------
bool networkManager_saveCredentials(const String& ssid,
                                    const String& password,
                                    char* errmsg,
                                    size_t errmsg_sz);

bool networkManager_forgetCredentials(char* errmsg, size_t errmsg_sz);
bool networkManager_requestReconnect(char* errmsg, size_t errmsg_sz);
bool networkManager_requestForgetAndStartAp(bool disconnectSta,
                                            char* errmsg,
                                            size_t errmsg_sz);

// ------------------------------------------------------------
// Address/status helpers
// ------------------------------------------------------------
const char* networkManager_primaryIpString(char* out, size_t out_sz);
const char* networkManager_streamHostIpString(char* out, size_t out_sz);
int networkManager_writeStatusJson(char* out, size_t out_sz);
const char* networkManager_rssiQualityName(NetworkRssiQuality q);
