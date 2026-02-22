#ifndef NETWORK_AP_MODE_H
#define NETWORK_AP_MODE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t network_ap_mode_start(const char *ap_ssid, const char *ap_password);

#ifdef __cplusplus
}
#endif

#endif // NETWORK_AP_MODE_H
