#include "network_mqtt_config.h"

#include <cstring>

#define MQTT_URI_MAX_LEN 128
#define MQTT_USER_MAX_LEN 64
#define MQTT_PASS_MAX_LEN 128

static char s_mqtt_uri[MQTT_URI_MAX_LEN] = {0};
static char s_mqtt_username[MQTT_USER_MAX_LEN] = {0};
static char s_mqtt_password[MQTT_PASS_MAX_LEN] = {0};

static bool is_valid_mqtt_uri(const char *broker_uri)
{
    if (broker_uri == NULL || broker_uri[0] == '\0') {
        return false;
    }

    const bool scheme_ok = (strncmp(broker_uri, "mqtt://", 7) == 0)
                        || (strncmp(broker_uri, "mqtts://", 8) == 0);
    if (!scheme_ok) {
        return false;
    }

    const char *host = strstr(broker_uri, "://");
    if (host == NULL) {
        return false;
    }
    host += 3;

    if (host[0] == '\0' || host[0] == ':' || host[0] == '/') {
        return false;
    }

    return true;
}

bool network_mqtt_config_prepare(const char *broker_uri, const char *username, const char *password)
{
    if (!is_valid_mqtt_uri(broker_uri)) {
        return false;
    }

    strncpy(s_mqtt_uri, broker_uri, sizeof(s_mqtt_uri) - 1);
    s_mqtt_uri[sizeof(s_mqtt_uri) - 1] = '\0';

    s_mqtt_username[0] = '\0';
    if (username != NULL) {
        strncpy(s_mqtt_username, username, sizeof(s_mqtt_username) - 1);
        s_mqtt_username[sizeof(s_mqtt_username) - 1] = '\0';
    }

    s_mqtt_password[0] = '\0';
    if (password != NULL) {
        strncpy(s_mqtt_password, password, sizeof(s_mqtt_password) - 1);
        s_mqtt_password[sizeof(s_mqtt_password) - 1] = '\0';
    }

    return true;
}

const char *network_mqtt_config_uri(void)
{
    return s_mqtt_uri;
}

const char *network_mqtt_config_username_or_null(void)
{
    return (s_mqtt_username[0] != '\0') ? s_mqtt_username : NULL;
}

const char *network_mqtt_config_password_or_null(void)
{
    return (s_mqtt_password[0] != '\0') ? s_mqtt_password : NULL;
}
