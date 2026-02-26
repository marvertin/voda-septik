#pragma once

#include <stdio.h>
#include <string.h>

extern bool g_debug_enabled;

void debug_mqtt_publish(const char *topic, const char *text);

#define TOPIC_ROOT  "voda/septik"
#define DEBUG_BASE  TOPIC_ROOT "/debug/"
#define DEBUG_TOPIC(x) DEBUG_BASE x
#define DEBUG_FILE_BASENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define DEBUG_PUBLISH(rel_topic, fmt, ...)                              \
    do {                                                                 \
        if (g_debug_enabled) {                                           \
            char _dbg_buf[192];                                          \
            snprintf(_dbg_buf, sizeof(_dbg_buf),                         \
                     "[%s:%d:%s] " fmt,                                 \
                     DEBUG_FILE_BASENAME, __LINE__, __func__,            \
                     ##__VA_ARGS__);                                     \
            debug_mqtt_publish(DEBUG_TOPIC(rel_topic), _dbg_buf);        \
        }                                                                \
    } while (0)
