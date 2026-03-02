#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    CONFIG_VALUE_STRING = 0,
    CONFIG_VALUE_INT32,
    CONFIG_VALUE_FLOAT,
    CONFIG_VALUE_BOOL,
} config_value_type_t;

typedef struct {
    const char *key;
    const char *label;
    const char *description;
    config_value_type_t type;

    const char *default_string;
    int32_t default_int;
    float default_float;
    bool default_bool;

    size_t max_string_len;
    int32_t min_int;
    int32_t max_int;
    float min_float;
    float max_float;
} config_item_t;

typedef struct {
    const config_item_t *items;
    size_t item_count;
} config_group_t;
