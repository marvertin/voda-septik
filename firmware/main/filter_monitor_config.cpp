#include "filter_monitor.h"
#include "config_store.h"
#include "app_error_check.h"
static const config_item_t ITEM_0 = {
    .key = "flt_min_flow", .label = "Minimalni prutok filtru [l/min]", .description = "Vyhodnoceni filtru v desetisekundovych oknech.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = 25.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = 25.0f, .max_float = 200.0f,
};
static const config_item_t ITEM_1 = {
    .key = "flt_dp_offset", .label = "Posun rozdilu cidel [bar]", .description = "Vyhodnoceni filtru v desetisekundovych oknech.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = -2.0f, .max_float = 2.0f,
};
static const config_item_t ITEM_2 = {
    .key = "flt_dp_std", .label = "Maximalni odchylka dP [bar]", .description = "Vyhodnoceni filtru v desetisekundovych oknech.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = 0.05f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = 0.001f, .max_float = 2.0f,
};
static const config_item_t ITEM_3 = {
    .key = "flt_flow_rng", .label = "Maximalni relativni rozsah prutoku", .description = "Vyhodnoceni filtru v desetisekundovych oknech.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = 0.2f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = 0.01f, .max_float = 1.0f,
};
static const config_item_t ITEM_4 = {
    .key = "flt_p_rng", .label = "Maximalni rozsah tlaku [bar]", .description = "Vyhodnoceni filtru v desetisekundovych oknech.",
    .type = CONFIG_VALUE_FLOAT, .default_string = nullptr, .default_int = 0, .default_float = 0.3f, .default_bool = false,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = 0.01f, .max_float = 5.0f,
};
void filter_monitor_register_config_items() {
    APP_ERROR_CHECK("E910", config_store_register_item(&ITEM_0));
    APP_ERROR_CHECK("E911", config_store_register_item(&ITEM_1));
    APP_ERROR_CHECK("E912", config_store_register_item(&ITEM_2));
    APP_ERROR_CHECK("E913", config_store_register_item(&ITEM_3));
    APP_ERROR_CHECK("E914", config_store_register_item(&ITEM_4));
}
FilterConfig filter_monitor_load_config() {
    return {config_store_get_float_item(&ITEM_0), config_store_get_float_item(&ITEM_1), config_store_get_float_item(&ITEM_2), config_store_get_float_item(&ITEM_3), config_store_get_float_item(&ITEM_4)};
}
