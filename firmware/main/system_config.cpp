#include "system_config.h"

#include "config_store.h"
#include "app_error_check.h"

static const config_item_t INTERVAL_S_ITEM = {
    .key = "interval_s", .label = "Interval měření [s]", .description = "Perioda měření a publikace hodnot.",
    .type = CONFIG_VALUE_INT32, .default_string = nullptr, .default_int = 30, .default_float = 0.0f, .default_bool = false,
    .max_string_len = 0, .min_int = 5, .max_int = 3600, .min_float = 0.0f, .max_float = 0.0f,
};
static const config_item_t AUTO_MODE_ITEM = {
    .key = "auto_mode", .label = "Automatický režim", .description = "Zapíná automatické vyhodnocení závlahy.",
    .type = CONFIG_VALUE_BOOL, .default_string = nullptr, .default_int = 0, .default_float = 0.0f, .default_bool = true,
    .max_string_len = 0, .min_int = 0, .max_int = 0, .min_float = 0.0f, .max_float = 0.0f,
};

void system_config_register_config_items(void)
{
    APP_ERROR_CHECK("E695", config_store_register_item(&INTERVAL_S_ITEM));
    APP_ERROR_CHECK("E696", config_store_register_item(&AUTO_MODE_ITEM));
}
