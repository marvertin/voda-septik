#pragma once

#include "network_event.h"

#ifdef __cplusplus
extern "C" {
#endif

void status_display_init(void);
void status_display_set_network_state(const network_event_t *event);
void status_display_set_text(const char *text);

#ifdef __cplusplus
}
#endif
