#pragma once

#include "network_event.h"
#include "sensor_events.h"

#ifdef __cplusplus
extern "C" {
#endif

void status_display_init(void);
void status_display_set_network_state(const network_event_t *event);
void status_display_set_sensor_fault(sensor_event_type_t sensor_type, bool is_fault);
void status_display_notify_mqtt_activity(void);
void status_display_set_flow_rate(float flow_l_min);
//void status_display_set_text(const char *text);
void status_display_ap_mode(void);
void set_segments(const uint8_t segments, uint8_t position, bool on); 


#ifdef __cplusplus
}
#endif
