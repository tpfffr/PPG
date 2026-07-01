#ifndef EVENT_LOG_H_
#define EVENT_LOG_H_

#include <stddef.h>
#include <stdint.h>

#define EVENT_LOG_CAPACITY 128U

enum event_log_id {
    EVENT_LOG_BOOT = 1,
    EVENT_LOG_MAIN_ENTER,
    EVENT_LOG_SENSOR_READY,
    EVENT_LOG_BLE_INIT_START,
    EVENT_LOG_BLE_INIT_DONE,
    EVENT_LOG_ADV_STARTED,
    EVENT_LOG_BLE_CONNECTED,
    EVENT_LOG_BLE_DISCONNECTED,
    EVENT_LOG_SESSION_START,
    EVENT_LOG_SESSION_STOP,
    EVENT_LOG_LOW_BATT_CHECK,
    EVENT_LOG_LOW_BATT_SHUTDOWN,
    EVENT_LOG_SENSOR_FETCH_ERR,
};

struct event_log_entry {
    uint32_t seq;
    uint32_t uptime_ms;
    uint16_t id;
    int16_t arg;
};

void event_log_init(void);
void event_log_add(uint16_t id, int16_t arg);
size_t event_log_snapshot(struct event_log_entry *out, size_t max_entries);
void event_log_dump(void);
uint32_t event_log_boot_count(void);

#endif
