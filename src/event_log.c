#include "event_log.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#define EVENT_LOG_MAGIC 0x45564C47u

struct event_log_state {
    uint32_t magic;
    uint32_t boot_count;
    uint32_t write_seq;
    struct event_log_entry entries[EVENT_LOG_CAPACITY];
};

static struct event_log_state event_log_state __noinit;

void event_log_init(void)
{
    if (event_log_state.magic != EVENT_LOG_MAGIC) {
        event_log_state.magic = EVENT_LOG_MAGIC;
        event_log_state.boot_count = 0;
        event_log_state.write_seq = 0;
        for (size_t i = 0; i < EVENT_LOG_CAPACITY; i++) {
            event_log_state.entries[i] = (struct event_log_entry){0};
        }
    }

    event_log_state.boot_count++;
}

void event_log_add(uint16_t id, int16_t arg)
{
    uint32_t seq = event_log_state.write_seq++;
    struct event_log_entry *entry = &event_log_state.entries[seq % EVENT_LOG_CAPACITY];

    entry->seq = seq;
    entry->uptime_ms = (uint32_t)k_uptime_get();
    entry->id = id;
    entry->arg = arg;
}

size_t event_log_snapshot(struct event_log_entry *out, size_t max_entries)
{
    uint32_t write_seq = event_log_state.write_seq;
    size_t total = MIN((size_t)write_seq, (size_t)EVENT_LOG_CAPACITY);
    size_t count = MIN(total, max_entries);
    size_t start = (write_seq > EVENT_LOG_CAPACITY) ? (write_seq % EVENT_LOG_CAPACITY) : 0U;

    for (size_t i = 0; i < count; i++) {
        out[i] = event_log_state.entries[(start + i) % EVENT_LOG_CAPACITY];
    }

    return count;
}

void event_log_dump(void)
{
    uint32_t write_seq = event_log_state.write_seq;
    size_t count = MIN((size_t)write_seq, (size_t)EVENT_LOG_CAPACITY);
    size_t start = (write_seq > EVENT_LOG_CAPACITY) ? (write_seq % EVENT_LOG_CAPACITY) : 0U;

    printk("Event log dump: boots=%u count=%u\n",
           (unsigned int)event_log_state.boot_count,
           (unsigned int)count);

    for (size_t i = 0; i < count; i++) {
        const struct event_log_entry *entry =
            &event_log_state.entries[(start + i) % EVENT_LOG_CAPACITY];

        printk("[%u] seq=%u t=%u id=%u arg=%d\n",
               (unsigned int)i,
               (unsigned int)entry->seq,
               (unsigned int)entry->uptime_ms,
               (unsigned int)entry->id,
               (int)entry->arg);
    }
}

uint32_t event_log_boot_count(void)
{
    return event_log_state.boot_count;
}
