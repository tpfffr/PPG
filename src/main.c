/*
 * Copyright (c) 2017, NXP
 * Copyright (c) 2025, CATIE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include "ble.h"
#include <zephyr/random/random.h>


struct __packed sensor_packet {
	uint64_t timestamp;
	int32_t  ecg;
	int32_t  resp;
};

static void print_sample_fetch(const struct device *dev, int32_t *green_out, int32_t *red_out)
{
    static struct sensor_value green;
    static struct sensor_value red;

    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_GREEN, &green);
    sensor_channel_get(dev, SENSOR_CHAN_RED, &red);

    printk("GREEN = %d\n", (int32_t)green.val1);
    printk("RED = %d\n", (int32_t)red.val1);

    *green_out = green.val1;
    *red_out = red.val1;
}

int main(void)
{
    ble_init();

    const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(max30101));

    struct sensor_value attr;
    attr.val1 = 0x7F;
    sensor_attr_set(dev, SENSOR_CHAN_RED, SENSOR_ATTR_FULL_SCALE, &attr);

    if (!device_is_ready(dev)) {
        printk("Device %s is not ready\n", dev->name);
    }

    uint32_t sleep_ms = 20;
    int32_t green_value, red_value;

    while (1) {
        print_sample_fetch(dev, &green_value, &red_value);

        if (ble_is_ready()) {
            struct sensor_packet pkt;

            pkt.timestamp = (uint64_t)k_uptime_get();
            pkt.ecg = green_value;
            pkt.resp = red_value;

            /* Send the 16-byte packet to the Android app */
            ble_send_sensor_data(&pkt, sizeof(pkt));
        }

        k_msleep(sleep_ms);
    }
    return 0;
}
