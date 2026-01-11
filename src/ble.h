#ifndef BLE_H_
#define BLE_H_

#include <stdint.h>
#include <stdbool.h>

/* Function declarations */
int ble_init(void);
int ble_send_sensor_data(const void *data, uint16_t len);
bool ble_is_ready(void);

#endif
