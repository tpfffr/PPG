#ifndef MAXM86161_LITE_H
#define MAXM86161_LITE_H

#include <zephyr/drivers/i2c.h>

#define MAXM86161_ADDR 0x62
#define MAXM86161_PART_ID 0x36

// Registers
#define REG_IRQ_STATUS1      0x00
#define REG_FIFO_DATA_COUNTER 0x07
#define REG_FIFO_DATA         0x08
#define REG_SYSTEM_CONTROL    0x0D
#define REG_PPG_CONFIG1       0x11
#define REG_PPG_CONFIG2       0x12
#define REG_LED_SEQ1          0x20
#define REG_LED_SEQ2          0x21
#define REG_PART_ID           0xFF

typedef struct {
    uint32_t led1;
    uint32_t led2;
    uint32_t led3;
    uint32_t ambient;
} maxm86161_sample_t;

int maxm86161_init(const struct i2c_dt_spec *spec);
int maxm86161_read_fifo(const struct i2c_dt_spec *spec, maxm86161_sample_t *samples, int max_count);
int maxm86161_start(const struct i2c_dt_spec *spec);
int maxm86161_stop(const struct i2c_dt_spec *spec);

#endif
