#include "maxm86161.h"

#include <zephyr/logging/log.h>

#define REG_LED1_PA 0x23
#define REG_LED2_PA 0x24
#define REG_LED3_PA 0x25
#define REG_LED_RANGE1 0x2A

LOG_MODULE_REGISTER(maxm_lite, LOG_LEVEL_INF);
int maxm86161_init(const struct i2c_dt_spec *spec) {
    uint8_t id;
    if (!device_is_ready(spec->bus)) return -ENODEV;

    // 1. Software Reset
    i2c_reg_write_byte_dt(spec, REG_SYSTEM_CONTROL, 0x01); //
    k_msleep(50);

    // 2. Verify Part ID
    i2c_reg_read_byte_dt(spec, REG_PART_ID, &id); //
    if (id != 0x36) return -EIO;

    // 3. Timing: 400 SPS, 4x Averaging = 100Hz effective output
    // SMP_AVE (bits 2:0) = 0x02 (4x) | PPG_SR (bits 7:3) = 0x05 (400Hz)
    i2c_reg_write_byte_dt(spec, REG_PPG_CONFIG2, (0x05 << 3) | 0x02); //

    // 4. ADC Config: 16,384nA range, 117.3us integration time
    // PPG1_ADC_RGE (bits 3:2) = 0x02 | PPG_TINT (bits 1:0) = 0x03
    i2c_reg_write_byte_dt(spec, 0x11, 0x0B); //

    // 5. LED Range: Set Green LED to 124mA full-scale
    i2c_reg_write_byte_dt(spec, REG_LED_RANGE1, 0x3F); //

    // 6. LED Current: Set Green LED to ~38mA (0x4F)
    i2c_reg_write_byte_dt(spec, REG_LED1_PA, 0x4F); //

    // 7. Picket Fence: Enable to cancel rapid ambient transients
    i2c_reg_write_byte_dt(spec, 0x16, 0x83); //

    // 8. Sequence: Slot 1 = Green LED, others disabled
    i2c_reg_write_byte_dt(spec, REG_LED_SEQ1, 0x01); //
    i2c_reg_write_byte_dt(spec, REG_LED_SEQ2, 0x00);

    return 0;
}

int maxm86161_start(const struct i2c_dt_spec *spec) {
    return i2c_reg_write_byte_dt(spec, REG_SYSTEM_CONTROL, 0x04); // LP Mode On
}

/* Add this to maxm86161.c */
int maxm86161_stop(const struct i2c_dt_spec *spec) {
    // 0x02 is the shutdown bit in REG_SYSTEM_CONTROL (0x0D)
    // We set it to 1 to enter shutdown mode
    return i2c_reg_write_byte_dt(spec, REG_SYSTEM_CONTROL, 0x02);
}
int maxm86161_read_fifo(const struct i2c_dt_spec *spec, maxm86161_sample_t *samples, int max_count) {
    uint8_t count;
    uint8_t raw[3]; // Only 3 bytes per Green LED sample

    i2c_reg_read_byte_dt(spec, REG_FIFO_DATA_COUNTER, &count);

    // With only 1 LED active, count is the actual number of samples
    int to_read = (count > max_count) ? max_count : count;

    for (int i = 0; i < to_read; i++) {
        // Read 3 bytes for LED1 (Green)
        if (i2c_burst_read_dt(spec, REG_FIFO_DATA, raw, 3) != 0) break;

        // Mask 19-bit data
        samples[i].led1 = ((raw[0] & 0x07) << 16) | (raw[1] << 8) | raw[2];
        samples[i].led2 = 0; // Not used
        samples[i].led3 = 0; // Not used
    }
    return to_read;
}
