

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#define I2C_NODE DT_NODELABEL(i2c1)
#define TSL2591_ADDR 0x39
#define TSL2591_COMMAND_BIT 0xA0
#define TSL2591_ID_ADDR 0x12

void main(void) {
    const struct device *const i2c_dev = DEVICE_DT_GET(I2C_NODE);

    /* Place the command in a variable so we can take its address */
    uint8_t reg_addr = TSL2591_COMMAND_BIT | TSL2591_ID_ADDR;
    uint8_t chip_id = 0;
    int ret;

    if (!device_is_ready(i2c_dev)) {
        printk("I2C1 not ready\n");
        return;
    }

    k_msleep(100);

    // loop throguh all possible I2C addresses to find the TSL2591
    for (uint8_t addr = 0x01; addr <= 0x99; addr++) {
        ret = i2c_write_read(i2c_dev, addr, &reg_addr, 1, &chip_id, 1);
        if (ret == 0) {
            printk("Found TSL2591 at address 0x%02x\n", addr);
        }
        // } else {
        //     printk("No device at address 0x%02x (err %d)\n", addr, ret);
        // }
    }

    /* Use the variable reg_addr instead of the macro */
    // ret = i2c_write_read(i2c_dev, TSL2591_ADDR, &reg_addr, 1, &chip_id, 1);

    // if (ret != 0) {
    //     printk("I2C Read Error: %d (NACK at 0x29)\n", ret);
    // } else {
    //     printk("SUCCESS! TSL2591 Chip ID: 0x%02x (Expected 0x50)\n", chip_id);
    // }
}
