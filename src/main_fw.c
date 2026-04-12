#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>
#include <nrfx.h>
#include <string.h>
#include <errno.h>

#include "max32664_fw.h"

/* Board wiring */
#define MAX32664_RSTN_PORT 1
#define MAX32664_RSTN_PIN  13   /* P1.13 */
#define MAX32664_MFIO_PORT 0
#define MAX32664_MFIO_PIN  3    /* P0.03 */

/* MAX32664 node must exist in DT and contain bus + address */
static const struct i2c_dt_spec hub_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(max32664));

/* .msbl layout assumptions */
#define FW_HEADER_SIZE              0x4C
#define FW_NUM_PAGES_OFFSET         0x44
#define FW_IV_OFFSET                0x28
#define FW_IV_SIZE                  11
#define FW_AUTH_OFFSET              0x34
#define FW_AUTH_SIZE                16
#define FW_PAGE_SIZE                8192
#define FW_PAGE_TRAILER_SIZE        16
#define FW_PAGE_WRITE_SIZE          (FW_PAGE_SIZE + FW_PAGE_TRAILER_SIZE)

/* Delays */
#define CMD_DELAY_MS                10
#define ERASE_DELAY_MS              2200
#define PAGE_WRITE_DELAY_MS         900
#define APP_START_DELAY_MS          1700
#define BL_START_DELAY_MS           100

static int regulators_init(void)
{
    NRF_POWER->DCDCEN0 = 1;
    NRF_POWER->DCDCEN  = 0;
    NRF_SPIM3->ENABLE = 0;
    *(volatile uint32_t *)0x4002F004 = 1;
    return 0;
}
SYS_INIT(regulators_init, PRE_KERNEL_1, 0);

static inline uint32_t rstn_psel(void)
{
    return NRF_GPIO_PIN_MAP(MAX32664_RSTN_PORT, MAX32664_RSTN_PIN);
}

static inline uint32_t mfio_psel(void)
{
    return NRF_GPIO_PIN_MAP(MAX32664_MFIO_PORT, MAX32664_MFIO_PIN);
}

static void pins_init(void)
{
    nrf_gpio_cfg_output(rstn_psel());
    nrf_gpio_cfg_output(mfio_psel());

    nrf_gpio_pin_write(rstn_psel(), 1);
    nrf_gpio_pin_write(mfio_psel(), 1);
}

static void enter_bootloader_reset_only(void)
{
    /* RSTN low, MFIO low, then release reset */
    nrf_gpio_pin_write(rstn_psel(), 0);
    nrf_gpio_pin_write(mfio_psel(), 0);
    k_msleep(20);
    nrf_gpio_pin_write(rstn_psel(), 1);
    k_msleep(BL_START_DELAY_MS);
}

static void enter_app_reset_only(void)
{
    /* RSTN low, MFIO high, then release reset */
    nrf_gpio_pin_write(rstn_psel(), 0);
    nrf_gpio_pin_write(mfio_psel(), 1);
    k_msleep(20);
    nrf_gpio_pin_write(rstn_psel(), 1);
    k_msleep(APP_START_DELAY_MS);
}

static int bl_txrx(const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len)
{
    int err;

    i2c_recover_bus(hub_i2c.bus);
    k_msleep(2);

    err = i2c_write_dt(&hub_i2c, tx, tx_len);
    if (err) {
        printk("BL WRITE err=%d cmd=%02X %02X\n",
               err, tx[0], (tx_len > 1) ? tx[1] : 0);
        return err;
    }

    k_msleep(CMD_DELAY_MS);

    err = i2c_read_dt(&hub_i2c, rx, rx_len);
    if (err) {
        printk("BL READ err=%d cmd=%02X %02X\n",
               err, tx[0], (tx_len > 1) ? tx[1] : 0);
        return err;
    }

    k_msleep(CMD_DELAY_MS);

    printk("BL STATUS: cmd=%02X %02X status=%02X\n",
           tx[0], (tx_len > 1) ? tx[1] : 0, rx[0]);

    if (rx[0] != 0x00) {
        return rx[0];
    }

    return 0;
}

static int app_read(uint8_t family, uint8_t index, uint8_t *rx, size_t rx_len)
{
    uint8_t tx[2] = { family, index };
    int err;

    /* In app mode, MFIO is the wake pin */
    nrf_gpio_pin_write(mfio_psel(), 0);
    k_usleep(500);

    err = i2c_write_dt(&hub_i2c, tx, sizeof(tx));
    if (err) {
        nrf_gpio_pin_write(mfio_psel(), 1);
        printk("APP WRITE err=%d cmd=%02X %02X\n", err, family, index);
        return err;
    }

    k_msleep(CMD_DELAY_MS);

    err = i2c_read_dt(&hub_i2c, rx, rx_len);
    nrf_gpio_pin_write(mfio_psel(), 1);
    if (err) {
        printk("APP READ err=%d cmd=%02X %02X\n", err, family, index);
        return err;
    }

    if (rx[0] != 0x00) {
        printk("APP STATUS err=%02X cmd=%02X %02X\n", rx[0], family, index);
        return -EINVAL;
    }

    return 0;
}

static int read_bootloader_info(void)
{
    uint8_t tx[3];
    uint8_t rx[4];
    int err;

    tx[0] = 0x02;
    tx[1] = 0x00;
    err = bl_txrx(tx, 2, rx, 2);
    if (err) {
        printk("Mode read failed: %d\n", err);
        return err;
    }
    printk("Current mode = 0x%02X\n", rx[1]);

    tx[0] = 0x81;
    tx[1] = 0x00;
    err = bl_txrx(tx, 2, rx, 4);
    if (err) {
        printk("Bootloader version read failed: %d\n", err);
        return err;
    }
    printk("Bootloader version: %u.%u.%u\n", rx[1], rx[2], rx[3]);

    tx[0] = 0x81;
    tx[1] = 0x01;
    err = bl_txrx(tx, 2, rx, 3);
    if (err) {
        printk("Bootloader page size read failed: %d\n", err);
        return err;
    }
    printk("Bootloader page size: %u\n", ((uint16_t)rx[1] << 8) | rx[2]);

    return 0;
}

static int bootloader_enter(void)
{
    uint8_t tx[3] = {0x01, 0x00, 0x08};
    uint8_t rx[4] = {0};

    enter_bootloader_reset_only();
    return bl_txrx(tx, 3, rx, 1);
}

static int erase_app(void)
{
    uint8_t tx[2] = {0x80, 0x03};
    uint8_t rx = 0;
    int err;

    i2c_recover_bus(hub_i2c.bus);
    k_msleep(2);

    err = i2c_write_dt(&hub_i2c, tx, sizeof(tx));
    if (err) {
        printk("ERASE WRITE err=%d\n", err);
        return err;
    }

    k_msleep(ERASE_DELAY_MS);

    err = i2c_read_dt(&hub_i2c, &rx, 1);
    if (err) {
        printk("ERASE READ err=%d\n", err);
        return err;
    }

    printk("ERASE STATUS=%02X\n", rx);
    if (rx != 0x00) {
        return rx;
    }

    return 0;
}

static int write_page(const uint8_t *fw, uint32_t offset, uint8_t page_idx, uint8_t num_pages)
{
    uint8_t *tx;
    uint8_t rx = 0;
    int err = 0;

    /* The bootloader expects exactly 8208 bytes (8192 data + 16 CRC) per page [cite: 924] */
    tx = k_malloc(FW_PAGE_WRITE_SIZE + 2);
    if (!tx) {
        return -ENOMEM;
    }

    tx[0] = 0x80; /* Family ID: Bootloader [cite: 875] */
    tx[1] = 0x04; /* Index: Write Page [cite: 875] */
    memcpy(&tx[2], &fw[offset], FW_PAGE_WRITE_SIZE);

    /* The final page verification can take significantly longer [cite: 919] */
    int max_attempts = (page_idx == num_pages) ? 20 : 10;

    for (int attempt = 1; attempt <= max_attempts; attempt++) {
        i2c_recover_bus(hub_i2c.bus);
        k_msleep(10);

        /* Write command + data [cite: 832] */
        err = i2c_write_dt(&hub_i2c, tx, FW_PAGE_WRITE_SIZE + 2);
        if (err) {
            printk("PAGE %u/%u WRITE I2C ERR=%d\n", page_idx, num_pages, err);
            k_msleep(100);
            continue;
        }

        /* Allow internal flash processing time [cite: 225, 830] */
        k_msleep(PAGE_WRITE_DELAY_MS);

        /* Read status byte [cite: 229, 834] */
        err = i2c_read_dt(&hub_i2c, &rx, 1);
        if (err) {
            k_msleep(100);
            continue;
        }

        /* Status 0x05: Device is busy, wait and retry  */
        if (rx == 0x05) {
            printk("PAGE %u/%u Hub Busy (0x05), waiting... (Attempt %d)\n", page_idx, num_pages, attempt);
            k_msleep(1000);
            continue;
        }

        if (rx == 0x00) {
            k_free(tx);
            return 0;
        }

        /* Any other status is a failure  */
        printk("PAGE %u/%u FAILED with status 0x%02X\n", page_idx, num_pages, rx);
        k_free(tx);
        return rx;
    }

    k_free(tx);
    return -ETIMEDOUT;
}

static int leave_bootloader_and_verify(void)
{
    uint8_t tx[3] = {0x01, 0x00, 0x00};   /* exit bootloader */
    uint8_t rx[4] = {0};
    int err;

    /* Tell the bootloader to switch to application mode */
    err = bl_txrx(tx, 3, rx, 1);
    if (err) {
        printk("EXIT BOOTLOADER failed: %d\n", err);
        return err;
    }

    /* Give it a moment, then do a clean app-mode reset */
    k_msleep(20);
    enter_app_reset_only();

    err = app_read(0x02, 0x00, rx, 2);
    if (err) {
        printk("App mode read failed: %d\n", err);
        return err;
    }

    printk("APP MODE = 0x%02X\n", rx[1]);
    if (rx[1] != 0x00) {
        return -EINVAL;
    }

    err = app_read(0xFF, 0x03, rx, 4);
    if (err) {
        printk("FW version read failed: %d\n", err);
        return err;
    }

    printk("APP MODE still 0x08, trying explicit exit again...\n");
    err = bl_txrx(tx, 3, rx, 1);
    if (!err) {
        k_msleep(20);
        enter_app_reset_only();
        err = app_read(0x02, 0x00, rx, 2);
        printk("APP MODE retry = 0x%02X err=%d\n", rx[1], err);
    }

    printk("FW VERSION = %u.%u.%u\n", rx[1], rx[2], rx[3]);
    return 0;
}

static int flash_from_scratch(void)
{
    uint8_t tx[18] = {0};
    uint8_t rx[4] = {0};
    int err;
    const uint8_t num_pages = max32664_fw[FW_NUM_PAGES_OFFSET];
    uint32_t expected_len = FW_HEADER_SIZE + ((uint32_t)num_pages * FW_PAGE_WRITE_SIZE);

    printk("FW LAST 16 BYTES:");
    for (uint32_t i = max32664_fw_len - 16; i < max32664_fw_len; i++) {
        printk(" %02X", max32664_fw[i]);
    }
    printk("\n");

    printk("FW LEN actual=%u expected=%u delta=%d\n",
           max32664_fw_len, expected_len, (int)max32664_fw_len - (int)expected_len);
    printk("FW MAGIC=%02X %02X %02X %02X pages=%u\n",
           max32664_fw[0], max32664_fw[1], max32664_fw[2], max32664_fw[3], num_pages);

    err = bootloader_enter();
    if (err) {
        printk("Bootloader enter failed: %d\n", err);
        return err;
    }

    err = read_bootloader_info();
    if (err) {
        return err;
    }

    tx[0] = 0x80;
    tx[1] = 0x02;
    tx[2] = 0x00;
    tx[3] = num_pages;
    err = bl_txrx(tx, 4, rx, 1);
    printk("SET_NUM_PAGES -> %d\n", err);
    if (err) {
        return err;
    }

    tx[0] = 0x80;
    tx[1] = 0x00;
    memcpy(&tx[2], &max32664_fw[FW_IV_OFFSET], FW_IV_SIZE);
    err = bl_txrx(tx, 2 + FW_IV_SIZE, rx, 1);
    printk("WRITE_IV -> %d\n", err);
    if (err) {
        return err;
    }

    tx[0] = 0x80;
    tx[1] = 0x01;
    memcpy(&tx[2], &max32664_fw[FW_AUTH_OFFSET], FW_AUTH_SIZE);
    err = bl_txrx(tx, 2 + FW_AUTH_SIZE, rx, 1);
    printk("WRITE_AUTH -> %d\n", err);
    if (err) {
        return err;
    }

    err = erase_app();
    printk("ERASE_APP -> %d\n", err);
    if (err) {
        return err;
    }

    uint32_t page_offset = FW_HEADER_SIZE;
    for (uint8_t i = 0; i < num_pages; i++) {
        uint32_t page_end = page_offset + FW_PAGE_WRITE_SIZE - 1;

        printk("WRITE PAGE %u/%u offset=0x%08X end=0x%08X\n",
            i + 1, num_pages, page_offset, page_end);

        err = write_page(max32664_fw, page_offset, i + 1, num_pages);
        printk("WRITE PAGE %u RESULT=%d\n", i + 1, err);
        if (err) {
            return err;
        }

        page_offset += FW_PAGE_WRITE_SIZE;
    }

    return leave_bootloader_and_verify();
}


int main(void)
{
    int err;

    // uint32_t num_pages = max32664_fw[FW_NUM_PAGES_OFFSET];
    // uint32_t expected_len = FW_HEADER_SIZE + (num_pages * FW_PAGE_WRITE_SIZE);
    // uint32_t extra = max32664_fw_len - expected_len;

    // printk("FW LEN actual=%u expected=%u extra=%u\n",
    //     max32664_fw_len, expected_len, extra);

    // if (extra == 4) {
    //     printk("FW tail bytes: %02X %02X %02X %02X\n",
    //         max32664_fw[expected_len + 0],
    //         max32664_fw[expected_len + 1],
    //         max32664_fw[expected_len + 2],
    //         max32664_fw[expected_len + 3]);
    // }


    printk("\n=== MAX32664C RAW RECOVERY ===\n");

    if (!device_is_ready(hub_i2c.bus)) {
        printk("I2C bus not ready\n");
        while (1) {
            k_msleep(1000);
        }
    }

    pins_init();

    err = flash_from_scratch();
    printk("FINAL RESULT = %d\n", err);

    while (1) {
        k_msleep(1000);
    }

    return 0;
}
