/*
 * I2C firmware loader for the MAX32664C biometric sensor hub.
 *
 * Copyright (c) 2025, Daniel Kampert
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "max32664c.h"

#define MAX32664C_FW_PAGE_SIZE         8192
#define MAX32664C_FW_UPDATE_CRC_SIZE   16
#define MAX32664C_FW_UPDATE_WRITE_SIZE (MAX32664C_FW_PAGE_SIZE + MAX32664C_FW_UPDATE_CRC_SIZE)
#define MAX32664C_DEFAULT_CMD_DELAY_MS 10
#define MAX32664C_PAGE_WRITE_DELAY_MS  680

static uint8_t max32664c_fw_init_vector[11];
static uint8_t max32664c_fw_auth_vector[16];

LOG_MODULE_REGISTER(max32664_loader, CONFIG_SENSOR_LOG_LEVEL);

/** @brief          Read / write bootloader data from / to the sensor hub.
 *  @param dev      Pointer to device
 *  @param tx_buf   Pointer to transmit buffer
 *  @param tx_len   Length of transmit buffer
 *  @param rx_buf   Pointer to receive buffer
 *                  NOTE: The buffer must be large enough to store the response and the status byte!
 *  @param rx_len   Length of the receive buffer
 *  @return         0 when successful
 */
static int max32664c_bl_i2c_transmit(const struct device *dev, uint8_t *tx_buf, uint8_t tx_len,
				     uint8_t *rx_buf, uint8_t rx_len)
{
	int err;
	const struct max32664c_config *config = dev->config;

	err = i2c_write_dt(&config->i2c, tx_buf, tx_len);
	if (err) {
		printk("BL I2C WRITE ERR: %d (cmd 0x%02X 0x%02X)\n",
		       err, tx_buf[0], (tx_len > 1) ? tx_buf[1] : 0x00);
		return err;
	}

	k_msleep(MAX32664C_DEFAULT_CMD_DELAY_MS);

	err = i2c_read_dt(&config->i2c, rx_buf, rx_len);
	if (err) {
		printk("BL I2C READ ERR: %d (cmd 0x%02X 0x%02X)\n",
		       err, tx_buf[0], (tx_len > 1) ? tx_buf[1] : 0x00);
		return err;
	}

	k_msleep(MAX32664C_DEFAULT_CMD_DELAY_MS);

	printk("BL STATUS: cmd=0x%02X 0x%02X status=0x%02X\n",
	       tx_buf[0], (tx_len > 1) ? tx_buf[1] : 0x00, rx_buf[0]);

	if (rx_buf[0] != 0x00) {
		return rx_buf[0];
	}

	return 0;
}

/** @brief          Read application data from the sensor hub.
 *  @param dev      Pointer to device
 *  @param family   Family byte
 *  @param index    Index byte
 *  @param rx_buf   Pointer to receive buffer
 *                  NOTE: The buffer must be large enough to store the response and the status byte!
 *  @param rx_len   Length of receive buffer
 *  @return         0 when successful
 */
static int max32664c_app_i2c_read(const struct device *dev, uint8_t family, uint8_t index,
				  uint8_t *rx_buf, uint8_t rx_len)
{
	uint8_t tx_buf[] = {family, index};
	const struct max32664c_config *config = dev->config;

	/* Wake the sensor hub before starting an I2C read (see page 17 of the user Guide) */
	gpio_pin_set_dt(&config->mfio_gpio, false);
	k_usleep(300);

	i2c_write_dt(&config->i2c, tx_buf, sizeof(tx_buf));
	k_msleep(MAX32664C_DEFAULT_CMD_DELAY_MS);
	i2c_read_dt(&config->i2c, rx_buf, rx_len);
	k_msleep(MAX32664C_DEFAULT_CMD_DELAY_MS);

	gpio_pin_set_dt(&config->mfio_gpio, true);

	/* Check the status byte for a valid transaction */
	if (rx_buf[0] != 0) {
		return -EINVAL;
	}

	return 0;
}

/** @brief          Write a page of data into the sensor hub.
 *  @param dev      Pointer to device
 *  @param data     Pointer to firmware data
 *  @param offset   Start address in the firmware data
 *  @return         0 when successful
 */
static int max32664c_bl_write_page(const struct device *dev, const uint8_t *data, uint32_t offset)
{
    int err;
    uint8_t rx_buf;
    uint8_t *tx_buf;
    const struct max32664c_config *config = dev->config;

    tx_buf = (uint8_t *)k_malloc(MAX32664C_FW_UPDATE_WRITE_SIZE + 2);
    if (tx_buf == NULL) {
        return -ENOMEM;
    }

    memcpy(&tx_buf[2], &data[offset], MAX32664C_FW_UPDATE_WRITE_SIZE);
    tx_buf[0] = 0x80;
    tx_buf[1] = 0x04;

    for (int attempt = 0; attempt < 5; attempt++) {
        /* Try to recover the bus before every giant transfer */
        i2c_recover_bus(config->i2c.bus);
        k_msleep(20);

        err = i2c_write_dt(&config->i2c, tx_buf, MAX32664C_FW_UPDATE_WRITE_SIZE + 2);
        if (err) {
            printk("BL PAGE write err=%d attempt=%d\n", err, attempt + 1);
            k_msleep(100);
            continue;
        }

        k_msleep(800);

        err = i2c_read_dt(&config->i2c, &rx_buf, 1);
        if (err) {
            printk("BL PAGE read err=%d attempt=%d\n", err, attempt + 1);
            k_msleep(100);
            continue;
        }

        printk("BL PAGE status=0x%02X attempt=%d\n", rx_buf, attempt + 1);

        if (rx_buf == 0x00) {
            k_free(tx_buf);
            return 0;
        }

        if (rx_buf == 0x05) {
            k_msleep(100);
            continue;
        }

        k_free(tx_buf);
        return rx_buf;
    }

    k_free(tx_buf);
    return -EIO;
}

/** @brief      Erase the application from the sensor hub.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
static int max32664c_bl_erase_app(const struct device *dev)
{
	uint8_t tx_buf[2] = {0x80, 0x03};
	uint8_t rx_buf;
	const struct max32664c_config *config = dev->config;
	int err;

	err = i2c_write_dt(&config->i2c, tx_buf, sizeof(tx_buf));
	if (err) {
		printk("BL ERASE write err=%d\n", err);
		return err;
	}

	k_msleep(2200);

	err = i2c_read_dt(&config->i2c, &rx_buf, sizeof(rx_buf));
	if (err) {
		printk("BL ERASE read err=%d\n", err);
		return err;
	}

	k_msleep(MAX32664C_DEFAULT_CMD_DELAY_MS);

	printk("BL ERASE status=0x%02X\n", rx_buf);

	if (rx_buf != 0x00) {
		return rx_buf;
	}

	return 0;
}

/** @brief          Load the firmware into the hub.
 *                  NOTE: See User Guide, Table 9 for the required steps.
 *  @param dev      Pointer to device
 *  @param firmware Pointer to firmware data
 *  @param size     Firmware size
 *  @return         0 when successful
 */
static int max32664c_bl_load_fw(const struct device *dev, const uint8_t *firmware, uint32_t size)
{
	int err;
	uint8_t rx_buf;
	uint8_t tx_buf[18] = {0};
	uint32_t page_offset;
	uint8_t num_pages = firmware[0x44];

	printk("BL LOAD: size=%u num_pages=%u\n", size, num_pages);

	/* Set number of pages */
	tx_buf[0] = 0x80;
	tx_buf[1] = 0x02;
	tx_buf[2] = 0x00;
	tx_buf[3] = num_pages;
	err = max32664c_bl_i2c_transmit(dev, tx_buf, 4, &rx_buf, 1);
	printk("BL STEP set_num_pages -> %d\n", err);
	if (err) {
		return err;
	}

	/* Extract IV + auth from .msbl */
	memcpy(max32664c_fw_init_vector, &firmware[0x28], sizeof(max32664c_fw_init_vector));
	memcpy(max32664c_fw_auth_vector, &firmware[0x34], sizeof(max32664c_fw_auth_vector));

	/* Write IV */
	tx_buf[0] = 0x80;
	tx_buf[1] = 0x00;
	memcpy(&tx_buf[2], max32664c_fw_init_vector, sizeof(max32664c_fw_init_vector));
	err = max32664c_bl_i2c_transmit(dev, tx_buf, 13, &rx_buf, 1);
	printk("BL STEP write_iv -> %d\n", err);
	if (err) {
		return err;
	}

	/* Write auth */
	tx_buf[0] = 0x80;
	tx_buf[1] = 0x01;
	memcpy(&tx_buf[2], max32664c_fw_auth_vector, sizeof(max32664c_fw_auth_vector));
	err = max32664c_bl_i2c_transmit(dev, tx_buf, 18, &rx_buf, 1);
	printk("BL STEP write_auth -> %d\n", err);
	if (err) {
		return err;
	}

	/* Erase app */
	err = max32664c_bl_erase_app(dev);
	printk("BL STEP erase_app -> %d\n", err);
	if (err) {
		return err;
	}

	/* Write pages */
	page_offset = 0x4C;
	for (uint8_t i = 0; i < num_pages; i++) {
		int status;

		printk("BL PAGE %u/%u offset=0x%08X\n", i + 1, num_pages, page_offset);
		status = max32664c_bl_write_page(dev, firmware, page_offset);
		printk("BL PAGE %u status=0x%02X\n", i + 1, status);

		if (status != 0) {
			return status;
		}

		page_offset += MAX32664C_FW_UPDATE_WRITE_SIZE;
	}

	printk("BL STEP leave_bootloader\n");
	err = max32664c_bl_leave(dev);
	printk("BL STEP leave_bootloader -> %d\n", err);
	return err;
}

int max32664c_bl_enter(const struct device *dev, const uint8_t *firmware, uint32_t size)
{
	uint8_t rx_buf[4] = {0};
	uint8_t tx_buf[3];
	const struct max32664c_config *config = dev->config;

	gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT);
	gpio_pin_configure_dt(&config->mfio_gpio, GPIO_OUTPUT);

	/* Put the processor into bootloader mode */
	LOG_INF("Entering bootloader mode");
	gpio_pin_set_dt(&config->reset_gpio, false);
	k_msleep(20);

	gpio_pin_set_dt(&config->mfio_gpio, false);
	k_msleep(20);

	gpio_pin_set_dt(&config->reset_gpio, true);
	k_msleep(200);

	/* Set bootloader mode */
	tx_buf[0] = 0x01;
	tx_buf[1] = 0x00;
	tx_buf[2] = 0x08;
	if (max32664c_bl_i2c_transmit(dev, tx_buf, 3, rx_buf, 1)) {
		return -EINVAL;
	}

	/* Read the device mode */
	tx_buf[0] = 0x02;
	tx_buf[1] = 0x00;
	if (max32664c_bl_i2c_transmit(dev, tx_buf, 2, rx_buf, 2)) {
		return -EINVAL;
	}

	LOG_DBG("Mode: %x ", rx_buf[1]);
	if (rx_buf[1] != 8) {
		LOG_ERR("Device not in bootloader mode!");
		return -EINVAL;
	}

	/* Read the bootloader information */
	tx_buf[0] = 0x81;
	tx_buf[1] = 0x00;
	if (max32664c_bl_i2c_transmit(dev, tx_buf, 2, rx_buf, 4)) {
		return -EINVAL;
	}

	LOG_INF("Version: %d.%d.%d", rx_buf[1], rx_buf[2], rx_buf[3]);

	/* Read the bootloader page size */
	tx_buf[0] = 0x81;
	tx_buf[1] = 0x01;
	if (max32664c_bl_i2c_transmit(dev, tx_buf, 2, rx_buf, 3)) {
		return -EINVAL;
	}

	LOG_INF("Page size: %u", (uint16_t)(rx_buf[1] << 8) | rx_buf[2]);

	return max32664c_bl_load_fw(dev, firmware, size);
}

int max32664c_bl_leave(const struct device *dev)
{
    uint8_t hub_ver[3];
    uint8_t rx_buf[4] = {0};
    const struct max32664c_config *config = dev->config;

    gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT);
    gpio_pin_configure_dt(&config->mfio_gpio, GPIO_OUTPUT);

    /* Clean app-mode entry sequence */
    gpio_pin_set_dt(&config->reset_gpio, false);   // RSTN low
    k_msleep(20);

    gpio_pin_set_dt(&config->mfio_gpio, true);     // MFIO high
    k_msleep(2);                                   // >1 ms before reset release

    gpio_pin_set_dt(&config->reset_gpio, true);    // RSTN high
    k_msleep(1700);                                // wait for app startup

    if (max32664c_app_i2c_read(dev, 0x02, 0x00, rx_buf, 2)) {
        printk("APP mode read failed\n");
        return -EINVAL;
    }

    printk("APP mode = 0x%02X\n", rx_buf[1]);
    if (rx_buf[1] != 0x00) {
        printk("Device not in application mode\n");
        return -EINVAL;
    }

    if (max32664c_app_i2c_read(dev, 0xFF, 0x03, rx_buf, 4)) {
        printk("FW version read failed\n");
        return -EINVAL;
    }

    memcpy(hub_ver, &rx_buf[1], 3);
    printk("FW version: %u.%u.%u\n", hub_ver[0], hub_ver[1], hub_ver[2]);

    return 0;
}

int max32664c_bl_resume(const struct device *dev, const uint8_t *firmware, uint32_t size)
{
    uint8_t rx_buf[4] = {0};
    uint8_t tx_buf[2];

    /* Confirm current mode */
    tx_buf[0] = 0x02;
    tx_buf[1] = 0x00;
    if (max32664c_bl_i2c_transmit(dev, tx_buf, 2, rx_buf, 2)) {
        printk("BL: mode read failed\n");
        return -EINVAL;
    }

    printk("BL: current mode = 0x%02X\n", rx_buf[1]);

    if (rx_buf[1] != 0x08) {
        printk("BL: not in bootloader mode\n");
        return -EINVAL;
    }

    /* Read bootloader information */
    tx_buf[0] = 0x81;
    tx_buf[1] = 0x00;
    if (max32664c_bl_i2c_transmit(dev, tx_buf, 2, rx_buf, 4)) {
        printk("BL: bootloader info read failed\n");
        return -EINVAL;
    }

    printk("BL: version %u.%u.%u\n", rx_buf[1], rx_buf[2], rx_buf[3]);

    /* Read bootloader page size */
    tx_buf[0] = 0x81;
    tx_buf[1] = 0x01;
    if (max32664c_bl_i2c_transmit(dev, tx_buf, 2, rx_buf, 3)) {
        printk("BL: page size read failed\n");
        return -EINVAL;
    }

    printk("BL: page size %u\n", ((uint16_t)rx_buf[1] << 8) | rx_buf[2]);

    return max32664c_bl_load_fw(dev, firmware, size);
}
