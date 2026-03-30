/*
 * Copyright (c) 2025 Daniel Kampert
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX32664C_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX32664C_H_

#include <zephyr/device.h>

#define MAX32664C_MOTION_TIME(ms) ((uint8_t)((ms * 25UL) / 1000))

#define MAX32664C_MOTION_THRESHOLD(mg) ((uint8_t)((mg * 16UL) / 1000))

/* MAX32664C specific channels */

enum sensor_channel_max32664c {
        SENSOR_CHAN_MAX32664C_HEARTRATE = SENSOR_CHAN_PRIV_START,
        SENSOR_CHAN_MAX32664C_BLOOD_OXYGEN_SATURATION,
        SENSOR_CHAN_MAX32664C_RESPIRATION_RATE,
        SENSOR_CHAN_MAX32664C_SKIN_CONTACT,
        SENSOR_CHAN_MAX32664C_ACTIVITY,
        SENSOR_CHAN_MAX32664C_STEP_COUNTER,
};

/* MAX32664C specific attributes */

enum sensor_attribute_max32664c {
        SENSOR_ATTR_MAX32664C_GENDER = SENSOR_ATTR_PRIV_START,
        SENSOR_ATTR_MAX32664C_AGE,
        SENSOR_ATTR_MAX32664C_WEIGHT,
        SENSOR_ATTR_MAX32664C_HEIGHT,
        SENSOR_ATTR_MAX32664C_OP_MODE,
        SENSOR_ATTR_MAX32664C_DAC_OFFSET,
        SENSOR_ATTR_MAX32664C_INTEGRATION_TIME,
        SENSOR_ATTR_MAX32664C_PPG_CONFIG_1,
};

#ifdef __cplusplus
extern "C" {
#endif

enum max32664c_device_mode {
        MAX32664C_OP_MODE_IDLE,
        MAX32664C_OP_MODE_RAW,
        /* For hardware testing purposes, the user may choose to start the sensor hub to collect
         * raw PPG samples. In this case, the host configures the sensor hub to work in Raw Data
         * mode (no algorithm) by enabling the accelerometer and the AFE.
         */
        MAX32664C_OP_MODE_ALGO_AEC,
        /* Automatic Exposure Control (AEC) is Maxim’s gain control algorithm that is superior to
         * AGC. The AEC algorithm optimally maintains the best SNR range and power optimization. The
         * targeted SNR range is maintained regardless of skin color or ambient temperature within
         * the limits of the LED currents configurations; The AEC dynamically manages the
         * appropriate register settings for sampling rate, LED current, pulse width and integration
         * time.
         */
        MAX32664C_OP_MODE_ALGO_AEC_EXT,
        MAX32664C_OP_MODE_ALGO_AGC,
        /* In this mode, the wearable algorithm suite (SpO2 and WHRM) is enabled and the R value,
         * SpO2, SpO2 confidence level, heart rate, heart rate confidence level, RR value, and
         * activity class are reported. Furthermore, automatic gain control (AGC) is enabled.
         * Because AGC is a subset of AEC functionality, to enable AGC, AEC still needs to be
         * enabled. However, automatic calculation of target PD should be turned off, and the
         * desired level of AGC target PD current is set by the user. The user may change the
         * algorithm to the desired configuration mode. If signal quality is poor, the user may need
         * to adjust the AGC settings to maintain optimal performance. If signal quality is low, a
         * LowSNR flag will be set. Excessive motion is also reported with a flag.
         */
        MAX32664C_OP_MODE_ALGO_AGC_EXT,
        MAX32664C_OP_MODE_SCD,
        MAX32664C_OP_MODE_WAKE_ON_MOTION,
        MAX32664C_OP_MODE_EXIT_WAKE_ON_MOTION,
        MAX32664C_OP_MODE_STOP_ALGO,
};

enum max32664c_algo_mode {
        MAX32664C_ALGO_MODE_CONT_HR_CONT_SPO2,
        MAX32664C_ALGO_MODE_CONT_HR_SHOT_SPO2,
        MAX32664C_ALGO_MODE_CONT_HRM,
        /* NOTE: These algorithm modes are untested */
        /*MAX32664C_ALGO_MODE_SAMPLED_HRM,*/
        /*MAX32664C_ALGO_MODE_SAMPLED_HRM_SHOT_SPO2,*/
        /*MAX32664C_ALGO_MODE_ACTIVITY_TRACK,*/
        /*MAX32664C_ALGO_MODE_SAMPLED_HRM_FAST_SPO2 = 7,*/
};

enum max32664c_algo_gender {
        MAX32664_ALGO_GENDER_MALE,
        MAX32664_ALGO_GENDER_FEMALE,
};

enum max32664c_algo_activity {
        MAX32664C_ALGO_ACTIVITY_REST,
        MAX32664C_ALGO_ACTIVITY_OTHER,
        MAX32664C_ALGO_ACTIVITY_WALK,
        MAX32664C_ALGO_ACTIVITY_RUN,
        MAX32664C_ALGO_ACTIVITY_BIKE,
};

struct max32664c_acc_data_t {
        int16_t x;
        int16_t y;
        int16_t z;
} __packed;

// int max32664c_set_mode_raw(const struct device *dev);

int max32664c_bl_enter(const struct device *dev, const uint8_t *firmware, uint32_t size);

int max32664c_bl_leave(const struct device *dev);

int max32664c_acc_fill_fifo(const struct device *dev, struct max32664c_acc_data_t *data,
                            uint8_t length);

int max32664c_set_mode_raw(const struct device *dev);

int max32664c_set_mode_algo(const struct device *dev, uint8_t device_mode, uint8_t algo_mode, bool extended);


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX32664C_H_ */
