/* ST Microelectronics LIS2HH12 3-axis accelerometer driver
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lis2hh12.pdf
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_LIS2HH12_LIS2HH12_H_
#define ZEPHYR_DRIVERS_SENSOR_LIS2HH12_LIS2HH12_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>
#include <stmemsc.h>
#include "lis2hh12_reg.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <zephyr/drivers/i2c.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

/* Acc Gain value in ug/LSB in High Perf mode */
#define LIS2HH12_FS_2G_GAIN		244
#define LIS2HH12_FS_4G_GAIN		488
#define LIS2HH12_FS_8G_GAIN		976
#define LIS2HH12_FS_16G_GAIN		1952

#define LIS2HH12_SHFT_GAIN_NOLP1	2
#define LIS2HH12_ACCEL_GAIN_DEFAULT_VAL LIS2HH12_FS_2G_GAIN
#define LIS2HH12_FS_TO_GAIN(_fs, _lp1) \
		(LIS2HH12_FS_2G_GAIN << ((_fs) + (_lp1)))

/**
 * struct lis2hh12_device_config - lis2hh12 hw configuration
 * @bus_name: Pointer to bus master identifier.
 * @int_pin: Sensor int pin (int1/int2).
 */
struct lis2hh12_device_config {
	stmdev_ctx_t ctx;
	union {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
		const struct i2c_dt_spec i2c;
#endif
	} stmemsc_cfg;
	uint16_t odr;
	uint8_t range;
	uint8_t bw_filt;
	bool low_noise;
	bool hp_filter_path;
	bool hp_ref_mode;
	bool drdy_pulsed;
};

/* sensor data */
struct lis2hh12_data {
	int16_t acc[3];

	 /* save sensitivity */
	uint16_t gain;
	 /* output data rate */
	uint16_t odr;
};


#endif /* ZEPHYR_DRIVERS_SENSOR_LIS2HH12_LIS2HH12_H_ */
