/* ST Microelectronics LIS2HH12 3-axis accelerometer driver
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lis2hh12.pdf
 */

#define DT_DRV_COMPAT st_lis2hh12

#include <zephyr/init.h>
#include <stdlib.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <zephyr/drivers/i2c.h>
#endif

#include "lis2hh12.h"

LOG_MODULE_REGISTER(LIS2HH12, CONFIG_SENSOR_LOG_LEVEL);

/**
 * lis2hh12_set_range - set full scale range for acc
 * @dev: Pointer to instance of struct device
 * @fs: Full scale range (2, 4, 8 and 16 G)
 */
static int lis2hh12_set_range(const struct device *dev, uint8_t fs)
{
	int err;
	const struct lis2hh12_device_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	lis2hh12_xl_fs_t val;
	switch (fs) {
	case 2:
		val = LIS2HH12_2g;
		break;
	case 4:
		val = LIS2HH12_4g;
		break;
	case 8:
		val = LIS2HH12_8g;
		break;

	default:
		LOG_ERR("%s: bad range %d", dev->name, fs);
		return -ENOTSUP;
	}

	err = lis2hh12_xl_full_scale_set(ctx, val);
	return err;
}

/**
 * lis2hh12_set_odr - set new sampling frequency
 * @dev: Pointer to instance of struct device
 * @odr: Output data rate
 */
static int lis2hh12_set_odr(const struct device *dev, uint16_t odr)
{
	const struct lis2hh12_device_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lis2hh12_data *lis2hh12 = dev->data;
	lis2hh12_xl_data_rate_t val;

	switch (odr) {
	case 0:
		val = LIS2HH12_XL_ODR_OFF;
		break;
	case 10:
		val = LIS2HH12_XL_ODR_10Hz;
		break;
	case 50:
		val = LIS2HH12_XL_ODR_50Hz;
		break;
	case 100:
		val = LIS2HH12_XL_ODR_100Hz;
		break;
	case 200:
		val = LIS2HH12_XL_ODR_200Hz;
		break;
	case 400:
		val = LIS2HH12_XL_ODR_400Hz;
		break;
	case 800:
		val = LIS2HH12_XL_ODR_800Hz;
		break;

	default:
		LOG_ERR("%s: bad odr %d", dev->name, odr);
		return -ENOTSUP;
	}


	lis2hh12->odr = odr;
	return lis2hh12_xl_data_rate_set(ctx, val);
}

static inline void lis2hh12_convert(struct sensor_value *val, int raw_val,
				    float gain)
{
	int64_t dval;

	/* Gain is in ug/LSB */
	/* Convert to m/s^2 */
	dval = ((int64_t)raw_val * gain * SENSOR_G) / 1000000LL;
	val->val1 = dval / 1000000LL;
	val->val2 = dval % 1000000LL;
}

static inline void lis2hh12_channel_get_acc(const struct device *dev,
					     enum sensor_channel chan,
					     struct sensor_value *val)
{
	int i;
	uint8_t ofs_start, ofs_stop;
	struct lis2hh12_data *lis2hh12 = dev->data;
	struct sensor_value *pval = val;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		ofs_start = ofs_stop = 0U;
		break;
	case SENSOR_CHAN_ACCEL_Y:
		ofs_start = ofs_stop = 1U;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		ofs_start = ofs_stop = 2U;
		break;
	default:
		ofs_start = 0U; ofs_stop = 2U;
		break;
	}

	for (i = ofs_start; i <= ofs_stop ; i++) {
		lis2hh12_convert(pval++, lis2hh12->acc[i], lis2hh12->gain);
	}
}

static int lis2hh12_channel_get(const struct device *dev,
				 enum sensor_channel chan,
				 struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		lis2hh12_channel_get_acc(dev, chan, val);
		return 0;
	default:
		LOG_DBG("Channel not supported");
		break;
	}

	return -ENOTSUP;
}

static int lis2hh12_config(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return lis2hh12_set_range(dev,sensor_ms2_to_g(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return lis2hh12_set_odr(dev, val->val1);
	default:
		LOG_DBG("Acc attribute not supported");
		break;
	}

	return -ENOTSUP;
}


static inline int32_t sensor_ms2_to_mg(const struct sensor_value *ms2)
{
	int64_t nano_ms2 = (ms2->val1 * 1000000LL + ms2->val2) * 1000LL;

	if (nano_ms2 > 0) {
		return (nano_ms2 + SENSOR_G / 2) / SENSOR_G;
	} else {
		return (nano_ms2 - SENSOR_G / 2) / SENSOR_G;
	}
}

static int lis2hh12_attr_set(const struct device *dev,
			      enum sensor_channel chan,
			      enum sensor_attribute attr,
			      const struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		return lis2hh12_config(dev, chan, attr, val);
	default:
		LOG_DBG("Attr not supported on %d channel", chan);
		break;
	}

	return -ENOTSUP;
}

static int lis2hh12_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	struct lis2hh12_data *lis2hh12 = dev->data;
	const struct lis2hh12_device_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int16_t buf[3];

	/* fetch raw data sample */
	if (lis2hh12_acceleration_raw_get(ctx, buf) < 0) {
		LOG_DBG("Failed to fetch raw data sample");
		return -EIO;
	}

	lis2hh12->acc[0] = sys_le16_to_cpu(buf[0]);
	lis2hh12->acc[1] = sys_le16_to_cpu(buf[1]);
	lis2hh12->acc[2] = sys_le16_to_cpu(buf[2]);

	return 0;
}

static const struct sensor_driver_api lis2hh12_driver_api = {
	.attr_set = lis2hh12_attr_set,
	.sample_fetch = lis2hh12_sample_fetch,
	.channel_get = lis2hh12_channel_get,
};


static int lis2hh12_init(const struct device *dev)
{
	const struct lis2hh12_device_config *cfg = dev->config;
	struct lis2hh12_data *lis2hh12 = dev->data;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	uint8_t wai;
	int ret;

	/* check chip ID */
	ret = lis2hh12_dev_id_get(ctx, &wai);
	if (ret < 0) {
		LOG_ERR("Not able to read dev id");
		return ret;
	}

	if (wai != LIS2HH12_ID) {
		LOG_ERR("Invalid chip ID");
		return -EINVAL;
	}

	/* reset device */
	ret = lis2hh12_dev_reset_set(ctx, PROPERTY_ENABLE);
	if (ret < 0) {
		return ret;
	}

	k_busy_wait(100);

	/* Enable Block Data Update */
	ret = lis2hh12_block_data_update_set(ctx, PROPERTY_ENABLE);
	if (ret < 0) {
		LOG_ERR("Not able to set BDU");
		return ret;
	}

	/* Configure filtering chain */
    /* Accelerometer data output- filter path / bandwidth */
    ret = lis2hh12_xl_filter_aalias_bandwidth_set(ctx, LIS2HH12_AUTO);
	if (ret < 0) {
		LOG_ERR("Not able to set filter_aalias_bandwidth");
		return ret;
	}

    ret = lis2hh12_xl_filter_out_path_set(ctx, LIS2HH12_FILT_LP);
	if (ret < 0) {
		LOG_ERR("Not able to set filter_out_path");
		return ret;
	}

    ret = lis2hh12_xl_filter_low_bandwidth_set(ctx,
                                        LIS2HH12_LP_ODR_DIV_400);
	if (ret < 0) {
		LOG_ERR("Not able to set filter_low_bandwidth");
		return ret;
	}

	/* Accelerometer interrupt diosable - filter path / bandwidth */
    ret = lis2hh12_xl_filter_int_path_set(ctx, LIS2HH12_HP_DISABLE);
	if (ret < 0) {
		LOG_ERR("Not able to set filter_int_path");
		return ret;
	}

	/* set the output data rate */
	ret = lis2hh12_set_odr(dev, cfg->odr);
	if (ret < 0) {
		LOG_ERR("odr init error %d", cfg->odr);
		return ret;
	}

	lis2hh12->odr = cfg->odr;

	LOG_DBG("range is %d", cfg->range);
	ret = lis2hh12_set_range(dev, cfg->range);
	if (ret < 0) {
		LOG_ERR("range init error %d", cfg->range);
		return ret;
	}

	return 0;
}

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "LIS2HH12 driver enabled without any devices"
#endif

/*
 * Device creation macro and
 * LIS2HH12_DEFINE_I2C().
 */

#define LIS2HH12_DEVICE_INIT(inst)					\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			    lis2hh12_init,				\
			    NULL,					\
			    &lis2hh12_data_##inst,			\
			    &lis2hh12_config_##inst,			\
			    POST_KERNEL,				\
			    CONFIG_SENSOR_INIT_PRIORITY,		\
			    &lis2hh12_driver_api);

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define LIS2HH12_CONFIG_I2C(inst)					\
	{								\
		.ctx = {						\
			.read_reg =					\
			   (stmdev_read_ptr) stmemsc_i2c_read,		\
			.write_reg =					\
			   (stmdev_write_ptr) stmemsc_i2c_write,	\
			.handle =					\
			   (void *)&lis2hh12_config_##inst.stmemsc_cfg,	\
		},							\
		.stmemsc_cfg = {					\
			.i2c = I2C_DT_SPEC_INST_GET(inst),		\
		},							\
		.odr = DT_INST_PROP_OR(inst, odr, 12),			\
		.range = DT_INST_PROP(inst, range),			\
		.bw_filt = DT_INST_PROP(inst, bw_filt),      \
		.hp_filter_path = DT_INST_PROP(inst, hp_filter_path),      \
		.hp_ref_mode = DT_INST_PROP(inst, hp_ref_mode), \
		.drdy_pulsed = DT_INST_PROP(inst, drdy_pulsed),      \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, irq_gpios),	\
			(LIS2HH12_CFG_IRQ(inst)), ())			\
	}

/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define LIS2HH12_DEFINE(inst)						\
	static struct lis2hh12_data lis2hh12_data_##inst;		\
	static const struct lis2hh12_device_config lis2hh12_config_##inst =	\
	LIS2HH12_CONFIG_I2C(inst);			\
	LIS2HH12_DEVICE_INIT(inst)

DT_INST_FOREACH_STATUS_OKAY(LIS2HH12_DEFINE)
