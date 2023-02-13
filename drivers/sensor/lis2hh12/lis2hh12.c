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

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "lis2hh12.h"

LOG_MODULE_REGISTER(LIS2HH12, CONFIG_SENSOR_LOG_LEVEL);

static int lis2hh12_set_odr(const struct device *dev, uint8_t odr)
{
	const struct lis2hh12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lis2hh12_xl_data_rate_t val;

	switch (odr) {
	case LIS2HH12_XL_ODR_OFF:
		val = LIS2HH12_XL_ODR_OFF;
		break;
	case LIS2HH12_XL_ODR_10Hz:
		val = LIS2HH12_XL_ODR_10Hz;
		break;
	case LIS2HH12_XL_ODR_50Hz:
		val = LIS2HH12_XL_ODR_50Hz;
		break;
	case LIS2HH12_XL_ODR_100Hz:
		val = LIS2HH12_XL_ODR_100Hz;
		break;
	case LIS2HH12_XL_ODR_200Hz:
		val = LIS2HH12_XL_ODR_200Hz;
		break;
	case LIS2HH12_XL_ODR_400Hz:
		val = LIS2HH12_XL_ODR_400Hz;
		break;
	case LIS2HH12_XL_ODR_800Hz:
		val = LIS2HH12_XL_ODR_800Hz;
		break;

	default:
		LOG_ERR("%s: bad odr %d", dev->name, odr);
		return -ENOTSUP;
	}

	return lis2hh12_xl_data_rate_set(ctx, val);
}

static int lis2hh12_set_range(const struct device *dev, uint8_t range)
{
	int err;
	struct lis2hh12_data *data = dev->data;
	const struct lis2hh12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	switch (range) {
	default:
	case 2U:
		err = lis2hh12_xl_full_scale_set(ctx, LIS2HH12_2g);
		data->gain = lis2hh12_from_fs2g_to_mg(1);
		break;
	case 4U:
		err = lis2hh12_xl_full_scale_set(ctx, LIS2HH12_4g);
		data->gain = lis2hh12_from_fs4g_to_mg(1);
		break;
	case 8U:
		err = lis2hh12_xl_full_scale_set(ctx, LIS2HH12_8g);
		data->gain = lis2hh12_from_fs8g_to_mg(1);
		break;
	}

	return err;
}

static int lis2hh12_accel_config(const struct device *dev,
				 enum sensor_channel chan,
				 enum sensor_attribute attr,
				 const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return lis2hh12_set_range(dev, sensor_ms2_to_g(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		LOG_DBG("%s: set odr to %d Hz", dev->name, val->val1);
		return lis2hh12_set_odr(dev, LIS2HH12_ODR_TO_REG(val->val1));
	default:
		LOG_DBG("Accel attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int lis2hh12_attr_set(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		return lis2hh12_accel_config(dev, chan, attr, val);
	default:
		LOG_WRN("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

static int lis2hh12_sample_fetch_accel(const struct device *dev)
{
	struct lis2hh12_data *data = dev->data;
	const struct lis2hh12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int16_t buf[3];

	/* fetch raw data sample */
	if (lis2hh12_acceleration_raw_get(ctx, buf) < 0) {
		LOG_ERR("Failed to fetch raw data sample");
		return -EIO;
	}

	data->sample_x = sys_le16_to_cpu(buf[0]);
	data->sample_y = sys_le16_to_cpu(buf[1]);
	data->sample_z = sys_le16_to_cpu(buf[2]);

	return 0;
}

static int lis2hh12_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		lis2hh12_sample_fetch_accel(dev);
		break;
#if defined(CONFIG_LIS2HH12_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		/* ToDo:
		lis2hh12_sample_fetch_temp(dev)
		*/
		break;
#endif
	case SENSOR_CHAN_ALL:
		lis2hh12_sample_fetch_accel(dev);
#if defined(CONFIG_LIS2HH12_ENABLE_TEMP)
		/* ToDo:
		lis2hh12_sample_fetch_temp(dev)
		*/
#endif
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline void lis2hh12_convert(struct sensor_value *val, int raw_val,
				    float gain)
{
	int64_t dval;

	/* Gain is in mg/LSB */
	/* Convert to m/s^2 */
	dval = ((int64_t)raw_val * gain * SENSOR_G) / 1000;
	val->val1 = dval / 1000000LL;
	val->val2 = dval % 1000000LL;
}

static inline int lis2hh12_get_channel(enum sensor_channel chan,
					     struct sensor_value *val,
					     struct lis2hh12_data *data,
					     float gain)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		lis2hh12_convert(val, data->sample_x, gain);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		lis2hh12_convert(val, data->sample_y, gain);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		lis2hh12_convert(val, data->sample_z, gain);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		lis2hh12_convert(val, data->sample_x, gain);
		lis2hh12_convert(val + 1, data->sample_y, gain);
		lis2hh12_convert(val + 2, data->sample_z, gain);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int lis2hh12_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct lis2hh12_data *data = dev->data;

	return lis2hh12_get_channel(chan, val, data, data->gain);
}

static const struct sensor_driver_api lis2hh12_driver_api = {
	.attr_set = lis2hh12_attr_set,
#if defined(CONFIG_LIS2HH12_TRIGGER)
	.trigger_set = lis2hh12_trigger_set,
#endif
	.sample_fetch = lis2hh12_sample_fetch,
	.channel_get = lis2hh12_channel_get,
};

static int lis2hh12_init(const struct device *dev)
{
	const struct lis2hh12_config * const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	uint8_t chip_id;
	int ret;

	/* check chip ID */
	ret = lis2hh12_dev_id_get(ctx, &chip_id);
	if (ret < 0) {
		LOG_ERR("%s: Not able to read dev id", dev->name);
		return ret;
	}

	if (chip_id != LIS2HH12_ID) {
		LOG_ERR("%s: Invalid chip ID 0x%02x", dev->name, chip_id);
		return -EINVAL;
	}

	/* reset device */
	ret = lis2hh12_dev_reset_set(ctx, PROPERTY_ENABLE);
	if (ret < 0) {
		return ret;
	}

	k_busy_wait(100);

	LOG_DBG("%s: chip id 0x%x", dev->name, chip_id);

#ifdef CONFIG_LIS2HH12_TRIGGER
	ret = lis2hh12_trigger_init(dev);
	if (ret < 0) {
		LOG_ERR("%s: Failed to initialize triggers", dev->name);
		return ret;
	}
#endif

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

    ret = lis2hh12_xl_filter_low_bandwidth_set(ctx, LIS2HH12_LP_ODR_DIV_400);
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

	/* set sensor default odr */
	LOG_DBG("%s: odr: %d", dev->name, cfg->odr);
	ret = lis2hh12_set_odr(dev, cfg->odr);
	if (ret < 0) {
		LOG_ERR("%s: odr init error %d", dev->name, cfg->odr);
		return ret;
	}

	/* set sensor default scale */
	LOG_DBG("%s: range is %d", dev->name, cfg->range);
	ret = lis2hh12_set_range(dev, cfg->range);
	if (ret < 0) {
		LOG_ERR("%s: range init error %d", dev->name, cfg->range);
		return ret;
	}

	return 0;
}

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "LIS2HH12 driver enabled without any devices"
#endif

/*
 * Device creation macro, shared by LIS2HH12_DEFINE_SPI() and
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
 * Instantiation macros used when a device is on a SPI bus.
 */

#ifdef CONFIG_LIS2HH12_TRIGGER
#define LIS2HH12_CFG_IRQ(inst) \
	.gpio_int = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),
#else
#define LIS2HH12_CFG_IRQ(inst)
#endif /* CONFIG_LIS2HH12_TRIGGER */

#define LIS2HH12_SPI_OPERATION (SPI_WORD_SET(8) |			\
				SPI_OP_MODE_MASTER |			\
				SPI_MODE_CPOL |				\
				SPI_MODE_CPHA)				\

#define LIS2HH12_CONFIG_SPI(inst)					\
	{								\
		.ctx = {						\
			.read_reg =					\
			   (stmdev_read_ptr) stmemsc_spi_read,		\
			.write_reg =					\
			   (stmdev_write_ptr) stmemsc_spi_write,	\
			.handle =					\
			   (void *)&lis2hh12_config_##inst.stmemsc_cfg,	\
		},							\
		.stmemsc_cfg = {					\
			.spi = SPI_DT_SPEC_INST_GET(inst,		\
					   LIS2HH12_SPI_OPERATION,	\
					   0),				\
		},							\
		.range = DT_INST_PROP(inst, range),			\
		.odr = DT_INST_PROP(inst, odr),				\
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, irq_gpios),	\
			(LIS2HH12_CFG_IRQ(inst)), ())			\
	}

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
		.range = DT_INST_PROP(inst, range),			\
		.odr = DT_INST_PROP(inst, odr),				\
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, irq_gpios),	\
			(LIS2HH12_CFG_IRQ(inst)), ())			\
	}

/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define LIS2HH12_DEFINE(inst)						\
	static struct lis2hh12_data lis2hh12_data_##inst;		\
	static const struct lis2hh12_config lis2hh12_config_##inst =	\
	COND_CODE_1(DT_INST_ON_BUS(inst, spi),				\
		    (LIS2HH12_CONFIG_SPI(inst)),			\
		    (LIS2HH12_CONFIG_I2C(inst)));			\
	LIS2HH12_DEVICE_INIT(inst)

DT_INST_FOREACH_STATUS_OKAY(LIS2HH12_DEFINE)
