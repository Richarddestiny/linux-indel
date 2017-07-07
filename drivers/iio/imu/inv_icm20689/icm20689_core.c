/*
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/iio/iio.h>
#include <linux/i2c-mux.h>
#include <linux/acpi.h>
#include "icm20689.h"

/*
 * this is the gyro scale translated from dynamic range plus/minus
 * {250, 500, 1000, 2000} to rad/s
 */
static const int gyro_scale_6050[] = {133090, 266181, 532362, 1064724};

/*
 * this is the accel scale translated from dynamic range plus/minus
 * {2, 4, 8, 16} to m/s^2
 */
static const int accel_scale[] = {598, 1196, 2392, 4785};

static const struct icm20689_reg_map reg_set_6050 = {
	.sample_rate_div		= ICM20689_REG_SAMPLE_RATE_DIV,
	.config                 = ICM20689_REG_CONFIG,
	.accl_intel				= ICM20689_REG_ACCL_INTEL_CTRL,
	.user_ctrl              = ICM20689_REG_USER_CTRL,
	.fifo_en                = ICM20689_REG_FIFO_EN,
	.accl_wom_thr			= ICM20689_REG_ACCEL_WOM,
	.accl_wom_thr_x			= ICM20689_REG_ACCEL_WOM_X,
	.accl_wom_thr_y			= ICM20689_REG_ACCEL_WOM_Y,
	.accl_wom_thr_z			= ICM20689_REG_ACCEL_WOM_Z,
	.gyro_config            = ICM20689_REG_GYRO_CONFIG,
	.accl_config_1          = ICM20689_REG_ACCEL_CONFIG_1,
	.accl_config_2			= ICM20689_REG_ACCEL_CONFIG_2,
	.fifo_count_h           = ICM20689_REG_FIFO_COUNT_H,
	.fifo_r_w               = ICM20689_REG_FIFO_R_W,
	.raw_gyro               = ICM20689_REG_RAW_GYRO,
	.raw_accl               = ICM20689_REG_RAW_ACCEL,
	.temperature            = ICM20689_REG_TEMPERATURE,
	.int_enable             = ICM20689_REG_INT_ENABLE,
	.pwr_mgmt_1             = ICM20689_REG_PWR_MGMT_1,
	.pwr_mgmt_2             = ICM20689_REG_PWR_MGMT_2,
	.int_pin_cfg			= ICM20689_REG_INT_PIN_CFG,
	.int_status				= ICM20689_REG_INT_STATUS,
};

static const struct icm20689_chip_config chip_config_6050 = {
	.fsr = ICM20689_FSR_2000DPS,
	.lpf = ICM20689_FILTER_20HZ,
	.fifo_rate = ICM20689_INIT_FIFO_RATE,
	.gyro_fifo_enable = false,
	.accl_fifo_enable = false,
	.accl_fs = ICM20689_FS_02G,
};

static const struct icm20689_hw hw_info[INV_NUM_PARTS] = {
	{
		.num_reg = 117,
		.name = "ICM20689",
		.reg = &reg_set_6050,
		.config = &chip_config_6050,
	},
};


int icm20689_write_reg(struct icm20689_state *st, int reg, u8 d)
{
	return i2c_smbus_write_i2c_block_data(st->client, reg, 1, &d);
}

int icm20689_read_reg(struct icm20689_state *st, int reg, u8 d)
{
	return i2c_smbus_read_i2c_block_data(st->client, reg, 1, &d);
}


int icm20689_switch_engine(struct icm20689_state *st, bool en, u32 mask)
{
	u8 d, mgmt_1;
	int err;

	/* switch clock needs to be careful. Only when gyro is on, can
	   clock source be switched to gyro. Otherwise, it must be set to
	   internal clock */
	if (ICM20689_BIT_PWR_GYRO_STBY == mask) {
		err = i2c_smbus_read_i2c_block_data(st->client,
				       st->reg->pwr_mgmt_1, 1, &mgmt_1);
		if (err != 1)
			return err;

		mgmt_1 &= ~ICM20689_BIT_CLK_MASK;
	}

	if ((ICM20689_BIT_PWR_GYRO_STBY == mask) && (!en)) {
		/* turning off gyro requires switch to internal clock first.
		   Then turn off gyro engine */
		mgmt_1 |= INV_CLK_INTERNAL;
		err = icm20689_write_reg(st, st->reg->pwr_mgmt_1, mgmt_1);
		if (err)
			return err;
	}

	err = i2c_smbus_read_i2c_block_data(st->client,
				       st->reg->pwr_mgmt_2, 1, &d);
	if (err != 1)
		return err;
	if (en)
		d &= ~mask;
	else
		d |= mask;
	err = icm20689_write_reg(st, st->reg->pwr_mgmt_2, d);
	if (err)
		return err;

	if (en) {
		/* Wait for output stabilize */
		msleep(ICM20689_TEMP_UP_TIME);
		if (ICM20689_BIT_PWR_GYRO_STBY == mask) {
			/* switch internal clock to PLL */
			mgmt_1 |= INV_CLK_PLL;
			err = icm20689_write_reg(st,
					st->reg->pwr_mgmt_1, mgmt_1);
			if (err)
				return err;
		}
	}

	return 0;
}

int icm20689_set_power_itg(struct icm20689_state *st, bool power_on)
{
	int err = 0;

	if (power_on) {
		if (!st->powerup_count)
			err = icm20689_write_reg(st, st->reg->pwr_mgmt_1, 0);
		if (!err)
			st->powerup_count++;
	} else {
		st->powerup_count--;
		if (!st->powerup_count)
			err = icm20689_write_reg(st, st->reg->pwr_mgmt_1,
						       ICM20689_BIT_SLEEP);
	}

	if (err)
		return err;

	if (power_on)
		msleep(ICM20689_REG_UP_TIME);

	return 0;
}


static int icm20689_sensor_show(struct icm20689_state  *st, int reg,
				int axis, int *val)
{
	int ind, result;
	__be16 d;

	ind = (axis - IIO_MOD_X) * 2;
	result = i2c_smbus_read_i2c_block_data(st->client, reg + ind,  2,
						(u8 *)&d);
	if (result != 2)
		return -EINVAL;
	*val = (short)be16_to_cpup(&d);

	return IIO_VAL_INT;
}

static int icm20689_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask) {
	struct icm20689_state  *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	{
		int ret, result;

		ret = IIO_VAL_INT;
		result = 0;
		mutex_lock(&indio_dev->mlock);
		if (!st->chip_config.enable) {
			result = icm20689_set_power_itg(st, true);
			if (result)
				goto error_read_raw;
		}
		/* when enable is on, power is already on */
		switch (chan->type) {
		case IIO_ANGL_VEL:
			if (!st->chip_config.gyro_fifo_enable ||
					!st->chip_config.enable) {
				result = icm20689_switch_engine(st, true,
						ICM20689_BIT_PWR_GYRO_STBY);
				if (result)
					goto error_read_raw;
			}
			ret =  icm20689_sensor_show(st, st->reg->raw_gyro,
						chan->channel2, val);
			if (!st->chip_config.gyro_fifo_enable ||
					!st->chip_config.enable) {
				result = icm20689_switch_engine(st, false,
						ICM20689_BIT_PWR_GYRO_STBY);
				if (result)
					goto error_read_raw;
			}
			break;
		case IIO_ACCEL:
			if (!st->chip_config.accl_fifo_enable ||
					!st->chip_config.enable) {
				result = icm20689_switch_engine(st, true,
						ICM20689_BIT_PWR_ACCL_STBY);
				if (result)
					goto error_read_raw;
			}
			ret = icm20689_sensor_show(st, st->reg->raw_accl,
						chan->channel2, val);
			if (!st->chip_config.accl_fifo_enable ||
					!st->chip_config.enable) {
				result = icm20689_switch_engine(st, false,
						ICM20689_BIT_PWR_ACCL_STBY);
				if (result)
					goto error_read_raw;
			}
			break;
		case IIO_TEMP:
			/* wait for stablization */
			msleep(ICM20689_SENSOR_UP_TIME);
			icm20689_sensor_show(st, st->reg->temperature,
							IIO_MOD_X, val);
			break;
		default:
			ret = -EINVAL;
			break;
		}
error_read_raw:
		if (!st->chip_config.enable)
			result |= icm20689_set_power_itg(st, false);
		mutex_unlock(&indio_dev->mlock);
		if (result)
			return result;

		return ret;
	}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			*val  = 0;
			*val2 = gyro_scale_6050[st->chip_config.fsr];

			return IIO_VAL_INT_PLUS_NANO;
		case IIO_ACCEL:
			*val = 0;
			*val2 = accel_scale[st->chip_config.accl_fs];

			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_TEMP:
			*val = 0;
			*val2 = ICM20689_TEMP_SCALE;

			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_TEMP:
			*val = ICM20689_TEMP_OFFSET;

			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int icm20689_write_gyro_scale(struct icm20689_state *st, int val)
{
	int result, i;
	u8 d;

	for (i = 0; i < ARRAY_SIZE(gyro_scale_6050); ++i) {
		if (gyro_scale_6050[i] == val) {
			d = (i << ICM20689_GYRO_CONFIG_FSR_SHIFT);
			result = icm20689_write_reg(st,
					st->reg->gyro_config, d);
			if (result)
				return result;

			st->chip_config.fsr = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int inv_write_raw_get_fmt(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			return IIO_VAL_INT_PLUS_NANO;
		default:
			return IIO_VAL_INT_PLUS_MICRO;
		}
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}
static int icm20689_write_accel_scale(struct icm20689_state *st, int val)
{
	int result, i;
	u8 d;

	for (i = 0; i < ARRAY_SIZE(accel_scale); ++i) {
		if (accel_scale[i] == val) {
			d = (i << ICM20689_ACCL_CONFIG_FSR_SHIFT);
			result = icm20689_write_reg(st,
					st->reg->accl_config_1, d);
			if (result)
				return result;

			st->chip_config.accl_fs = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int icm20689_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask) {
	struct icm20689_state  *st = iio_priv(indio_dev);
	int result;

	mutex_lock(&indio_dev->mlock);
	/* we should only update scale when the chip is disabled, i.e.,
		not running */
	if (st->chip_config.enable) {
		result = -EBUSY;
		goto error_write_raw;
	}
	result = icm20689_set_power_itg(st, true);
	if (result)
		goto error_write_raw;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			result = icm20689_write_gyro_scale(st, val2);
			break;
		case IIO_ACCEL:
			result = icm20689_write_accel_scale(st, val2);
			break;
		default:
			result = -EINVAL;
			break;
		}
		break;
	default:
		result = -EINVAL;
		break;
	}

error_write_raw:
	result |= icm20689_set_power_itg(st, false);
	mutex_unlock(&indio_dev->mlock);

	return result;
}

/**
 *  icm20689_set_lpf() - set low pass filer based on fifo rate.
 *
 *                  Based on the Nyquist principle, the sampling rate must
 *                  exceed twice of the bandwidth of the signal, or there
 *                  would be alising. This function basically search for the
 *                  correct low pass parameters based on the fifo rate, e.g,
 *                  sampling frequency.
 */
static int icm20689_set_lpf(struct icm20689_state *st, int rate)
{
	const int hz[] = {188, 98, 42, 20, 10, 5};
	const int d[] = {ICM20689_FILTER_188HZ, ICM20689_FILTER_98HZ,
			ICM20689_FILTER_42HZ, ICM20689_FILTER_20HZ,
			ICM20689_FILTER_10HZ, ICM20689_FILTER_5HZ};
	int i, h, result;
	u8 data;

	h = (rate >> 1);
	i = 0;
	while ((h < hz[i]) && (i < ARRAY_SIZE(d) - 1))
		i++;
	data = d[i];
	result = icm20689_write_reg(st, st->reg->config, data);
	if (result)
		return result;
	st->chip_config.lpf = data;

	return 0;
}

/**
 * icm20689_fifo_rate_store() - Set fifo rate.
 */
static ssize_t icm20689_fifo_rate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	s32 fifo_rate;
	u8 d;
	int result;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct icm20689_state *st = iio_priv(indio_dev);

	if (kstrtoint(buf, 10, &fifo_rate))
		return -EINVAL;
	if (fifo_rate < ICM20689_MIN_FIFO_RATE ||
				fifo_rate > ICM20689_MAX_FIFO_RATE)
		return -EINVAL;
	if (fifo_rate == st->chip_config.fifo_rate)
		return count;

	mutex_lock(&indio_dev->mlock);
	if (st->chip_config.enable) {
		result = -EBUSY;
		goto fifo_rate_fail;
	}
	result = icm20689_set_power_itg(st, true);
	if (result)
		goto fifo_rate_fail;

	d = ICM20689_ONE_K_HZ / fifo_rate - 1;
	result = icm20689_write_reg(st, st->reg->sample_rate_div, d);
	if (result)
		goto fifo_rate_fail;
	st->chip_config.fifo_rate = fifo_rate;

	result = icm20689_set_lpf(st, fifo_rate);
	if (result)
		goto fifo_rate_fail;

fifo_rate_fail:
	result |= icm20689_set_power_itg(st, false);
	mutex_unlock(&indio_dev->mlock);
	if (result)
		return result;

	return count;
}

/**
 * inv_fifo_rate_show() - Get the current sampling rate.
 */
static ssize_t inv_fifo_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct icm20689_state *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->chip_config.fifo_rate);
}

/**
 * inv_attr_show() - calling this function will show current
 *                    parameters.
 */
static ssize_t inv_attr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct icm20689_state *st = iio_priv(dev_to_iio_dev(dev));
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct device_node *np = st->client->dev.of_node;
	s8 m[9];
	int err;

	switch (this_attr->address) {
	/* In MPU6050, the two matrix are the same because gyro and accel     of_property_read_u8_array
	   are integrated in one chip */
	case ATTR_GYRO_MATRIX:
	case ATTR_ACCL_MATRIX:
		err = of_property_read_u8_array(np,"orientation",m,9);

		return sprintf(buf, "%d, %d, %d; %d, %d, %d; %d, %d, %d\n",
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
	default:
		return -EINVAL;
	}
}

/**
 * icm20689_validate_trigger() - validate_trigger callback for invensense
 *                                  MPU6050 device.
 * @indio_dev: The IIO device
 * @trig: The new trigger
 *
 * Returns: 0 if the 'trig' matches the trigger registered by the MPU6050
 * device, -EINVAL otherwise.
 */
static int icm20689_validate_trigger(struct iio_dev *indio_dev,
					struct iio_trigger *trig)
{
	struct icm20689_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

#define ICM20689_CHAN(_type, _channel2, _index)\
	{\
		.type = _type,\
		.modified = 1,\
		.channel2 = _channel2,\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),\
		.scan_index = _index,\
		.scan_type = {\
				.sign = 's',\
				.realbits = 16,\
				.storagebits = 16,\
				.shift = 0 ,                          \
				.endianness = IIO_BE,                 \
			     },                                   \
	}

static const struct iio_chan_spec icm20689_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(ICM20689_SCAN_TIMESTAMP),
	/*
	 * Note that temperature should only be via polled reading only,
	 * not the final scan elements output.
	 */
	{
		.type = IIO_TEMP,
		.info_mask_separate =  BIT(IIO_CHAN_INFO_RAW)
				| BIT(IIO_CHAN_INFO_OFFSET)
				| BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = -1,
	},
	ICM20689_CHAN(IIO_ANGL_VEL, IIO_MOD_X, ICM20689_SCAN_GYRO_X),
	ICM20689_CHAN(IIO_ANGL_VEL, IIO_MOD_Y, ICM20689_SCAN_GYRO_Y),
	ICM20689_CHAN(IIO_ANGL_VEL, IIO_MOD_Z, ICM20689_SCAN_GYRO_Z),

	ICM20689_CHAN(IIO_ACCEL, IIO_MOD_X, ICM20689_SCAN_ACCL_X),
	ICM20689_CHAN(IIO_ACCEL, IIO_MOD_Y, ICM20689_SCAN_ACCL_Y),
	ICM20689_CHAN(IIO_ACCEL, IIO_MOD_Z, ICM20689_SCAN_ACCL_Z),
};


static ssize_t set_wom_thr(struct device *dev,
          	  	  	  	struct device_attribute *attr,
						const char *buffer, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct icm20689_state *st = iio_priv(indio_dev);
	ssize_t err;
	int buf;
	err = sscanf(buffer, "%d", &buf);
	st->wom_thr = (u8)buf;
	return count;
}

/* Attach the sysfs write method */
DEVICE_ATTR(wom_thr, 0644, NULL, set_wom_thr);

/* constant IIO attribute */
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("10 20 50 100 200 500");
static IIO_DEV_ATTR_SAMP_FREQ(S_IRUGO | S_IWUSR, inv_fifo_rate_show,
	icm20689_fifo_rate_store);
static IIO_DEVICE_ATTR(in_gyro_matrix, S_IRUGO, inv_attr_show, NULL,
	ATTR_GYRO_MATRIX);
static IIO_DEVICE_ATTR(in_accel_matrix, S_IRUGO, inv_attr_show, NULL,
	ATTR_ACCL_MATRIX);

static struct attribute *inv_attributes[] = {
	&iio_dev_attr_in_gyro_matrix.dev_attr.attr,
	&iio_dev_attr_in_accel_matrix.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	&dev_attr_wom_thr.attr,
	NULL,
};

static const struct attribute_group inv_attribute_group = {
	.attrs = inv_attributes
};

static const struct iio_info mpu_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &icm20689_read_raw,
	.write_raw = &icm20689_write_raw,
	.write_raw_get_fmt = &inv_write_raw_get_fmt,
	.attrs = &inv_attribute_group,
	.validate_trigger = icm20689_validate_trigger,
};

/**
 *  inv_check_and_setup_chip() - check and setup chip.
 */
static int inv_check_and_setup_chip(struct icm20689_state *st,
		const struct i2c_device_id *id)
{
	int err = 0;

	st->chip_type = ICM20689;
	st->hw  = &hw_info[st->chip_type];
	st->reg = hw_info[st->chip_type].reg;

	/* Reset device */
	err = icm20689_write_reg(st, st->reg->pwr_mgmt_1,
								ICM20689_BIT_H_RESET );
	if (err)
		return err;

	msleep(ICM20689_POWER_UP_TIME);

	/* toggle power state. After reset, the sleep bit could be on
		or off depending on the OTP settings. Toggling power would
		make it in a definite state as well as making the hardware
		state align with the software state */
	err = icm20689_set_power_itg(st, false);
	if (err)
		return err;
	err = icm20689_set_power_itg(st, true);
	if (err)
		return err;

	err = icm20689_switch_engine(st, false,
								ICM20689_BIT_PWR_ACCL_STBY);
	if (err)
		return err;
	err = icm20689_switch_engine(st, false,
								ICM20689_BIT_PWR_GYRO_STBY);

	if (err)
		return err;

	return 0;
}

/**
 *  icm20689_init_config() - Initialize hardware, disable FIFO.
 *
 *  Initial configuration:
 *  FSR: ± 2000DPS
 *  DLPF: 20Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 */
static int icm20689_init_config(struct iio_dev *indio_dev)
{
	int err;
	u8 d;
	struct icm20689_state *st = iio_priv(indio_dev);

	err = icm20689_set_power_itg(st, true);
	if (err)
		return err;
	/* Gyro Full Scale Select */
	d = (ICM20689_FSR_2000DPS << ICM20689_GYRO_CONFIG_FSR_SHIFT);
	err = icm20689_write_reg(st, st->reg->gyro_config, d);
	if (err)
		return err;
	/* Select low pass filter */
	d = ICM20689_FILTER_20HZ;
	err = icm20689_write_reg(st, st->reg->config, d);
	if (err)
		return err;
	/* Set sample rate divider */
	d = ICM20689_ONE_K_HZ / ICM20689_INIT_FIFO_RATE - 1;
	err = icm20689_write_reg(st, st->reg->sample_rate_div, d);
	if (err)
		return err;
	/* Set Accel Full Scale */
	d = (ICM20689_FS_02G << ICM20689_ACCL_CONFIG_FSR_SHIFT);
	err = icm20689_write_reg(st, st->reg->accl_config_1, d);
	if (err)
		return err;
	/* Enable DMP */
	d=ICM20689_BIT_DMP_EN;
	err = icm20689_write_reg(st, st->reg->user_ctrl, d);
	if (err)
		return err;
	/* Set Interrupt configs */
	d = ICM20689_BITS_INT_PIN_CFG;
	err = icm20689_write_reg(st, st->reg->int_pin_cfg, d);
	if (err)
			return err;

	/* Enable Interrupts*/
	d = ICM20689_BITS_INT_ENABLE;
	err = icm20689_write_reg(st, st->reg->int_enable, d);
	if (err)
		return err;

	memcpy(&st->chip_config, hw_info[st->chip_type].config,                 /* ?????????????????????????????????*/
		sizeof(struct icm20689_chip_config));

	/* Sleep mode */
	err = icm20689_set_power_itg(st, false);

	return err;
}

/**
 * st_irq_handler - dummy irq handler
 *
 * Not used yet
 *
 */
static irqreturn_t st_irq_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}


/**
 *  icm20689_probe() - probe function.
 *  @client:          i2c client.
 *  @id:              i2c device id.
 *
 *  Returns 0 on success, a negative error code otherwise.
 */
static int icm20689_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct icm20689_state *st;
	struct iio_dev *indio_dev;
	int err;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENOSYS;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->client = client;
	st->powerup_count = 0;
	st->wom_thr = 0xF0;

	/* power is turned on inside check chip type*/
	err = inv_check_and_setup_chip(st, id);

	if (err)
		return err;

	err = icm20689_init_config(indio_dev);

	if (err) {
		dev_err(&client->dev,
			"Could not initialize device.\n");
		return err;
	}

	/* Needed for wakeup capability */
	err = devm_request_threaded_irq(&client->dev, client->irq,
						  NULL, st_irq_handler,
						  IRQF_ONESHOT,
						  client->name, st);

	i2c_set_clientdata(client, indio_dev);


	indio_dev->dev.parent = &client->dev;
	indio_dev->name = (char *)id->name;
	indio_dev->channels = icm20689_channels;
	indio_dev->num_channels = ARRAY_SIZE(icm20689_channels);

	indio_dev->info = &mpu_info;
	indio_dev->modes = INDIO_BUFFER_TRIGGERED;

	err = iio_triggered_buffer_setup(indio_dev,
					    icm20689_irq_handler,
					    icm20689_read_fifo,
					    NULL);


	if (err) {
		dev_err(&st->client->dev, "configure buffer fail %d\n",
				err);
		return err;
	}
	err = icm20689_probe_trigger(indio_dev);
	if (err) {
		dev_err(&st->client->dev, "trigger probe fail %d\n", err);
	}

	INIT_KFIFO(st->timestamps);
	spin_lock_init(&st->time_stamp_lock);

	err = iio_device_register(indio_dev);

	if (err) {
		dev_err(&st->client->dev, "IIO register fail %d\n", err);
	}

	/* Device is a wake-up source*/
	device_init_wakeup(&client->dev, 1);

	return err;
}


/* set wake-on-motion interrupt */
static int icm20689_set_motion_int(struct icm20689_state *st)
{
	int err = 0;
	u8 d = 0x00;

	/* Ensure that accelerometer is running
	err = icm20689_read_reg(st,st->reg->pwr_mgmt_1,d);
	d |= 0x00;
	d &= ~0x00;*/
	err = icm20689_write_reg(st, st->reg->pwr_mgmt_1,0x00);
	if (err)
		return err;

	/*
	err = icm20689_read_reg(st,st->reg->pwr_mgmt_2,d);
	d &= ~ICM20689_BIT_PWR_ACCL_STBY;
	d |= ICM20689_BIT_PWR_GYRO_STBY;*/
	err = icm20689_write_reg(st, st->reg->pwr_mgmt_2,0x07);
	if (err)
		return err;

	/* Accelerometer configuration */
	d = 0x01;
	err = icm20689_write_reg(st, st->reg->accl_config_2, 0x01);
	if (err)
		return err;
	/* Enable motion interrupt
	err = icm20689_read_reg(st,st->reg->int_enable,d);
	d |= ICM20689_BITS_WOM_ENABLE;
	d &= ~ICM20689_BITS_INT_ENABLE;*/
	err = icm20689_write_reg(st, st->reg->int_enable, 0xE0);
	if (err)
		return err;

	/* Set motion threshold */
	err = icm20689_write_reg(st, st->reg->accl_wom_thr, st->wom_thr);
	err = icm20689_write_reg(st, st->reg->accl_wom_thr_x, st->wom_thr);
	err = icm20689_write_reg(st, st->reg->accl_wom_thr_y, st->wom_thr);
	err = icm20689_write_reg(st, st->reg->accl_wom_thr_z, st->wom_thr);
	if (err)
		return err;

	/* Enable accelerometer hardware intelligence */
	err = icm20689_read_reg(st,st->reg->accl_intel,d);
	d |= ICM20689_BIT_ACCL_INTEL_ENABLE;
	err = icm20689_write_reg(st, st->reg->accl_intel, 0xC0);
	if (err)
		return err;

	/* Set frequency of Wake-Up */
	d = 0x08;
	err = icm20689_write_reg(st, st->reg->sample_rate_div, 0x08);
	if (err)
		return err;

	/* Enable cycle mode (accelerometer low-power mode)
	err = icm20689_read_reg(st,st->reg->pwr_mgmt_1,d);
	d |= ICM20689_BIT_CYCLE	; */
	err = icm20689_write_reg(st, st->reg->pwr_mgmt_1, 0x20);
	if (err)
		return err;

	return 0;
}

static int icm20689_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct icm20689_state *st = iio_priv(indio_dev);
	device_init_wakeup(&client->dev, 0);

	iio_device_unregister(indio_dev);
	/* iio_device_free(); not requiered -> devm (managed)*/
	icm20689_remove_trigger(st);
	iio_triggered_buffer_cleanup(indio_dev);

	return 0;
}


static int __maybe_unused icm20689_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct icm20689_state *st = iio_priv(indio_dev);
	int err = 0;
	u8 d = 0;

	err = icm20689_read_reg(st,st->reg->int_status,d);

	if (device_may_wakeup(&client->dev)) {
		disable_irq_wake(client->irq);
		icm20689_init_config(indio_dev);
	} else {
		err = icm20689_set_power_itg(st, true);
		enable_irq(client->irq);
	}

	return 0;
}

static int __maybe_unused icm20689_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct icm20689_state *st = iio_priv(indio_dev);

	if (device_may_wakeup(&client->dev)) {
		enable_irq_wake(client->irq);
		icm20689_set_motion_int(st);
	} else {
		disable_irq(client->irq);
		icm20689_set_power_itg(st, false);
	}


	return 0;
}
static SIMPLE_DEV_PM_OPS(icm20689_pm_ops, icm20689_suspend, icm20689_resume);


/*
 * device id table is used to identify what device can be
 * supported by this driver
 */
static const struct i2c_device_id icm20689_id[] = {
	{"icm20689", ICM20689},
	{}
};

MODULE_DEVICE_TABLE(i2c, icm20689_id);

static struct i2c_driver icm20689_driver = {
	.probe		=	icm20689_probe,
	.remove		=	icm20689_remove,
	.id_table	=	icm20689_id,
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	"icm20689",
		.pm     =   &icm20689_pm_ops,
	},
};

module_i2c_driver(icm20689_driver);

MODULE_AUTHOR("V.Z INDEL AG");
MODULE_DESCRIPTION("TDK / Invensense device ICM-20689 driver");
MODULE_LICENSE("GPL");
