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

#include "icm20689.h"

static void inv_scan_query(struct iio_dev *indio_dev)
{
	struct icm20689_state  *st = iio_priv(indio_dev);

	st->chip_config.gyro_fifo_enable =
		test_bit(ICM20689_SCAN_GYRO_X,
			indio_dev->active_scan_mask) ||
			test_bit(ICM20689_SCAN_GYRO_Y,
			indio_dev->active_scan_mask) ||
			test_bit(ICM20689_SCAN_GYRO_Z,
			indio_dev->active_scan_mask);

	st->chip_config.accl_fifo_enable =
		test_bit(ICM20689_SCAN_ACCL_X,
			indio_dev->active_scan_mask) ||
			test_bit(ICM20689_SCAN_ACCL_Y,
			indio_dev->active_scan_mask) ||
			test_bit(ICM20689_SCAN_ACCL_Z,
			indio_dev->active_scan_mask);
}

/**
 *  icm20689_set_enable() - enable chip functions.
 *  @indio_dev:	Device driver instance.
 *  @enable: enable/disable
 */
static int icm20689_set_enable(struct iio_dev *indio_dev, bool enable)
{
	struct icm20689_state *st = iio_priv(indio_dev);
	int result;

	if (enable) {
		result = icm20689_set_power_itg(st, true);
		if (result)
			return result;
		inv_scan_query(indio_dev);
		if (st->chip_config.gyro_fifo_enable) {
			result = icm20689_switch_engine(st, true,
					ICM20689_BIT_PWR_GYRO_STBY);
			if (result)
				return result;
		}
		if (st->chip_config.accl_fifo_enable) {
			result = icm20689_switch_engine(st, true,
					ICM20689_BIT_PWR_ACCL_STBY);
			if (result)
				return result;
		}
		result = inv_reset_fifo(indio_dev);
		if (result)
			return result;
	} else {
		result = icm20689_write_reg(st, st->reg->fifo_en, 0);
		if (result)
			return result;

		result = icm20689_write_reg(st, st->reg->int_enable, 0);
		if (result)
			return result;

		result = icm20689_write_reg(st, st->reg->user_ctrl, 0);
		if (result)
			return result;

		result = icm20689_switch_engine(st, false,
					ICM20689_BIT_PWR_GYRO_STBY);
		if (result)
			return result;

		result = icm20689_switch_engine(st, false,
					ICM20689_BIT_PWR_ACCL_STBY);
		if (result)
			return result;
		result = icm20689_set_power_itg(st, false);
		if (result)
			return result;
	}
	st->chip_config.enable = enable;

	return 0;
}

/**
 * inv_mpu_data_rdy_trigger_set_state() - set data ready interrupt state
 * @trig: Trigger instance
 * @state: Desired trigger state
 */
static int inv_mpu_data_rdy_trigger_set_state(struct iio_trigger *trig,
						bool state)
{
	return icm20689_set_enable(iio_trigger_get_drvdata(trig), state);
}

static const struct iio_trigger_ops inv_mpu_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &inv_mpu_data_rdy_trigger_set_state,
};

int icm20689_probe_trigger(struct iio_dev *indio_dev)
{
	int ret;
	struct icm20689_state *st = iio_priv(indio_dev);

	st->trig = devm_iio_trigger_alloc(&indio_dev->dev,
					  "%s-dev%d",
					  indio_dev->name,
					  indio_dev->id);
	if (!st->trig)
		return -ENOMEM;

	ret = devm_request_irq(&indio_dev->dev, st->client->irq,
			       &iio_trigger_generic_data_rdy_poll,
				   IRQF_ONESHOT,
			       "icm20689",
			       st->trig);
	if (ret)
		return ret;

	st->trig->dev.parent = &st->client->dev;
	st->trig->ops = &inv_mpu_trigger_ops;
	iio_trigger_set_drvdata(st->trig, indio_dev);

	ret = iio_trigger_register(st->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trig);

	return 0;
}

void icm20689_remove_trigger(struct icm20689_state *st)
{
	iio_trigger_unregister(st->trig);
}
