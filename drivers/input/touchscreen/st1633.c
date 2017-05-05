/*
 * ST1633 Touchscreen Controller Driver
 *
 * Copyright (C) 2017 Indel AG.
 *
 * Using code from:
 *  - android.git.kernel.org: projects/kernel/common.git: synaptics_i2c_rmi.c
 *	Copyright (C) 2007 Google, Inc.
 *  - st1232.c
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/types.h>


#define ST1633_TS_NAME	"st1633-ts"

#define MIN_X		0
#define MIN_Y		0
#define MAX_X		479	/* (480 - 1) */
#define MAX_Y		271	/* (272 - 1) */
#define MAX_AREA	20
#define MAX_FINGERS	2

struct st1633_ts_finger {
	u16 x;
	u16 y;
	bool is_valid;
};

struct st1633_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct st1633_ts_finger finger[MAX_FINGERS];
	struct dev_pm_qos_request low_latency_req;
};

static int st1633_ts_read_data(struct st1633_ts_data *ts)
{
	struct st1633_ts_finger *finger = ts->finger;
	struct i2c_client *client = ts->client;
	struct i2c_msg msg[2];
	int i, y, error;
	u8 start_reg;
	u8 buf[MAX_FINGERS*4];

	/* read touchscreen data from st1633 */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x12;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	error = i2c_transfer(client->adapter, msg, 2);
	if (error < 0)
		return error;

	/* get "valid" bits */
	for (i = 0,y = 0; i < MAX_FINGERS; i++, y+=3) {
		finger[i].is_valid = buf[i+y] >> 7;
		if(finger[i].is_valid){
			finger[i].x = ((buf[i+y] & 0x0070) << 4) | buf[i+1];
			finger[i].y = ((buf[i+y] & 0x0007) << 8) | buf[i+2];
		}
	}
	return 0;
}

static irqreturn_t st1633_ts_irq_handler(int irq, void *dev_id)
{
	struct st1633_ts_data *ts = dev_id;
	struct st1633_ts_finger *finger = ts->finger;
	struct input_dev *input_dev = ts->input_dev;
	int count = 0;
	int i, ret;

	ret = st1633_ts_read_data(ts);
	if (ret < 0)
		goto end;

	/* multi touch protocol */
	for (i = 0; i < MAX_FINGERS; i++) {
		if (!finger[i].is_valid)
			continue;

		input_report_abs(input_dev, ABS_MT_POSITION_X, finger[i].x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[i].y);
		input_mt_sync(input_dev);
		count++;
	}
	input_report_key(input_dev, BTN_TOUCH, 1);

	/* SYN_MT_REPORT only if no contact */
	if (!count) {
		input_mt_sync(input_dev);
		if (ts->low_latency_req.dev) {
			dev_pm_qos_remove_request(&ts->low_latency_req);
			ts->low_latency_req.dev = NULL;
		}
	} else if (!ts->low_latency_req.dev) {
		/* First contact, request 100 us latency. */
		dev_pm_qos_add_ancestor_request(&ts->client->dev,
						&ts->low_latency_req,
						DEV_PM_QOS_RESUME_LATENCY, 100);
	}

	/* SYN_REPORT */
	input_sync(input_dev);

end:
	return IRQ_HANDLED;
}

static int st1633_ts_power(struct st1633_ts_data *ts, bool poweron)
{
	struct device_node *np = ts->client->dev.of_node;
	int gpio;

	if (!np)
		return -ENODEV;

	gpio = of_get_named_gpio(np,"wakeup-gpios",0);
	if (!gpio_is_valid(gpio)){
			dev_err(&ts->client->dev, "no Power ON?\n");
			return -ENODEV;
	}

	gpio_direction_output(gpio, poweron);

	return 0;
}

static int st1633_ts_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct st1633_ts_data *ts;
	struct input_dev *input_dev;
	int error;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		return -EIO;
	}

	if (!client->irq) {
		dev_err(&client->dev, "no IRQ?\n");
		return -EINVAL;
	}

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev)
		return -ENOMEM;

	ts->client = client;
	ts->input_dev = input_dev;

	error = st1633_ts_power(ts, true);

	if (error) {
		dev_err(&client->dev, "no Power ON?\n");
		return -EINVAL;
	}

	input_dev->name = "st1633-touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xFF;
	input_dev->id.version = 0x1001;
	input_dev->dev.parent = &client->dev;

	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);
	/* Only for debug tools, which do not support Multitouch */
	input_set_abs_params(input_dev, ABS_X, 			   MIN_X, MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 			   MIN_Y, MAX_Y, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, MIN_X, MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, MIN_Y, MAX_Y, 0, 0);



	if (error) {
		dev_err(&client->dev, "Unable to init MT slots.\n");
		return error;
	}

	input_set_drvdata(input_dev, ts);

	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, st1633_ts_irq_handler,
					  IRQF_ONESHOT,
					  client->name, ts);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		return error;
	}

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&client->dev, "Unable to register %s input device\n",
			input_dev->name);
		return error;
	}

	i2c_set_clientdata(client, ts);
	device_init_wakeup(&client->dev, 1);

	return 0;
}

static int st1633_ts_remove(struct i2c_client *client)
{
	struct st1633_ts_data *ts = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, 0);
	st1633_ts_power(ts, false);

	return 0;
}

static int __maybe_unused st1633_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1633_ts_data *ts = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev)) {
		enable_irq_wake(client->irq);
	} else {
		disable_irq(client->irq);
		st1633_ts_power(ts, false);
	}

	return 0;
}

static int __maybe_unused st1633_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1633_ts_data *ts = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev)) {
		disable_irq_wake(client->irq);
	} else {
		st1633_ts_power(ts, true);
		enable_irq(client->irq);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(st1633_ts_pm_ops,
			 st1633_ts_suspend, st1633_ts_resume);

static const struct i2c_device_id st1633_ts_id[] = {
	{ ST1633_TS_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, st1232_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id st1633_ts_dt_ids[] = {
	{ .compatible = "sitronix,st1633", },
	{ }
};
MODULE_DEVICE_TABLE(of, st1633_ts_dt_ids);
#endif

static struct i2c_driver st1633_ts_driver = {
	.probe		= st1633_ts_probe,
	.remove		= st1633_ts_remove,
	.id_table	= st1633_ts_id,
	.driver = {
		.name	= ST1633_TS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(st1633_ts_dt_ids),
		.pm	= &st1633_ts_pm_ops,
	},
};

module_i2c_driver(st1633_ts_driver);

MODULE_AUTHOR("V.Z. <zuellig@indel.ch>");
MODULE_DESCRIPTION("SITRONIX ST1633 Touchscreen Controller Driver");
MODULE_LICENSE("GPL");
