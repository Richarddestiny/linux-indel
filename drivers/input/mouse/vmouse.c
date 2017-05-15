/*
 *  Virtual mouse driver for Linux/ARM
 *
 *
 *
 *  Copyright (c) 2017 Indel AG V.Z.
 *
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This handles the Acorn RiscPCs mouse.  We basically have a couple of
 * hardware registers that track the sensor count for the X-Y movement and
 * another register holding the button state.  On every VSYNC interrupt we read
 * the complete state and then work out if something has changed.
 */

#include <linux/module.h>
#include <linux/ptrace.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/platform_device.h>

MODULE_AUTHOR("V.Z.");
MODULE_DESCRIPTION("Virtual mouse driver");
MODULE_LICENSE("GPL");


static struct input_dev *vmouse_input_dev;
static struct platform_device *vmouse_dev; /* Device structure */

static ssize_t write_vmouse(struct device *dev,
          	  	  	  	 struct device_attribute *attr,
						 const char *buffer, size_t count)
{
    int x,y;

    sscanf(buffer, "%d%d", &x, &y);
    /* Report relative coordinates via the
       event interface */
    input_report_rel(vmouse_input_dev, REL_X, x);
    input_report_rel(vmouse_input_dev, REL_Y, y);
    input_sync(vmouse_input_dev);
    return count;
}

/* Attach the sysfs write method */
DEVICE_ATTR(coordinates, 0644, NULL, write_vmouse);

/* Attribute Descriptor */
static struct attribute *vmouse_attrs[] = {
    &dev_attr_coordinates.attr,
    NULL
};

/* Attribute group */
static struct attribute_group vmouse_attr_group = {
    .attrs = vmouse_attrs,
};


static int vmouse_open(struct input_dev *dev)
{
	return 0;
}

static void vmouse_close(struct input_dev *dev)
{

}


static int __init vmouse_init(void)
{
	int err;

	/* Register a platform device */
	vmouse_dev = platform_device_register_simple("vmouse", -1, NULL, 0);
	if (IS_ERR(vmouse_dev)){
		printk ("vmouse_init: error\n");
	    return PTR_ERR(vmouse_dev);
	}

	/* Create a sysfs node to read simulated coordinates */
	err = sysfs_create_group(&vmouse_dev->dev.kobj, &vmouse_attr_group);

	vmouse_input_dev = input_allocate_device();
	if (!vmouse_input_dev)
		return -ENOMEM;

	vmouse_input_dev->name = "Virtual Mouse";
	vmouse_input_dev->phys = "vmouse/input0";
	vmouse_input_dev->id.bustype = BUS_VIRTUAL;
	vmouse_input_dev->id.vendor  = 0x0005;
	vmouse_input_dev->id.product = 0x0001;
	vmouse_input_dev->id.version = 0x0100;

	vmouse_input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);
	vmouse_input_dev->keybit[BIT_WORD(BTN_LEFT)] = BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_MIDDLE) | BIT_MASK(BTN_RIGHT);
	vmouse_input_dev->relbit[0]	= BIT_MASK(REL_X) | BIT_MASK(REL_Y);

	vmouse_input_dev->open = vmouse_open;
	vmouse_input_dev->close = vmouse_close;

	err = input_register_device(vmouse_input_dev);
	if (err)
		goto err_free_dev;

	return 0;

 err_free_dev:
	input_free_device(vmouse_input_dev);

	return err;
}

static void __exit vmouse_exit(void)
{
	input_unregister_device(vmouse_input_dev);
}

module_init(vmouse_init);
module_exit(vmouse_exit);


