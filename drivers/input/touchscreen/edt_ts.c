/*
 * Touch Screen driver for EDT ET070003DM6 display
 *
 * Mostly derived from migor_ts driver,
 * Copyright (c) 2008 Magnus Damm
 * Copyright (c) 2007 Ujjwal Pande <ujjwal@kenati.com>
 *
 * Copyright (c) 2011 DENX Software Engineering,
 * Anatolij Gustschin <agust@denx.de>
 *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU  General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#define EDT_EVENT_TOUCH_DOWN	0x0
#define EDT_EVENT_TOUCH_UP	0x4
#define EDT_EVENT_TOUCH_ON	0x8

struct edt_ts_priv {
	struct i2c_client *client;
	struct input_dev *input;
	int irq;
};

static irqreturn_t edt_ts_irq_thread(int irq, void *dev_id)
{
	struct edt_ts_priv *priv = dev_id;
	unsigned short xpos[2];
	unsigned short ypos[2];
	struct i2c_msg xfer[2];
	unsigned char buf[26], sendbuf[1];
	int event, touch_id;
	int ret;

	sendbuf[0] = 0xf9;

	memset(buf, 0, sizeof(buf));

	xfer[0].addr = xfer[1].addr = priv->client->addr;
	xfer[0].flags = 0;
	xfer[0].len = sizeof(sendbuf);
	xfer[0].buf = sendbuf;

	xfer[1].flags = I2C_M_RD;
	xfer[1].len = sizeof(buf);
	xfer[1].buf = buf;

	ret = i2c_transfer(priv->client->adapter, xfer, 2);
	if (ret != 2) {
		dev_err(&priv->client->dev, "I2C transfer failed\n");
		return IRQ_HANDLED;
	}

	for (ret = 0; ret < sizeof(buf); ret++)
		dev_dbg(&priv->client->dev, "%d: 0x%x\n", ret, buf[ret]);

	event = (buf[5] & 0xf0) >> 4;
	touch_id = (buf[7] & 0xf0) >> 4;

	dev_dbg(&priv->client->dev, "Event %d, TouchID %d\n", event, touch_id);

	xpos[0] = (((buf[5] & 0xf) << 8) | buf[6]) >> 1;
	ypos[0] = (((buf[7] & 0xf) << 8) | buf[8]) >> 1;

	if (touch_id == 0 && event & EDT_EVENT_TOUCH_UP) {
		input_report_key(priv->input, BTN_TOUCH, 0);
		input_sync(priv->input);
		return IRQ_HANDLED;
	}

	if (touch_id == 0 && event & EDT_EVENT_TOUCH_ON) {
		input_report_key(priv->input, BTN_TOUCH, 1);
		input_report_abs(priv->input, ABS_X, xpos[0]);
		input_report_abs(priv->input, ABS_Y, ypos[0]);
		input_sync(priv->input);
	}

	return IRQ_HANDLED;
}

static int edt_ts_open(struct input_dev *dev)
{
	struct edt_ts_priv *priv = input_get_drvdata(dev);
	int ret;
	char buf[2];

	/* check the controller access */
	ret = i2c_master_recv(priv->client, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		dev_err(&priv->client->dev, "controller access failed\n");
		return -EIO;
	}

	enable_irq(priv->irq);

	return 0;
}

static void edt_ts_close(struct input_dev *dev)
{
	struct edt_ts_priv *priv = input_get_drvdata(dev);

	disable_irq(priv->irq);
}

static int __devinit edt_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct edt_ts_priv *priv;
	struct input_dev *input;
	int error;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "can't allocate driver data\n");
		return -ENOMEM;
	}

	dev_set_drvdata(&client->dev, priv);

	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "can't allocate input device.\n");
		error = -ENOMEM;
		goto err0;
	}

	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input, ABS_X, 0, 800, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 480, 0, 0);

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	input->open = edt_ts_open;
	input->close = edt_ts_close;

	input_set_drvdata(input, priv);

	priv->client = client;
	priv->input = input;
	priv->irq = client->irq;

	error = request_threaded_irq(priv->irq, NULL, edt_ts_irq_thread,
			IRQF_TRIGGER_FALLING, client->name, priv);
	if (error) {
		dev_err(&client->dev, "can't request IRQ.\n");
		goto err1;
	}
	disable_irq(priv->irq);

	error = input_register_device(input);
	if (error) {
		dev_err(&client->dev, "can't register input device.\n");
		goto err2;
	}

	device_init_wakeup(&client->dev, 1);
	return 0;

err2:
	free_irq(priv->irq, priv);
err1:
	input_free_device(input);
err0:
	kfree(priv);
	return error;
}

static int __devexit edt_ts_remove(struct i2c_client *client)
{
	struct edt_ts_priv *priv = dev_get_drvdata(&client->dev);

	free_irq(priv->irq, priv);
	input_unregister_device(priv->input);
	kfree(priv);
	device_init_wakeup(&client->dev, 0);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int edt_ts_suspend(struct device *dev)
{
	struct edt_ts_priv *priv = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(priv->irq);
	else
		disable_irq(priv->irq);

	return 0;
}

static int edt_ts_resume(struct device *dev)
{
	struct edt_ts_priv *priv = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(priv->irq);
	else
		enable_irq(priv->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(edt_pm_ops, edt_ts_suspend, edt_ts_resume);

static const struct i2c_device_id edt_ts_idtbl[] = {
	{ "edt_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, edt_ts_idtbl);

static struct i2c_driver edt_ts_driver = {
	.driver = {
		.name = "edt_ts",
		.owner = THIS_MODULE,
		.pm = &edt_pm_ops,
	},
	.probe = edt_ts_probe,
	.remove = __devexit_p(edt_ts_remove),
	.id_table = edt_ts_idtbl,
};

static int __init edt_ts_init(void)
{
	return i2c_add_driver(&edt_ts_driver);
}
module_init(edt_ts_init);

static void __exit edt_ts_exit(void)
{
	i2c_del_driver(&edt_ts_driver);
}
module_exit(edt_ts_exit);

MODULE_DESCRIPTION("EDT Display Touchscreen driver");
MODULE_AUTHOR("Anatolij Gustschin <agust@denx.de>");
MODULE_LICENSE("GPL");
