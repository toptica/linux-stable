/*
 * isl90840.c: Driver for the Intersil 4-channel digital potentiometer
 *
 * Author: Arno Euteneuer <arno.euteneuer@toptica.com>
 *
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>

#define DRIVER_NAME			"isl90840"
#define DRIVER_VERSION			"0.1"

/* sysfs functions */

static ssize_t sysfs_show_reg(struct device *dev,
			      struct device_attribute *attr, char *buf, u32 reg)
{
	struct i2c_client *client = to_i2c_client(dev);
	s32 value;

	value = i2c_smbus_read_byte_data(client, reg);

	if (value < 0)
		return -EINVAL;
	return sprintf(buf, "%u\n",value);
}

static ssize_t sysfs_set_reg(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count, u32 reg)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long value;
	int err;

	err = strict_strtoul(buf, 10, &value);
	if (err)
		return err;

	if (value > 0xff)
		value = 0xff;

	i2c_smbus_write_byte_data(client, reg, value);

	return count;
}

/* ------------------------------------------------------------------------- */

#define ISL90840_RDAC(index) 			\
static ssize_t set_rdac##index(struct device *dev, \
		struct device_attribute *da, \
		const char *buf, size_t count)	\
{ return sysfs_set_reg(dev, da, buf, count, index); }\
\
static ssize_t show_rdac##index(struct device *dev, \
		struct device_attribute *da, \
		char *buf) \
{ return sysfs_show_reg(dev, da, buf, index); }\
\
static DEVICE_ATTR(rdac##index, S_IWUSR | S_IRUGO, show_rdac##index,\
		set_rdac##index);

ISL90840_RDAC(0)
ISL90840_RDAC(1)
ISL90840_RDAC(2)
ISL90840_RDAC(3)


static struct attribute *isl90840_attributes_wipers[5] = {
		&dev_attr_rdac0.attr,
		&dev_attr_rdac1.attr,
		&dev_attr_rdac2.attr,
		&dev_attr_rdac3.attr,
		NULL
};

static const struct attribute_group isl90840_group_wipers = {
	.attrs = isl90840_attributes_wipers
};

/* ------------------------------------------------------------------------- */

static int isl90840_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int err = 0;

	dev_dbg(dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(dev, "missing I2C functionality for this driver\n");
		return -ENODEV;
	}

	err = sysfs_create_group(&dev->kobj,&isl90840_group_wipers);
	if (err) {
		dev_err(dev, "failed to register sysfs hooks\n");
		return err;
	}
	dev_info(dev, "ISL90840 Digital Potentiometer registered\n");

	return 0;
}

static int __devexit isl90840_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;

	sysfs_remove_group(&dev->kobj,
				   &isl90840_group_wipers);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id isl90840_idtable[] = {
	{"isl90840", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, isl90840_idtable);

static struct i2c_driver isl90840_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = DRIVER_NAME,
		   },
	.id_table = isl90840_idtable,
	.probe = isl90840_probe,
	.remove = __devexit_p(isl90840_remove),
};

static int __init isl90840_init(void)
{
	return i2c_add_driver(&isl90840_driver);
}

module_init(isl90840_init);

static void __exit isl90840_exit(void)
{
	i2c_del_driver(&isl90840_driver);
}

module_exit(isl90840_exit);

MODULE_AUTHOR("Arno Euteneuer <arno.euteneuer@toptica.com>");
MODULE_DESCRIPTION("ISL90840 digital potentiometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
