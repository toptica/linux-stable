/*
 * ADS1112 ADC driver
  *
 * Copyright (C) 2010, TOPTICA Photonics AG (www.toptica.com)
 *
 * Arno Euteneuer <arno.euteneuer@toptica.com>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/delay.h>


static const unsigned short normal_i2c[] = { 0x48, 0x49, 0x4a, 0x4b, 0x4c,
					0x4d, 0x4e, 0x4e, I2C_CLIENT_END };
I2C_CLIENT_INSMOD ;

/* ADS1112 registers */
#define DRVNAME			"ads1112"

#define ADS1112_NOTREADY	(1 << 7)

#define ADS1112_INP_MASK	(3 << 5)
#define ADS1112_INP_01		(0 << 5)
#define ADS1112_INP_23		(1 << 5)
#define ADS1112_INP_03		(2 << 5)
#define ADS1112_INP_13		(3 << 5)

#define ADS1112_SINGLE_CONV	(1 << 4)

#define ADS1112_BITS_MASK	(3 << 2)
#define ADS1112_BITS_12		(0 << 2)
#define ADS1112_BITS_14		(1 << 2)
#define ADS1112_BITS_15		(2 << 2)
#define ADS1112_BITS_16		(3 << 2)

#define ADS1112_GAIN_MASK	(3 << 0)
#define ADS1112_GAIN_1		(0 << 0)
#define ADS1112_GAIN_2		(1 << 0)
#define ADS1112_GAIN_4		(2 << 0)
#define ADS1112_GAIN_8		(3 << 0)

/*-----------------------------------------------------------------------*/

inline int ads1112_set(struct i2c_client *client, u8 control)
{
	return i2c_master_send(client, &control, 1);
}

inline int ads1112_read(struct i2c_client *client, u8 *control, s16 *value)
{
	u8 buf[3];
	u16 tmp;
	int res = i2c_master_recv(client, buf, 3);
	/* dev_info(&client->dev, "read %02x %02x %02x\n", buf[0], buf[1], buf[2]); */
	if (control)
		*control = buf[2];
	if (value) {
		tmp = buf[0]*256 + buf[1];
		*value = tmp;
	}
	return res;
}

/*-----------------------------------------------------------------------*/


static ssize_t show_value(struct device *dev, struct device_attribute *da,
			 char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct i2c_client *client = to_i2c_client(dev);
	int res;
	unsigned int control;
	u8 status;
	s16 value;

	control = (unsigned int) i2c_get_clientdata(client);
	control &= ~ADS1112_INP_MASK;
	control |= attr->index | ADS1112_NOTREADY;

	res = ads1112_set(client, (u8) control);
	if (res<0) return res;

	switch (attr->index) {
	case ADS1112_BITS_16:
		msleep(66);  /* 15 sps */
	case ADS1112_BITS_15:
		msleep(33);  /* 30 sps */
	case ADS1112_BITS_14:
		msleep(16);	 /* 60 sps */
	case ADS1112_BITS_12:
		msleep(3);	 /* 240 sps */
	}

	do {
		msleep(1);
		res = ads1112_read(client, &status, &value);
		if (res<0) return res;

	} while (status & ADS1112_NOTREADY);

	switch (status & ADS1112_BITS_MASK) {
	case ADS1112_BITS_16:
		return sprintf(buf, "%d\n", (int) value);
	case ADS1112_BITS_15:
		return sprintf(buf, "%d\n", (int) 2*value);
	case ADS1112_BITS_14:
		return sprintf(buf, "%d\n", (int) 4*value);
	case ADS1112_BITS_12:
		return sprintf(buf, "%d\n", (int) 16*value);
	default:
		return sprintf(buf, "%d unknown resolution\n", (int) value);
	}
}

static ssize_t show_resolution(struct device *dev, struct device_attribute *da,
			 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned int  control;

	control = (unsigned int) i2c_get_clientdata(client);

	switch (control & ADS1112_BITS_MASK) {
	case ADS1112_BITS_16:
		return sprintf(buf, "16\n");
	case ADS1112_BITS_15:
		return sprintf(buf, "15\n");
	case ADS1112_BITS_14:
		return sprintf(buf, "14\n");
	case ADS1112_BITS_12:
		return sprintf(buf, "12\n");
	default:
		return -EIO;
	}
}

static ssize_t set_resolution(struct device *dev, struct device_attribute *da,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	long temp;
	unsigned int value;

	int status = strict_strtol(buf, 10, &temp);
	if (status < 0)
		return status;

	if (temp!=16 && temp!=15 && temp!=14 && temp!=12)
		return -EINVAL;

	value = (unsigned int) i2c_get_clientdata(client);

	value &= ~ADS1112_BITS_MASK;

	switch (temp) {
	case 16:
		value |= ADS1112_BITS_16;
		break;
	case 15:
		value |= ADS1112_BITS_15;
		break;
	case 14:
		value |= ADS1112_BITS_14;
		break;
	case 12:
		value |= ADS1112_BITS_12;
		break;
	}

	i2c_set_clientdata(client, (void *) value);
	return count;
}


static ssize_t show_gain(struct device *dev, struct device_attribute *da,
			 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 control;

	control = (u8) ((int) i2c_get_clientdata(client));

	switch (control & ADS1112_GAIN_MASK) {
	case ADS1112_GAIN_1:
		return sprintf(buf, "1\n");
	case ADS1112_GAIN_2:
		return sprintf(buf, "2\n");
	case ADS1112_GAIN_4:
		return sprintf(buf, "4\n");
	case ADS1112_GAIN_8:
		return sprintf(buf, "8\n");
	default:
		return -EIO;
	}
}

static ssize_t set_gain(struct device *dev, struct device_attribute *da,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	long temp;
	unsigned int value;

	int status = strict_strtol(buf, 10, &temp);
	if (status < 0)
		return status;

	if (temp!=1 && temp!=2 && temp!=4 && temp!=8)
		return -EINVAL;

	value = (unsigned int) i2c_get_clientdata(client);
	value &= ~ADS1112_GAIN_MASK;

	switch (temp) {
	case 1:
		value |= ADS1112_GAIN_1;
		break;
	case 2:
		value |= ADS1112_GAIN_2;
		break;
	case 4:
		value |= ADS1112_GAIN_4;
		break;
	case 8:
		value |= ADS1112_GAIN_8;
		break;
	}

	i2c_set_clientdata(client, (void *) value);

	return count;
}


/*-----------------------------------------------------------------------*/

/* sysfs attributes for hwmon */

static SENSOR_DEVICE_ATTR(adc01, S_IRUGO,
			show_value, NULL, ADS1112_INP_01);
static SENSOR_DEVICE_ATTR(adc03, S_IRUGO,
			show_value, NULL, ADS1112_INP_03);
static SENSOR_DEVICE_ATTR(adc13, S_IRUGO,
			show_value, NULL, ADS1112_INP_13);
static SENSOR_DEVICE_ATTR(adc23, S_IRUGO,
			show_value, NULL, ADS1112_INP_23);


static SENSOR_DEVICE_ATTR(bits, S_IWUSR | S_IRUGO,
			show_resolution, set_resolution, 0);
static SENSOR_DEVICE_ATTR(gain, S_IWUSR | S_IRUGO,
			show_gain, set_gain, 0);


static struct attribute *ads1112_attributes[] = {
	&sensor_dev_attr_adc01.dev_attr.attr,
	&sensor_dev_attr_adc03.dev_attr.attr,
	&sensor_dev_attr_adc13.dev_attr.attr,
	&sensor_dev_attr_adc23.dev_attr.attr,
	&sensor_dev_attr_bits.dev_attr.attr,
	&sensor_dev_attr_gain.dev_attr.attr,
	NULL
};

static const struct attribute_group ads1112_group = {
	.attrs = ads1112_attributes,
};

/*-----------------------------------------------------------------------*/

/* device probe and removal */

static int
ads1112_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int status;
	unsigned int control;

	control = ADS1112_SINGLE_CONV | ADS1112_GAIN_1 | ADS1112_BITS_16;
	status = ads1112_set(client, (u8) control);
	if (status < 0)
		return status;

	/* Register sysfs hooks */
	status = sysfs_create_group(&client->dev.kobj, &ads1112_group);
	if (status)
		return status;


	i2c_set_clientdata(client, (void *) control);
	dev_info(&client->dev, "ADC initialized\n");

	return 0;
}

static int ads1112_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &ads1112_group);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ads1112_ids[] = {
	{ "ads1112", 0, },
	{ /* LIST END */ }
};
MODULE_DEVICE_TABLE(i2c, ads1112_ids);



static struct i2c_driver ads1112_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "ads1112",
	},
	.probe		= ads1112_probe,
	.remove		= ads1112_remove,
	.id_table	= ads1112_ids,
	.detect		= NULL,
	.address_data	= &addr_data,
};

/* module glue */

static int __init sensors_ads1112_init(void)
{
	return i2c_add_driver(&ads1112_driver);
}

static void __exit sensors_ads1112_exit(void)
{
	i2c_del_driver(&ads1112_driver);
}

MODULE_AUTHOR("Arno Euteneuer <arno.euteneuer@toptica.com>");
MODULE_DESCRIPTION("ADS1112 driver");
MODULE_LICENSE("GPL");

module_init(sensors_ads1112_init);
module_exit(sensors_ads1112_exit);
