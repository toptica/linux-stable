/*
 * CXA2765ER laser driver IC driver
 *
 *
 * Copyright (C) 2010 Arno Euteneuer <arno.euteneuer@toptica.com>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/ctype.h>

#define CXA2765ER_READ 		(1 << 7)
#define CXA2765ER_WRITE		(0 << 7)
#define CXA2765ER_MASK		0x7f

#define CXA2765ER_modeldd	0x78
#define CXA2765ER_hfmp		0x79
#define CXA2765ER_hfmf		0x7a
#define CXA2765ER_emisel	0x7b
#define CXA2765ER_ldcr		0x7c
#define CXA2765ER_chsel		0x7d

struct cxa2765er_data {
	struct spi_device	*spi;
	struct spi_message	msg;
	struct spi_transfer	xfer;
	uint8_t tx_buf[2];
	uint8_t rx_buf[2];
	uint8_t regvalues[6];
	char is_ok;
};

static int cxa2765er_read_register(struct device *dev, char channel)
{
	struct cxa2765er_data *data = dev_get_drvdata(dev);
	if (data->is_ok)
		return data->regvalues[channel - CXA2765ER_modeldd];
	else
		return -EIO;
}

static int cxa2765er_write_register(struct device *dev, char channel, uint8_t value)
{
	struct cxa2765er_data *data = dev_get_drvdata(dev);
	int err;

	data->tx_buf[1] = CXA2765ER_WRITE | (channel  & CXA2765ER_MASK);
	data->tx_buf[0] = value;

	err = spi_sync(data->spi, &data->msg);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		return err;
	}

	data->tx_buf[1] = CXA2765ER_READ | (channel  & CXA2765ER_MASK);
	data->tx_buf[0] = 0;

	err = spi_sync(data->spi, &data->msg);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		return err;
	}
	/* read back data, one bit will be missing ...*/
	if ((value >> 1) != data->rx_buf[0]) {
		dev_err(dev, "consistency check register 0x%02x failed 0x%02x != 0x%02x\n",
				channel, value >> 1, data->rx_buf[0]);
		data->is_ok = 0;
		return -EIO;
	}
	data->is_ok = 1;
	data->regvalues[channel - CXA2765ER_modeldd] = value;
	return 0;
}


#define SHOW_FUNC(name)	\
static ssize_t show_reg_##name(struct device *dev, 		\
		struct device_attribute *attr, char *buf)	\
{ \
	int ret = cxa2765er_read_register(dev, CXA2765ER_##name);		\
	if (ret < 0) return ret;							\
	return sprintf(buf, "%d\n", ret); \
}

#define STORE_FUNC(name)	\
static ssize_t store_reg_##name(struct device *dev, 		\
		struct device_attribute *attr, const char *buf,size_t size)	\
{ \
	ssize_t ret = -EINVAL; \
	char *after; \
	long value = simple_strtol(buf, &after, 10); \
	size_t count = after - buf; \
	if (*after && isspace(*after)) \
			count++; \
	if (count != size) return ret; \
	ret = cxa2765er_write_register(dev, CXA2765ER_##name, value);		\
	if (ret < 0) return ret;							\
	return size;  \
}

#define CXA2765ER_ATTR(name) \
STORE_FUNC(name)\
SHOW_FUNC(name)\
static DEVICE_ATTR(name, S_IRUGO | S_IWUSR, show_reg_##name, store_reg_##name);

CXA2765ER_ATTR(modeldd)
CXA2765ER_ATTR(hfmp)
CXA2765ER_ATTR(hfmf)
CXA2765ER_ATTR(emisel)
CXA2765ER_ATTR(ldcr)
CXA2765ER_ATTR(chsel)

static struct attribute *cxa2765er_attributes[] = {
	&dev_attr_modeldd.attr,
	&dev_attr_hfmp.attr,
	&dev_attr_hfmf.attr,
	&dev_attr_emisel.attr,
	&dev_attr_ldcr.attr,
	&dev_attr_chsel.attr,
	NULL,
};

static const struct attribute_group cxa2765er_attr_group = {
	.attrs	= cxa2765er_attributes,
};

static int setup_transfer(struct cxa2765er_data *data)
{
	struct spi_message *m;
	struct spi_transfer *x;

	m = &data->msg;
	x = &data->xfer;

	spi_message_init(m);

	x->tx_buf = &data->tx_buf[0];
	x->rx_buf = &data->rx_buf[0];
	x->len = 2;
	spi_message_add_tail(x, m);

	return 0;
}

static int __devinit cxa2765er_probe(struct spi_device *spi)
{
	struct cxa2765er_data *data;
	int err;

	spi->bits_per_word = 16;
	spi->mode = SPI_MODE_3 | SPI_CS_HIGH;
	spi->max_speed_hz = 500000;
	err = spi_setup(spi);
	if (err < 0)
		return err;

	data = kzalloc(sizeof(struct cxa2765er_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&spi->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	err = setup_transfer(data);
	if (err)
		goto err_free_data;

	data->spi = spi;
	spi_set_drvdata(spi, data);

	err = sysfs_create_group(&spi->dev.kobj, &cxa2765er_attr_group);
	if (err) {
		dev_err(&spi->dev, "failed to create attribute group\n");
		goto err_free_data;
	}

	memset (data->regvalues,0,6);
	err = cxa2765er_write_register(&data->spi->dev, CXA2765ER_chsel, (1<<7) );
	if (err) {
		dev_err(&data->spi->dev, "currently no device connected\n");
		data->is_ok = 0;
	} else
		cxa2765er_write_register(&data->spi->dev, CXA2765ER_chsel, 0 );
		data->is_ok = 1;
	return 0;

err_free_data:
	kfree(data);
	return err;
}

static int __devexit cxa2765er_remove(struct spi_device *spi)
{
	struct cxa2765er_data *data = spi_get_drvdata(spi);

	sysfs_remove_group(&spi->dev.kobj, &cxa2765er_attr_group);
	kfree(data);
	return 0;
}

static struct spi_driver cxa2765er_driver = {
	.driver		= {
		.name	= "laserdriver",
		.owner	= THIS_MODULE,
	},
	.probe		= cxa2765er_probe,
	.remove		= __devexit_p(cxa2765er_remove),
};

static int __init cxa2765er_init(void)
{
	return spi_register_driver(&cxa2765er_driver);
}
module_init(cxa2765er_init);

static void __exit cxa2765er_exit(void)
{
	spi_unregister_driver(&cxa2765er_driver);
}
module_exit(cxa2765er_exit);

MODULE_AUTHOR("Arno Euteneuer <arno.euteneuer@toptica.com>");
MODULE_DESCRIPTION("CXA2765ER Laserdiode Driver");
MODULE_LICENSE("GPL");
