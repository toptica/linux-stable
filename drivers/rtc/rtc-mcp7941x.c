/*
 * rtc class driver for the microchip MCP7941x chip
 *
 * SRAM, EEPROM, calibration as well as alarms are not supported!
 *
 * Author: Arno Euteneuer <arno.euteneuer@toptica.com>
 *
 * based on mcp7941x driver by Dale Farnsworth <dale@farnsworth.org>
 *
 * 2007 (c) TOPTICA Photonics AG  This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>

#define DRV_VERSION "0.1"

#define MPC7941x_RTC_I2C_ADDR	0x6F

/*
 * register indices
 */

#define MCP7941x_REG_SC			0	/* seconds      00-59 */
#define MCP7941x_REG_MN			1	/* minutes      00-59 */
#define MCP7941x_REG_HR			2	/* hours        00-23 */
#define MCP7941x_REG_DW			3	/* day of week   1-7  */
#define MCP7941x_REG_DT			4	/* day of month 00-31 */
#define MCP7941x_REG_MO			5	/* month        01-12 */
#define MCP7941x_REG_YR			6	/* year         00-99 */
#define MCP7941x_REG_CT			7	/* control */

#define MCP7941x_REG_LEN			8
#define MCP7941x_BURST_LEN		8	/* can burst r/w first 8 regs */

#define MCP7941x_REG_CT_WP		(1 << 7)	/* Write Protect */

/*
 * register bit masks
 */
#define MCP7941x_REG_SC_TIME_MASK	0x7f
#define MCP7941x_REG_SC_START_MASK	0x80

#define MCP7941x_REG_MN_TIME_MASK	0x7f

#define MCP7941x_REG_HR_TIME_MASK	0x3f
#define MCP7941x_REG_HR_AMPM_MASK	0x40

#define MCP7941x_REG_DW_TIME_MASK	0x07
#define MCP7941x_REG_DW_OSCON_MASK	0x20
#define MCP7941x_REG_DW_VBAT_MASK	0x10
#define MCP7941x_REG_DW_VBATEN_MASK	0x08

#define MCP7941x_REG_DT_TIME_MASK	0x3f

#define MCP7941x_REG_MO_TIME_MASK	0x1f
#define MCP7941x_REG_MO_LP_MASK	0x20

#define MCP7941x_REG_YR_TIME_MASK	0xff

#define MCP7941x_REG_CT_OUT	0x80
#define MCP7941x_REG_CT_SQWE	0x40
#define MCP7941x_REG_CT_1HZ	0x00
#define MCP7941x_REG_CT_4KHZ	0x01
#define MCP7941x_REG_CT_8KHZ	0x02
#define MCP7941x_REG_CT_32KHZ	0x03
/*
 * register read/write commands
 */
#define MCP7941x_REG_CONTROL_WRITE	0x8e
#define MCP7941x_REG_CENTURY_WRITE	0x92
#define MCP7941x_REG_CENTURY_READ	0x93
#define MCP7941x_REG_RESERVED_READ	0x96
#define MCP7941x_REG_BURST_WRITE		0xbe
#define MCP7941x_REG_BURST_READ		0xbf

#define MCP7941x_IDLE_TIME_AFTER_WRITE	3	/* specification says 2.5 mS */

static struct i2c_driver mcp7941x_driver;

static int mcp7941x_i2c_read_regs(struct i2c_client *client, u8 *buf)
{
	u8 dt_addr[1] = { MCP7941x_REG_SC };

	struct i2c_msg msgs[] = {
	{
		.addr	= client->addr,
		.flags	= 0,
		.len	= 1,
		.buf	= dt_addr,
	},
	{
		.addr	= client->addr,
		.flags	= I2C_M_RD,
		.len	= MCP7941x_REG_LEN - MCP7941x_REG_SC,
		.buf	= buf,
	},
	};
	int rc;

	rc = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (rc != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: register read failed\n", __func__);
		return -EIO;
	}
	return 0;
}

static int mcp7941x_i2c_write_regs(struct i2c_client *client, u8 const *buf)
{
	struct i2c_msg msgs[1] = {
		{
		 .addr = MPC7941x_RTC_I2C_ADDR, /* client->addr, */
		 .flags = 0,	/* write */
		 .len = MCP7941x_REG_LEN + 1,
		 .buf = buf}
	};

	int rc;
	rc = i2c_transfer(client->adapter, msgs,
			  ARRAY_SIZE(msgs));
	if (rc != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: register write failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mcp7941x_i2c_read_time(struct i2c_client *client, struct rtc_time *tm)
{
	int rc;
	u8 regs[MCP7941x_REG_LEN];

	rc = mcp7941x_i2c_read_regs(client, regs);
	if (rc < 0)
		return rc;

	dev_dbg(&client->dev, "%s: regs %02x %02x %02x %02x %02x %02x %02x %02x \n",
				__func__, regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7]);
	tm->tm_sec = bcd2bin(regs[MCP7941x_REG_SC] & MCP7941x_REG_SC_TIME_MASK);
	tm->tm_min = bcd2bin(regs[MCP7941x_REG_MN] & MCP7941x_REG_MN_TIME_MASK);
	tm->tm_hour = bcd2bin(regs[MCP7941x_REG_HR] & MCP7941x_REG_HR_TIME_MASK);
	tm->tm_mday = bcd2bin(regs[MCP7941x_REG_DT] & MCP7941x_REG_DT_TIME_MASK);
	tm->tm_mon = bcd2bin(regs[MCP7941x_REG_MO] & MCP7941x_REG_MO_TIME_MASK) - 1;
	tm->tm_year = bcd2bin(regs[MCP7941x_REG_YR] & MCP7941x_REG_YR_TIME_MASK) +
		      2000 - 1900;
	tm->tm_wday = bcd2bin(regs[MCP7941x_REG_DW] & MCP7941x_REG_DW_TIME_MASK);

	return 0;
}


static int
mcp7941x_i2c_set_time(struct i2c_client *client, struct rtc_time const *tm)
{
	u8 regs[MCP7941x_REG_LEN + 1];
	u8 secs;
	int rc;
	/* we first read the time in order to minimize the likelyhood
	 * of register increments (see datasheet 4.1, 2nd item)
	 */
	rc = mcp7941x_i2c_read_regs(client, regs);
	if (rc>=0) {
		secs = bcd2bin(regs[MCP7941x_REG_SC]) & MCP7941x_REG_SC_TIME_MASK;
		if (secs >= 58)
			mdelay(2500);
	}

	regs[0] = MCP7941x_REG_SC;
	regs[MCP7941x_REG_SC + 1] = bin2bcd(tm->tm_sec) | MCP7941x_REG_SC_START_MASK;
	regs[MCP7941x_REG_MN + 1] = bin2bcd(tm->tm_min);
	regs[MCP7941x_REG_HR + 1] = bin2bcd(tm->tm_hour);
	regs[MCP7941x_REG_DT + 1] = bin2bcd(tm->tm_mday);
	regs[MCP7941x_REG_MO + 1] = bin2bcd(tm->tm_mon + 1);
	regs[MCP7941x_REG_DW + 1] = bin2bcd(tm->tm_wday) | MCP7941x_REG_DW_VBATEN_MASK;
	regs[MCP7941x_REG_YR + 1] = bin2bcd(tm->tm_year % 100);
	regs[MCP7941x_REG_CT + 1] = MCP7941x_REG_CT_SQWE+ MCP7941x_REG_CT_1HZ;

	rc = mcp7941x_i2c_write_regs(client, regs);
	if (rc < 0)
		return rc;

	return 0;
}

static int mcp7941x_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return mcp7941x_i2c_read_time(to_i2c_client(dev), tm);
}

static int mcp7941x_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return mcp7941x_i2c_set_time(to_i2c_client(dev), tm);
}

static int mcp7941x_remove(struct i2c_client *client)
{
	struct rtc_device *rtc = i2c_get_clientdata(client);

	if (rtc)
		rtc_device_unregister(rtc);

	return 0;
}

static const struct rtc_class_ops mcp7941x_rtc_ops = {
	.read_time = mcp7941x_rtc_read_time,
	.set_time = mcp7941x_rtc_set_time,
};

static int
mcp7941x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rtc_device *rtc;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	dev_info(&client->dev, "chip found, driver version " DRV_VERSION "\n");

	/* dev_info(&client->dev, "chip address wants to be 0x%2x but will be 0x%2x\n",
			client->addr, MPC7941x_RTC_I2C_ADDR); */

	client->addr = MPC7941x_RTC_I2C_ADDR;
	rtc = rtc_device_register(mcp7941x_driver.driver.name,
				  &client->dev, &mcp7941x_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	i2c_set_clientdata(client, rtc);

	return 0;
}

static struct i2c_device_id mcp7941x_id[] = {
	{ "mcp7941x", 0 },
	{ }
};

static struct i2c_driver mcp7941x_driver = {
	.driver = {
		   .name = "rtc-mcp7941x",
		   },
	.probe = mcp7941x_probe,
	.remove = mcp7941x_remove,
	.id_table = mcp7941x_id,
};

static int __init mcp7941x_init(void)
{
	return i2c_add_driver(&mcp7941x_driver);
}

static void __exit mcp7941x_exit(void)
{
	i2c_del_driver(&mcp7941x_driver);
}

MODULE_DESCRIPTION("Microchip MCP7941x RTC driver");
MODULE_AUTHOR("Arno Euteneuer <arno.euteneuer@toptica.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(mcp7941x_init);
module_exit(mcp7941x_exit);
