/* 
 * FPGA configurator and driver
 *
 * This is very specific to the laserbox platform it uses the PSC2 as gpio as
 * follows:
 * 	GPIOC_04 in  nSTATUS
 * 	GPIOD_29 in  CONF_DONE
 * 	GPIOB_26 out nCONFIG
 * 	GPIOA_19 out OE_FPGA
 *
 * Finaly it is used as memory mapped device
 * The major number is allocate dinamicaly; 
 * The minor number is the chip select number
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <mach/iomux.h>

struct laserbox_fpga {
        int cs;
        struct resource *iomem;
        void __iomem *base;
        unsigned long size;
};
//TODO: Move this to platform data!
#if 0
#define nSTATUS   (GPIO_PORTC |  4)
#define CONF_DONE (GPIO_PORTD | 29)
#define nCONFIG   (GPIO_PORTB | 26)
#define OE_FPGA   (GPIO_PORTA | 19)
#endif

#define nSTATUS   (GPIO_PORTB |  3)
#define CONF_DONE (GPIO_PORTB | 19)
#define nCONFIG   (GPIO_PORTB | 17)
#define OE_FPGA   (GPIO_PORTB | 14)

static inline int begin_config(void)
{
        int ret, i = 0;
	printk(KERN_ERR "1\n");
        ret = gpio_request(nSTATUS, "nSTATUS");
        if (ret < 0) {
                goto end;
	}
        ret = gpio_direction_input(nSTATUS);


	printk(KERN_ERR "2\n");
        ret = gpio_request(CONF_DONE, "CONF_DONE");
        if (ret < 0) {
                goto free_nSTATUS;
	}
        ret = gpio_direction_input(CONF_DONE);

	printk(KERN_ERR "3\n");
        ret = gpio_request(OE_FPGA, "OE_FPGA");
        if (ret < 0) {
                goto free_nCONFIG;
	}
        ret = gpio_direction_output(OE_FPGA, 0);

	printk(KERN_ERR "4\n");
        ret = gpio_request(nCONFIG, "nCONFIG");
        if (ret < 0) {
                goto free_CONF_DONE;
	}
        ret = gpio_direction_output(nCONFIG, 0);

        udelay(1);
        
        /* error if CONF_DONE or nSTATUS is set */
	printk(KERN_ERR "5\n");
        if (gpio_get_value(CONF_DONE) || gpio_get_value(nSTATUS)) {
                ret = -ENODEV;
                goto free_OE_FPGA;
        }

	printk(KERN_ERR "6\n");
        /* enable config */
        gpio_set_value(nCONFIG, 1);
        /* nSTATUS should go high */
        while (!gpio_get_value(nSTATUS)) {
                if (++i >= 100) {
			printk(KERN_ERR "nSTATUS did not go high!\n");
                        ret = -ENODEV;
                        goto free_OE_FPGA;
                }
                udelay(1);
        }

	printk(KERN_ERR "7\n");
        return 0;

free_nCONFIG:
        gpio_free(nCONFIG);
free_OE_FPGA:
        gpio_free(OE_FPGA);
free_CONF_DONE:
        gpio_free(CONF_DONE);
free_nSTATUS:
        gpio_free(nSTATUS);
end:        
        return ret;
}

static inline int end_config(void)
{
        int ret = 0;

        /* CONF_DONE should be high */
        if (gpio_get_value(CONF_DONE)) {
		printk(KERN_ERR"FPGA configuration successfull!\n");
                gpio_set_value(OE_FPGA, 1); /* enable OE */
	}
        else {
		printk(KERN_ERR "FPGA configuration failed!\n");
                ret = -ENODEV;
	}

        printk(KERN_INFO "CONF_DONE: %s\n", gpio_get_value(CONF_DONE)?"1":"0");

        gpio_free(nCONFIG);
        gpio_free(OE_FPGA);
        gpio_free(CONF_DONE);
        gpio_free(nSTATUS);

        return ret;
}

static int config_fpga(struct platform_device *pdev)
{
        const struct firmware *fw;
        int ret;

        void __iomem *u;
        void __iomem *l;
        void __iomem *a;

        struct resource* res1;
        void __iomem *p;
        int i;
        u32 clk_conf;

        /* get firmware */
        ret = request_firmware(&fw, "mle_ii_fpgatest.fw", &pdev->dev);
        if (ret) {
                dev_err(&pdev->dev, "error reading firmware file\n");
                return ret;
        }

        /* configure CS0 and CS5 */
        res1 = request_mem_region(MX25_X_MEMC_BASE_ADDR, MX25_X_MEMC_SIZE,
                                  "CS config");
        p = ioremap(res1->start, res1->end-res1->start+1);

        u = CSCR_U(5) - (void*)MX25_X_MEMC_BASE_ADDR_VIRT + p;
        l = CSCR_L(5) - (void*)MX25_X_MEMC_BASE_ADDR_VIRT + p;
        a = CSCR_A(5) - (void*)MX25_X_MEMC_BASE_ADDR_VIRT + p;

        iowrite32(0x00000200, u);
        iowrite32(0x00002311, l);
//        iowrite32(0x00002331, l);
        iowrite32(0x00000000, a);

        u = CSCR_U(0) - (void*)MX25_X_MEMC_BASE_ADDR_VIRT + p;
        l = CSCR_L(0) - (void*)MX25_X_MEMC_BASE_ADDR_VIRT + p;
        a = CSCR_A(0) - (void*)MX25_X_MEMC_BASE_ADDR_VIRT + p;

        iowrite32(0x00000600, u);
//        iowrite32(0x00002311, l);
        iowrite32(0x00003311, l);
        iowrite32(0x00000000, a);

        iounmap(p);
        release_resource(res1);
        kfree(res1);

        res1 = request_mem_region(CS5_BASE_ADDR, PAGE_SIZE, "CS5");
        p = ioremap(res1->start, res1->end-res1->start+1);

        ret = begin_config();
        if (ret) {
                dev_err(&pdev->dev, "initializing FPGA: no answer\n");
                goto err;
        }

        iowrite8_rep(p, fw->data, fw->size);

        /* generate another min. 3185 clocks for initialization*/
        for (i = 0; i < 3200; i++)
                iowrite8(0, p);

        /* connect the clock CLKO */
        clk_conf = __raw_readl(MX25_CCM_BASE + CCM_MCR);
        clk_conf &= ~0x7FF00000u;
        clk_conf |= 0x43200000u;
        __raw_writel(clk_conf, MX25_CCM_BASE + CCM_MCR);

        ret = end_config();
err:
        iounmap(p);
        release_resource(res1);
        kfree(res1);

        release_firmware(fw);

        return ret;
}

static int uninit_mmap_regs(struct platform_device *pdev)
{
        struct laserbox_fpga *priv = platform_get_drvdata(pdev);

        if (priv->base) {
                iounmap(priv->base);
                priv->size = 0;
        }
        if (priv->iomem) {
                release_resource(priv->iomem);
                kfree(priv->iomem);
        }

        return 0;
}

static int init_mmap_regs(struct platform_device *pdev)
{
        struct laserbox_fpga *priv = platform_get_drvdata(pdev);
        struct resource* res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        int ret = 0;

        priv->iomem = priv->base = NULL;
        priv->size = 0;

        if (res == NULL) {
                dev_err(&pdev->dev, "Cannot get fpga address\n");
                ret = -EINVAL;
                goto unmap_regs;
        }

        priv->iomem = request_mem_region(res->start, res->end - res->start + 1,
                                         pdev->name);
        if (priv->iomem == NULL) {
                dev_err(&pdev->dev, "Could not reserve memory region\n");
                ret = -EINVAL;
                goto unmap_regs;
        }
        priv->size = 32; //res->end - res->start + 1;
        priv->base = ioremap_nocache(priv->iomem->start, priv->size);
        if (priv->base == NULL) {
                dev_err(&pdev->dev, "Failed to ioremap mem region\n");
                ret = -EIO;
                goto unmap_regs;
        }
        return 0;

unmap_regs:
        uninit_mmap_regs(pdev);
        return ret;
}

static struct laserbox_fpga *kobj_to_laserbox_fpga(struct kobject *kobj)
{
        return dev_get_drvdata(container_of(kobj, struct device, kobj));
}

static ssize_t laserbox_fpga_read(struct kobject *kobj,
                                  struct bin_attribute *attr,
                                  char *buf, loff_t pos, size_t size)
{
        struct laserbox_fpga *priv = kobj_to_laserbox_fpga(kobj);
        
        if (pos > priv->size)
                return 0;
        if (pos + size > priv->size)
                size = priv->size - pos;
        memcpy(buf, priv->base+pos, size);
        return size;
}

static ssize_t laserbox_fpga_write(struct kobject *kobj, 
                                   struct bin_attribute *attr,
                                   char *buf, loff_t pos, size_t size)
{
        struct laserbox_fpga *priv = kobj_to_laserbox_fpga(kobj);
        
        if (pos > priv->size)
                return 0;
        if (pos + size > priv->size)
                size = priv->size - pos;
        memcpy(priv->base+pos, buf, size);
        return size;
}

static int laserbox_fpga_mmap(struct kobject *kobj, struct bin_attribute *attr,
		    struct vm_area_struct *vma)
{
        struct laserbox_fpga *priv = kobj_to_laserbox_fpga(kobj);
        unsigned long size = vma->vm_end - vma->vm_start;
        unsigned long pfn = (priv->iomem->start>>PAGE_SHIFT) + vma->vm_pgoff;

        if (size > PAGE_ALIGN(priv->size))
                return -EINVAL;

        return io_remap_pfn_range(vma, vma->vm_start, pfn, size, 
                                  pgprot_noncached(PAGE_SHARED));
}

static struct bin_attribute laserbox_fpga_attr = {
        .attr = {
                .name = "fpga",
                .mode = S_IRUGO | S_IWUSR,
                .owner = THIS_MODULE,
        },
        .read = laserbox_fpga_read,
        .write = laserbox_fpga_write,
        .mmap = laserbox_fpga_mmap,
};

static int laserbox_fpga_probe(struct platform_device *pdev)
{
        struct laserbox_fpga *priv;
        int ret;

        priv = (struct laserbox_fpga *)kzalloc(sizeof *priv, GFP_KERNEL);
        if (!priv)
                return -ENOMEM;
        platform_set_drvdata(pdev, priv);

        //ret = config_fpga(pdev);
	ret = 0;
        if (ret)
                goto err;

        ret = init_mmap_regs(pdev);
        if (ret)
                goto err;

        /* create device file*/
        laserbox_fpga_attr.size = priv->size;
        ret = sysfs_create_bin_file(&pdev->dev.kobj, &laserbox_fpga_attr);
        if (ret)
                goto err;

        return 0;
err:
        uninit_mmap_regs(pdev);
        platform_set_drvdata(pdev, NULL);
        kfree(priv);
        return ret;
}

static int __exit laserbox_fpga_remove(struct platform_device *pdev)
{
        struct laserbox_fpga *priv = platform_get_drvdata(pdev);
        if (priv) {
                sysfs_remove_bin_file(&pdev->dev.kobj, &laserbox_fpga_attr);
                uninit_mmap_regs(pdev);
                kfree(priv);
                platform_set_drvdata(pdev, NULL);
        }
        return 0;
}

static struct platform_driver laserbox_fpga_driver = {
        .probe     = laserbox_fpga_probe,
        .remove    = __exit_p(laserbox_fpga_remove),
        .driver    = {
                .name  = "laserbox-fpga",
                .owner = THIS_MODULE,
        },
};

/* don't need to remove anything the platform device struct is static */
static void laserbox_fpga_release(struct device *dev) {}

static struct platform_device husky_fpga = {
        .name = "laserbox-fpga",
        .id = -1,
        .num_resources = 1,
        .resource = (struct resource[]) {
                {
                        .flags = IORESOURCE_MEM,
                        .start = CS0_BASE_ADDR,
                        .end = CS0_BASE_ADDR + PAGE_SIZE - 1,
                },
        },
        .dev = {
                .release = laserbox_fpga_release,
        }
};

static int __init laserbox_fpga_init(void)
{
        int res;
        res = platform_driver_register(&laserbox_fpga_driver);
        if (res == 0) {
                res = platform_device_register(&husky_fpga);
        }
        return res;
}

static void __exit laserbox_fpga_exit(void)
{
        platform_device_unregister(&husky_fpga);
        platform_driver_unregister(&laserbox_fpga_driver);
}

module_init(laserbox_fpga_init);
module_exit(laserbox_fpga_exit);

MODULE_AUTHOR("Dragos Carp <dragos.carp@toptica.com");
MODULE_DESCRIPTION("Laserbox FPGA access driver");
MODULE_LICENSE("GPL");
