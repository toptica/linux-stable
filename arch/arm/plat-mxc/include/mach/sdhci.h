#ifndef __MACH_SDHCI_H
#define __MACH_SDHCI_H

#include <linux/mmc/host.h>

struct device;

struct mxc_sdhci_platform_data {
	/* Return values for the get_ro callback should be:
	 *   0 for a read/write card
	 *   1 for a read-only card
	 *   -ENOSYS when not supported (equal to NULL callback)
	 *   or a negative errno value when something bad happened
	 */
	int (*get_ro)(struct device *);

	/* board specific hook to (de)initialize the SD slot.
	 * The board code can call 'handler' on a card detection
	 * change giving data as argument.
	 */
	int (*init)(struct device *dev, irq_handler_t handler, void *data);
	void (*exit)(struct device *dev, void *data);

	/* delay in ms between card detect IRQ and scanning for
	 * card insertion/removal
	 */
	int detect_delay;

	/* available voltages. If not given, assume
	 * MMC_VDD_32_33 | MMC_VDD_33_34
	 */
	unsigned int ocr_avail;
	unsigned int caps;
	unsigned int min_clk;
	unsigned int max_clk;
	unsigned int mxc_quirks;

	/* adjust slot voltage */
	int (*setpower)(struct device *dev, unsigned int vdd);
#if 1
	// FIXME: get rid of this
	int (*status)(struct device *dev);
#endif
	int (*suspend)(struct device *dev);
	int (*resume)(struct device *dev);
};

#endif /* __MACH_SDHCI_H */
