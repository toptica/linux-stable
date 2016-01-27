#ifndef __SDMA_H
#define __SDMA_H

extern struct clk *mxc_sdma_clk;
extern void __iomem *sdma_base_addr;

/*
 * SDMA buffers pool initialization function
 */
extern void init_sdma_pool(void);


#endif /* __SDMA_H */
