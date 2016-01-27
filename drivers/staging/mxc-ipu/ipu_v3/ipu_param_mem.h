/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __INCLUDE_IPU_PARAM_MEM_H__
#define __INCLUDE_IPU_PARAM_MEM_H__

#include <linux/types.h>
#include <linux/bitrev.h>

extern u32 *ipu_cpmem_base;

struct ipu_ch_param_word {
	uint32_t data[5];
	uint32_t res[3];
};

struct ipu_ch_param {
	struct ipu_ch_param_word word[2];
};

#define ipu_ch_param_addr(ch) (((struct ipu_ch_param *)ipu_cpmem_base) + (ch))

#define _param_word(base, w) \
	(((struct ipu_ch_param *)(base))->word[(w)].data)

#define ipu_ch_param_set_field(base, w, bit, size, v) { \
	int i = (bit) / 32; \
	int off = (bit) % 32; \
	_param_word(base, w)[i] |= (v) << off; \
	if (((bit)+(size)-1)/32 > i) { \
		_param_word(base, w)[i + 1] |= (v) >> (off ? (32 - off) : 0); \
	} \
}

#define ipu_ch_param_mod_field(base, w, bit, size, v) { \
	int i = (bit) / 32; \
	int off = (bit) % 32; \
	u32 mask = (1UL << size) - 1; \
	u32 temp = _param_word(base, w)[i]; \
	temp &= ~(mask << off); \
	_param_word(base, w)[i] = temp | (v) << off; \
	if (((bit)+(size)-1)/32 > i) { \
		temp = _param_word(base, w)[i + 1]; \
		temp &= ~(mask >> (32 - off)); \
		_param_word(base, w)[i + 1] = \
			temp | ((v) >> (off ? (32 - off) : 0)); \
	} \
}

#define ipu_ch_param_read_field(base, w, bit, size) ({ \
	u32 temp2; \
	int i = (bit) / 32; \
	int off = (bit) % 32; \
	u32 mask = (1UL << size) - 1; \
	u32 temp1 = _param_word(base, w)[i]; \
	temp1 = mask & (temp1 >> off); \
	if (((bit)+(size)-1)/32 > i) { \
		temp2 = _param_word(base, w)[i + 1]; \
		temp2 &= mask >> (off ? (32 - off) : 0); \
		temp1 |= temp2 << (off ? (32 - off) : 0); \
	} \
	temp1; \
})

static inline void _ipu_ch_params_set_packing(struct ipu_ch_param *p,
					      int red_width, int red_offset,
					      int green_width, int green_offset,
					      int blue_width, int blue_offset,
					      int alpha_width, int alpha_offset)
{
	/* Setup red width and offset */
	ipu_ch_param_set_field(p, 1, 116, 3, red_width - 1);
	ipu_ch_param_set_field(p, 1, 128, 5, red_offset);
	/* Setup green width and offset */
	ipu_ch_param_set_field(p, 1, 119, 3, green_width - 1);
	ipu_ch_param_set_field(p, 1, 133, 5, green_offset);
	/* Setup blue width and offset */
	ipu_ch_param_set_field(p, 1, 122, 3, blue_width - 1);
	ipu_ch_param_set_field(p, 1, 138, 5, blue_offset);
	/* Setup alpha width and offset */
	ipu_ch_param_set_field(p, 1, 125, 3, alpha_width - 1);
	ipu_ch_param_set_field(p, 1, 143, 5, alpha_offset);
}

static inline void _ipu_ch_param_dump(int ch)
{
	struct ipu_ch_param *p = ipu_ch_param_addr(ch);
	pr_debug("ch %d word 0 - %08X %08X %08X %08X %08X\n", ch,
		 p->word[0].data[0], p->word[0].data[1], p->word[0].data[2],
		 p->word[0].data[3], p->word[0].data[4]);
	pr_debug("ch %d word 1 - %08X %08X %08X %08X %08X\n", ch,
		 p->word[1].data[0], p->word[1].data[1], p->word[1].data[2],
		 p->word[1].data[3], p->word[1].data[4]);
	pr_debug("PFS 0x%x, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 85, 4));
	pr_debug("BPP 0x%x, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 0, 107, 3));
	pr_debug("NPB 0x%x\n",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 78, 7));

	pr_debug("FW %d, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 0, 125, 13));
	pr_debug("FH %d, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 0, 138, 12));
	pr_debug("Stride %d\n",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 102, 14));

	pr_debug("Width0 %d+1, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 116, 3));
	pr_debug("Width1 %d+1, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 119, 3));
	pr_debug("Width2 %d+1, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 122, 3));
	pr_debug("Width3 %d+1, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 125, 3));
	pr_debug("Offset0 %d, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 128, 5));
	pr_debug("Offset1 %d, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 133, 5));
	pr_debug("Offset2 %d, ",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 138, 5));
	pr_debug("Offset3 %d\n",
		 ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 143, 5));
}

static inline void _ipu_ch_param_set_burst_size(uint32_t ch,
						uint16_t burst_pixels)
{
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 78, 7,
			       burst_pixels - 1);
};

static inline int _ipu_ch_param_get_burst_size(uint32_t ch)
{
	return ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 78, 7) + 1;
};

static inline int _ipu_ch_param_get_bpp(uint32_t ch)
{
	return ipu_ch_param_read_field(ipu_ch_param_addr(ch), 0, 107, 3);
};

static inline void _ipu_ch_param_set_buffer(uint32_t ch, int bufNum,
					    dma_addr_t phyaddr)
{
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 29 * bufNum, 29,
			       phyaddr / 8);
};

static inline void _ipu_ch_param_set_rotation(uint32_t ch,
					      ipu_rotate_mode_t rot)
{
	u32 temp_rot = bitrev8(rot) >> 5;
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 0, 119, 3, temp_rot);
};

static inline void _ipu_ch_param_set_block_mode(uint32_t ch)
{
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 0, 117, 2, 1);
};

static inline void _ipu_ch_param_set_alpha_use_separate_channel(uint32_t ch,
								bool option)
{
	if (option) {
		ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 89, 1, 1);
	} else {
		ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 89, 1, 0);
	}
};

static inline void _ipu_ch_param_set_alpha_condition_read(uint32_t ch)
{
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 149, 1, 1);
};

static inline void _ipu_ch_param_set_alpha_buffer_memory(uint32_t ch)
{
	int alp_mem_idx;

	switch (ch) {
	case 14: /* PRP graphic */
		alp_mem_idx = 0;
		break;
	case 15: /* PP graphic */
		alp_mem_idx = 1;
		break;
	case 23: /* DP BG SYNC graphic */
		alp_mem_idx = 4;
		break;
	case 27: /* DP FG SYNC graphic */
		alp_mem_idx = 2;
		break;
	default:
		dev_err(g_ipu_dev, "unsupported correlated channel of local alpha channel\n");
		return;
	}

	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 90, 3, alp_mem_idx);
};

static inline void _ipu_ch_param_set_interlaced_scan(uint32_t ch)
{
	u32 stride;
	ipu_ch_param_set_field(ipu_ch_param_addr(ch), 0, 113, 1, 1);
	stride = ipu_ch_param_read_field(ipu_ch_param_addr(ch), 1, 102, 14) + 1;
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 58, 20, stride / 8);
	stride *= 2;
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 102, 14, stride - 1);
};

static inline void _ipu_ch_param_set_high_priority(uint32_t ch)
{
	ipu_ch_param_mod_field(ipu_ch_param_addr(ch), 1, 93, 2, 1);
};

static inline void _ipu_ch_params_set_alpha_width(uint32_t ch, int alpha_width)
{
	ipu_ch_param_set_field(ipu_ch_param_addr(ch), 1, 125, 3, alpha_width - 1);
}

#endif
