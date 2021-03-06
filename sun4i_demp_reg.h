// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (c) 2020 Luc Verhaegen <libv@skynet.be>
 */

#ifndef _HAVE_SUN4I_DEMP_REG_H_
#define _HAVE_SUN4I_DEMP_REG_H_ 1

#define DEMP_REG_CONTROL		0x000
#define DEMP_REG_STATUS			0x004

#define DEMP_REG_IDMA_GLOBAL_CONTROL	0x008
#define DEMP_REG_IDMA_ADDRESS_HIGH	0x00C
#define DEMP_REG_IDMA0_ADDRESS_LOW	0x010
#define DEMP_REG_IDMA1_ADDRESS_LOW	0x014
#define DEMP_REG_IDMA2_ADDRESS_LOW	0x018
#define DEMP_REG_IDMA3_ADDRESS_LOW	0x01C
#define DEMP_REG_IDMA0_PITCH		0x020 /* bits */
#define DEMP_REG_IDMA1_PITCH		0x024 /* bits */
#define DEMP_REG_IDMA2_PITCH		0x028 /* bits */
#define DEMP_REG_IDMA3_PITCH		0x02C /* bits */
#define DEMP_REG_IDMA0_SIZE		0x030
#define DEMP_REG_IDMA1_SIZE		0x034
#define DEMP_REG_IDMA2_SIZE		0x038
#define DEMP_REG_IDMA3_SIZE		0x03C
#define DEMP_REG_IDMA0_COORD		0x040
#define DEMP_REG_IDMA1_COORD		0x044
#define DEMP_REG_IDMA2_COORD		0x048
#define DEMP_REG_IDMA3_COORD		0x04C
#define DEMP_REG_IDMA0_CONTROL		0x050
#define DEMP_REG_IDMA1_CONTROL		0x054
#define DEMP_REG_IDMA2_CONTROL		0x058
#define DEMP_REG_IDMA3_CONTROL		0x05C
#define DEMP_REG_IDMA0_FILLCOLOR	0x060
/* this one is listed but not documented, all bits work though */
#define DEMP_REG_IDMA_SORT		0x070

#define DEMP_REG_CSC0_CONTROL		0x074
#define DEMP_REG_CSC1_CONTROL		0x078

#define DEMP_REG_SCALE_CONTROL 		0x080
#define DEMP_REG_SCALE_OUT_SIZE		0x084
#define DEMP_REG_SCALE_FACTOR_H		0x088
#define DEMP_REG_SCALE_FACTOR_V		0x08C
#define DEMP_REG_SCALE_PHASE_H		0x090
#define DEMP_REG_SCALE_PHASE_V		0x094

#define DEMP_REG_ROP_CONTROL		0x0B0
#define DEMP_REG_ROP_CH3_INDEX0_CONTROL	0x0B8
#define DEMP_REG_ROP_CH3_INDEX1_CONTROL 0x0BC
#define DEMP_REG_ALPHA_COLORKEY		0x0C0
#define DEMP_REG_COLORKEY_MIN		0x0C4
#define DEMP_REG_COLORKEY_MAX		0x0C8
#define DEMP_REG_ROP_OUT_FILLCOLOR	0x0CC

#define DEMP_REG_CSC2_CONTROL		0x0D0

#define DEMP_REG_OUTPUT_CONTROL		0x0E0
#define DEMP_REG_OUTPUT_SIZE		0x0E8
#define DEMP_REG_OUTPUT_ADDRESS_HIGH	0x0EC
#define DEMP_REG_OUTPUT_ADDRESS_CH0	0x0F0
#define DEMP_REG_OUTPUT_ADDRESS_CH1	0x0F4
#define DEMP_REG_OUTPUT_ADDRESS_CH2	0x0F8
#define DEMP_REG_OUTPUT_PITCH_CH0	0x100
#define DEMP_REG_OUTPUT_PITCH_CH1	0x104
#define DEMP_REG_OUTPUT_PITCH_CH2	0x108
#define DEMP_REG_OUTPUT_ALPHA		0x120

#define DEMP_REG_INPUT_CSC_YG_GY_COEFF	0x180
#define DEMP_REG_INPUT_CSC_YG_RU_COEFF	0x184
#define DEMP_REG_INPUT_CSC_YG_BV_COEFF	0x188
#define DEMP_REG_INPUT_CSC_YG_CONSTANT	0x18C
#define DEMP_REG_INPUT_CSC_UR_GY_COEFF	0x190
#define DEMP_REG_INPUT_CSC_UR_RU_COEFF	0x194
#define DEMP_REG_INPUT_CSC_UR_BV_COEFF	0x198
#define DEMP_REG_INPUT_CSC_UR_CONSTANT	0x19C
#define DEMP_REG_INPUT_CSC_VB_GY_COEFF	0x1A0
#define DEMP_REG_INPUT_CSC_VB_RU_COEFF	0x1A4
#define DEMP_REG_INPUT_CSC_VB_BV_COEFF	0x1A8
#define DEMP_REG_INPUT_CSC_VB_CONSTANT	0x1AC

#define DEMP_REG_OUTPUT_CSC_YG_GY_COEFF	0x1C0
#define DEMP_REG_OUTPUT_CSC_YG_RU_COEFF	0x1C4
#define DEMP_REG_OUTPUT_CSC_YG_BV_COEFF	0x1C8
#define DEMP_REG_OUTPUT_CSC_YG_CONSTANT	0x1CC
#define DEMP_REG_OUTPUT_CSC_UR_GY_COEFF	0x1D0
#define DEMP_REG_OUTPUT_CSC_UR_RU_COEFF	0x1D4
#define DEMP_REG_OUTPUT_CSC_UR_BV_COEFF	0x1D8
#define DEMP_REG_OUTPUT_CSC_UR_CONSTANT	0x1DC
#define DEMP_REG_OUTPUT_CSC_VB_GY_COEFF	0x1E0
#define DEMP_REG_OUTPUT_CSC_VB_RU_COEFF	0x1E4
#define DEMP_REG_OUTPUT_CSC_VB_BV_COEFF	0x1E8
#define DEMP_REG_OUTPUT_CSC_VB_CONSTANT	0x1EC

#define DEMP_REG_SCALING_COEFF_HORIZONTAL(x) (0x200 + 4 * (x))
#define DEMP_REG_SCALING_COEFF_VERTICAL(x) (0x280 + 4 * (x))

#define DEMP_REG_PALETTE(x) (0x400 + 4 * (x))

#endif /* _HAVE_SUN4I_DEMP_REG_H_ */
