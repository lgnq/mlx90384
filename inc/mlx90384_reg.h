/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-15     lgnq         the first version
 */

#ifndef __MLX90384_REG_H__
#define __MLX90384_REG_H__

#define MLX90384_NVRAM_BASE_ADDR            0x100
#define MLX90384_CONFIG_REG                 (MLX90384_NVRAM_BASE_ADDR + 0x0)

#define MLX90382_SOFT_RESET                 0x004
#define MLX90384_TEMP                       0x03C
#define MLX90384_LIN_PHASE                  0x03E   //Angular value after linearization, resolution 360/216 deg; Writable only with LOCK_PHASE = 1; Before delay compensation
#define MLX90384_SPEED                      0x040
#define MLX90384_DRIFTC_PHASE               0x046   //Angular value after delay compensation and zero-point offset correction, resolution 360/0x10000 deg
#define MLX90384_SC_PHASE                   0x048   //Position value after signal conditioning
#define MLX90382_GC_I                       0x04A   //Gain compensated I component
#define MLX90382_GC_Q                       0x04E   //Gain compensated Q component

#define MLX90384_PHASE_OFS                  0x120   //Phase/Angle offset before signal conditioning, resolution 360/216 deg (signed 2th-complement)

#define MLX90384_ANA_VERSION                0x0EE
#define MLX90384_DIG_VERSION                0x0F0

#endif
