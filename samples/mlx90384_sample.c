/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-15     lgnq         the first version
 */

#include <rtthread.h>
#include <board.h>
#include <mlx90382.h>

/* Default configuration, please change according to the actual situation, support SPI device name */
#define MLX90382_DEVICE_NAME  "spi10"

static int mlx90382_test(void)
{
    struct mlx90382_device *dev;
    float angle;

    /* Initialize MLX90382, The parameter is RT_NULL, means auto probing for SPI*/
    dev = mlx90382_init(MLX90382_DEVICE_NAME, RT_NULL);

    if (dev == RT_NULL)
    {
        rt_kprintf("mlx90382 init failed\n");
        return -1;
    }

    for (int i = 0; i < 5; i++)
    {
        mlx90382_get_lin_phase(dev, &angle);

        rt_thread_mdelay(100);
    }

    mlx90382_deinit(dev);

    return 0;
}
MSH_CMD_EXPORT(mlx90382_test, mlx90382 sensor test function);
