/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-01-04     lgnq         the first version
 */

#ifndef SENSOR_MELEXIS_MLX90382_H__
#define SENSOR_MELEXIS_MLX90382_H__

#include "sensor.h"
#include <mlx90382.h>

enum CMD
{
    RT_SENSOR_CTRL_USER_CMD_SOFT_RESET = 0x101,
    RT_SENSOR_CTRL_USER_CMD_INFO,
    RT_SENSOR_CTRL_USER_CMD_LIN_PHASE,
    RT_SENSOR_CTRL_USER_CMD_DRIFTC_PHASE,
    RT_SENSOR_CTRL_USER_CMD_SC_PHASE,
    RT_SENSOR_CTRL_USER_CMD_SPEED,
    RT_SENSOR_CTRL_USER_CMD_TEMP,
    RT_SENSOR_CTRL_USER_CMD_GET_ZEROPOSITION,
    RT_SENSOR_CTRL_USER_CMD_SET_ZEROPOSITION,
    RT_SENSOR_CTRL_USER_CMD_SET_SENSING_MODE,
};

int rt_hw_mlx90382_init(const char *name, struct rt_sensor_config *cfg);

#endif
