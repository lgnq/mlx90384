/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-15     lgnq         the first version
 */

#ifndef MLX90382_H_
#define MLX90382_H_

#include <rtthread.h>
#include <stdint.h>

union mlx90382_config_reg
{
    rt_uint16_t word_val;

    struct
    {
        rt_uint8_t sensing_mode     : 3;    //BIT0-BIT2
        rt_uint8_t gpio_if          : 2;
        rt_uint8_t abi_if           : 1;
        rt_uint8_t gpio_cfg         : 5;
        rt_uint8_t abi_cfg          : 5;
    };
};

/* mlx90382 config structure */
struct mlx90382_config
{
    rt_uint16_t accel_range;
    rt_uint16_t gyro_range;
};

/* mlx90382 device structure */
struct mlx90382_device
{
    rt_device_t bus;
    rt_uint16_t id;
    rt_uint8_t i2c_addr;
    struct mlx90382_config config;
};

/**
 * This function initialize the mpu6xxx device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for spi
 *
 * @return the pointer of device driver structure, RT_NULL represents initialization failed.
 */
struct mlx90382_device *mlx90382_init(const char *dev_name, rt_uint8_t param);

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void mlx90382_deinit(struct mlx90382_device *dev);

rt_err_t mlx90382_soft_reset(struct mlx90382_device *dev);
rt_err_t mlx90382_get_config(struct mlx90382_device *dev, union mlx90382_config_reg *config);
rt_err_t mlx90382_set_sensing_mode(struct mlx90382_device *dev, rt_uint16_t mode);
rt_err_t mlx90382_get_lin_phase(struct mlx90382_device *dev, float *angle);
rt_err_t mlx90382_get_driftc_phase(struct mlx90382_device *dev, float *angle);
rt_err_t mlx90382_get_sc_phase(struct mlx90382_device *dev, float *angle);
rt_err_t mlx90382_set_zero_position(struct mlx90382_device *dev, rt_uint16_t position);
rt_err_t mlx90382_get_zero_position(struct mlx90382_device *dev, rt_uint16_t *position);

/**
 * This function gets the data of the temperature, unit: Centigrade
 *
 * @param dev the pointer of device driver structure
 * @param temp read data pointer
 *
 * @return the reading status, RT_EOK represents  reading the data successfully.
 */
rt_err_t mlx90382_get_temp(struct mlx90382_device *dev, float *temp);
rt_err_t mlx90382_get_speed(struct mlx90382_device *dev, rt_int16_t *speed);

rt_err_t mlx90382_get_analog_version(struct mlx90382_device *dev, rt_uint16_t *version);
rt_err_t mlx90382_get_digital_version(struct mlx90382_device *dev, rt_uint16_t *version);

#endif
