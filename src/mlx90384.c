/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-15     lgnq         the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include <string.h>
#include <stdlib.h>

#define DBG_TAG "mlx90384"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <mlx90384.h>
#include "mlx90384_reg.h"

#define MLX90384_SPI_MAX_SPEED (10000 * 1000)

static rt_err_t mlx90384_register_write(struct mlx90384_device *dev, rt_uint16_t reg, rt_uint16_t data)
{
    rt_int8_t res = 0;

    rt_uint8_t tmp[2];
    rt_uint8_t dat[2];

    if (dev->bus->type == RT_Device_Class_SPIDevice)
    {
        tmp[0] = CMD_FR | (reg>>9);
        tmp[1] = (reg>>1) & 0xFF;

        dat[0] = (data&0xFF00)>>8;
        dat[1] = (data&0xFF);

        res = rt_spi_send_then_send((struct rt_spi_device *)dev->bus, tmp, 2, dat, 2);
    }

    return res;
}

static rt_err_t mlx90384_register_read(struct mlx90384_device *dev, rt_uint16_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    rt_int8_t res = 0;

    rt_uint8_t cmd  = CMD_RR | (reg>>9);
    rt_uint8_t addr = (reg>>1) & 0xFF;

    if (dev->bus->type == RT_Device_Class_SPIDevice)
    {
        struct rt_spi_message msg1, msg2;

        msg1.send_buf   = &cmd;
        msg1.recv_buf   = RT_NULL;
        msg1.length     = 1;
        msg1.cs_take    = 1;
        msg1.cs_release = 0;
        msg1.next       = &msg2;

        msg2.send_buf   = &addr;
        msg2.recv_buf   = buf;
        msg2.length     = len;
        msg2.cs_take    = 0;
        msg2.cs_release = 1;
        msg2.next       = RT_NULL;

        rt_spi_transfer_message((struct rt_spi_device *)dev->bus, &msg1);
    }

    return res;
}

static rt_err_t mlx90384_frame_read(struct mlx90384_device *dev, rt_uint16_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    rt_int8_t res = 0;

    rt_uint8_t cmd  = CMD_FR | (reg>>9);
    rt_uint8_t addr = (reg>>1) & 0xFF;

    if (dev->bus->type == RT_Device_Class_SPIDevice)
    {
        struct rt_spi_message msg1, msg2;

        msg1.send_buf   = &cmd;
        msg1.recv_buf   = RT_NULL;
        msg1.length     = 1;
        msg1.cs_take    = 1;
        msg1.cs_release = 0;
        msg1.next       = &msg2;

        msg2.send_buf   = &addr;
        msg2.recv_buf   = buf;
        msg2.length     = len;
        msg2.cs_take    = 0;
        msg2.cs_release = 1;
        msg2.next       = RT_NULL;

        rt_spi_transfer_message((struct rt_spi_device *)dev->bus, &msg1);
    }

    return res;
}

static rt_err_t mlx90384_super_frame_read(struct mlx90384_device *dev, rt_uint16_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    rt_int8_t res = 0;

    rt_uint8_t cmd  = CMD_SFR | (reg>>9);
    rt_uint8_t addr = (reg>>1) & 0xFF;

    if (dev->bus->type == RT_Device_Class_SPIDevice)
    {
        struct rt_spi_message msg1, msg2;

        msg1.send_buf   = &cmd;
        msg1.recv_buf   = RT_NULL;
        msg1.length     = 1;
        msg1.cs_take    = 1;
        msg1.cs_release = 0;
        msg1.next       = &msg2;

        msg2.send_buf   = &addr;
        msg2.recv_buf   = buf;
        msg2.length     = len;
        msg2.cs_take    = 0;
        msg2.cs_release = 1;
        msg2.next       = RT_NULL;

        rt_spi_transfer_message((struct rt_spi_device *)dev->bus, &msg1);
    }

    return res;
}

rt_err_t mlx90384_soft_reset(struct mlx90384_device *dev)
{
    rt_err_t res=0;

    res = mlx90384_register_write(dev, MLX90384_SOFT_RESET, 1);

    return res;
}

rt_err_t mlx90384_set_gpio_if(struct mlx90384_device *dev, rt_uint8_t val)
{
    rt_err_t res=0;
    rt_uint8_t buf[4];

    union mlx90384_config_reg reg;

    mlx90384_register_read(dev, MLX90384_CONFIG_REG, 4, buf);

    reg.word_val = (buf[2]<<8) + buf[3];

    reg.gpio_if = val;

    res = mlx90384_register_write(dev, MLX90384_CONFIG_REG, reg.word_val);

    return res;
}

rt_err_t mlx90384_set_current_zero_angle(struct mlx90384_device *dev)
{
    rt_err_t res=0;
    float angle;
    rt_uint8_t buf[4];

    res = mlx90384_register_read(dev, MLX90384_LIN_PHASE, 4, buf);
    if (res != RT_EOK)
    {
        return res;
    }

    angle = ((buf[2] << 8) + buf[3]);
    angle = angle * 360 / 0x10000;

    res = mlx90384_register_write(dev, MLX90384_PHASE_OFS, angle);

    return res;
}

rt_err_t mlx90384_set_zero_position(struct mlx90384_device *dev, rt_uint16_t position)
{
    rt_err_t res = 0;

    res = mlx90384_register_write(dev, MLX90384_PHASE_OFS, position);

    if (res != RT_EOK)
    {
        rt_kprintf("mlx90384_set_zero_position is failed\r\n");
    }

    return res;
}

rt_err_t mlx90384_set_sensing_mode(struct mlx90384_device *dev, rt_uint16_t mode)
{
    rt_err_t res=0;
    rt_uint8_t buf[4];

    union mlx90384_config_reg reg;

    mlx90384_register_read(dev, MLX90384_CONFIG_REG, 4, buf);

    reg.word_val = (buf[2]<<8) + buf[3];

    reg.sensing_mode = mode;

    res = mlx90384_register_write(dev, MLX90384_CONFIG_REG, reg.word_val);

    return res;
}

rt_err_t mlx90384_get_zero_position(struct mlx90384_device *dev, rt_uint16_t *position)
{
    rt_err_t res = 0;
    rt_uint8_t buf[4];

    res = mlx90384_register_read(dev, MLX90384_PHASE_OFS, 4, buf);
    if (res != RT_EOK)
    {
        rt_kprintf("mlx90384_get_zero_position is failed\r\n");
        return res;
    }

    *position = (buf[2]<<8) + buf[3];

    return RT_EOK;
}

rt_err_t mlx90384_get_config(struct mlx90384_device *dev, union mlx90384_config_reg *config)
{
    rt_err_t res = 0;
    rt_uint8_t buf[4];

    res = mlx90384_register_read(dev, MLX90384_CONFIG_REG, 4, buf);
    if (res != RT_EOK)
    {
        return res;
    }

    config->word_val = ((buf[2] << 8) + buf[3]);

    return RT_EOK;
}

rt_err_t mlx90384_get_lin_phase(struct mlx90384_device *dev, float *angle)
{
    rt_err_t res = 0;
    rt_uint8_t buf[4];

    res = mlx90384_register_read(dev, MLX90384_LIN_PHASE, 4, buf);
    if (res != RT_EOK)
    {
        return res;
    }

    *angle = ((buf[2] << 8) + buf[3]);
    *angle = *angle * 360 / 0x10000;

    //Read Angular value after linearization
//    LOG_W("[0x3E]: 0x%x 0x%x 0x%x 0x%x", buf[0], buf[1], buf[2], buf[3]);
//    LOG_W("Angular value = %d.%d", (rt_uint16_t)*angle, (*angle-((rt_uint16_t)*angle))*100);

    return RT_EOK;
}

rt_err_t mlx90384_get_driftc_phase(struct mlx90384_device *dev, float *angle)
{
    rt_err_t res = 0;
    rt_uint8_t buf[4];

    res = mlx90384_register_read(dev, MLX90384_DRIFTC_PHASE, 4, buf);
    if (res != RT_EOK)
    {
        return res;
    }

    *angle = ((buf[2] << 8) + buf[3]);
    *angle = *angle * 360 / 0x10000;

    return RT_EOK;
}

rt_err_t mlx90384_get_sc_phase(struct mlx90384_device *dev, float *pos)
{
    rt_err_t res = 0;
    rt_uint8_t buf[4];

    res = mlx90384_register_read(dev, MLX90384_SC_PHASE, 4, buf);
    if (res != RT_EOK)
    {
        return res;
    }

    *pos = ((buf[2] << 8) + buf[3]);
    *pos = *pos * 360 / 0x10000;

    return RT_EOK;
}

/**
 * This function gets the data of the temperature, unit: Centigrade
 *
 * @param dev the pointer of device driver structure
 * @param temp read data pointer
 *
 * @return the reading status, RT_EOK represents  reading the data successfully.
 */
rt_err_t mlx90384_get_temp(struct mlx90384_device *dev, float *temp)
{
    rt_err_t res = 0;
    rt_uint8_t buf[4];

    res = mlx90384_register_read(dev, MLX90384_TEMP, 4, buf);
    if (res != RT_EOK)
    {
        return res;
    }

    *temp = (buf[2]<<8) + buf[3];

    *temp = *temp/8 + 200 - 273.15;

    return RT_EOK;
}

rt_err_t mlx90384_get_speed(struct mlx90384_device *dev, rt_int16_t *speed)
{
    rt_err_t res = 0;
    rt_uint8_t buf[4];

    res = mlx90384_register_read(dev, MLX90384_SPEED, 4, buf);
    if (res != RT_EOK)
    {
        return res;
    }

    *speed = (buf[2]<<8) + buf[3];

    *speed = (*speed)*11;

    return RT_EOK;
}

rt_err_t mlx90384_get_analog_version(struct mlx90384_device *dev, rt_uint16_t *version)
{
    rt_err_t res = 0;
    rt_uint8_t buf[4];

    res = mlx90384_register_read(dev, MLX90384_ANA_VERSION, 4, buf);
    if (res != RT_EOK)
    {
        return res;
    }

    *version = (buf[2]<<8) + buf[3];

    return RT_EOK;
}

rt_err_t mlx90384_get_digital_version(struct mlx90384_device *dev, rt_uint16_t *version)
{
    rt_err_t res = 0;
    rt_uint8_t buf[4];

    res = mlx90384_register_read(dev, MLX90384_DIG_VERSION, 4, buf);
    if (res != RT_EOK)
    {
        return res;
    }

    *version = (buf[2]<<8) + buf[3];

    return RT_EOK;
}

rt_err_t mlx90384_set_spi_faddr01(struct mlx90384_device *dev, rt_uint16_t faddr0, rt_uint16_t faddr1)
{
    rt_err_t res = 0;
    rt_uint16_t faddr01 = (faddr1 << 8) | faddr0;

    res = mlx90384_register_write(dev, MLX90384_SPI_FADDR0, faddr01);

    return res;
}

rt_err_t mlx90384_set_spi_faddr23(struct mlx90384_device *dev, rt_uint16_t faddr2, rt_uint16_t faddr3)
{
    rt_err_t res = 0;
    rt_uint16_t faddr23 = (faddr3 << 8) | faddr2;

    res = mlx90384_register_write(dev, MLX90384_SPI_FADDR2, faddr23);
    
    return res;
}

/**
 * This function initialize the mlx90384 device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for SPI
 *
 * @return the pointer of device driver structure, RT_NULL represents  initialization failed.
 */
struct mlx90384_device *mlx90384_init(const char *dev_name, rt_uint8_t param)
{
    struct mlx90384_device *dev = RT_NULL;
    rt_uint8_t reg[4];
    rt_uint8_t res = RT_EOK;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct mlx90384_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for mlx90384 device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("Can't find device:'%s'", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_SPIDevice)
    {
        struct rt_spi_configuration cfg;

        cfg.data_width = 8;
        cfg.mode       = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;  //RT_SPI_MODE_1,2 is not work well
        cfg.max_hz     = MLX90384_SPI_MAX_SPEED; /* Set SPI max speed */

        rt_spi_configure((struct rt_spi_device *)dev->bus, &cfg);
    }
    else
    {
        LOG_E("Unsupported device:'%s' %d!", dev_name, dev->bus->type);
        goto __exit;
    }

    // set GPIO_IF as SPI bus mode
    mlx90384_set_gpio_if(dev, 2);

    if (mlx90384_register_read(dev, MLX90384_CONFIG_REG, 4, reg) != RT_EOK)
    {
        LOG_E("Failed to read device id!");
        goto __exit;
    }
    else
    {
        LOG_W("Unknown device id: 0x%x 0x%x 0x%x 0x%x\n", reg[0], reg[1], reg[2], reg[3]);
    }

#if 1
    if (mlx90384_register_read(dev, 0xEE, 4, reg) == RT_EOK)
    {
        LOG_W("REG[0xEE]: 0x%x 0x%x 0x%x 0x%x, Analog Version is 0x%x Digital Version(8LSB) is 0x%x", reg[0], reg[1], reg[2], reg[3], reg[2], reg[3]);

        LOG_W("chipversion:%x", reg[2]);
    }

    if (mlx90384_register_read(dev, 0xF0, 4, reg) == RT_EOK)
    {
        LOG_W("REG[0xF0]: 0x%x 0x%x 0x%x 0x%x, Digital Version(16MSB) is 0x%02x%02x", reg[0], reg[1], reg[2], reg[3], reg[2], reg[3]);
    }

    if (mlx90384_register_write(dev, 0x13E, 0xBEEF) != RT_EOK)
    {
        LOG_E("Failed to write device register!");
        goto __exit;
    }

    if (mlx90384_register_read(dev, 0x13E, 4, reg) == RT_EOK)
    {
        LOG_W("REG[0x13E]: 0x%x 0x%x 0x%x 0x%x", reg[0], reg[1], reg[2], reg[3]);
    }

    if (mlx90384_set_zero_position(dev, 20) != RT_EOK)
    {
        LOG_E("Failed to set zero angle!");
        goto __exit;
    }
    else
    {
        LOG_E("set zero angle = 20!");
    }
#endif

    if (res == RT_EOK)
    {
        LOG_I("Device init succeed!");
    }
    else
    {
        LOG_W("Error in device initialization!");
    }
    return dev;

__exit:
    if (dev != RT_NULL)
    {
        rt_free(dev);
    }
    return RT_NULL;
}

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void mlx90384_deinit(struct mlx90384_device *dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}

static void mlx90384(int argc, char **argv)
{
    static struct mlx90384_device *dev = RT_NULL;

    /* If the number of arguments less than 2 */
    if (argc < 2)
    {
        rt_kprintf("\n");
        rt_kprintf("mlx90384 [OPTION] [PARAM]\n");
        rt_kprintf("         probe <dev_name>      Probe mlx90384 by given name, default is spi10\n");
        rt_kprintf("         angle [num]           read angle [num] times mlx90384\n");
        rt_kprintf("                               num default 5\n");
        rt_kprintf("         temp                  read temp mlx90384\n");

        return ;
    }
    else if (!strcmp(argv[1], "probe"))
    {
        if (dev)
        {
            mlx90384_deinit(dev);
        }

        if (argc == 2)
        {
            dev = mlx90384_init("spi10", RT_NULL);
        }
        else
            dev = mlx90384_init(argv[2], RT_NULL);
    }
    else if (!strcmp(argv[1], "angle"))
    {
        float angle_1, angle_2, p, t, speed;
        uint16_t num = 5;

        if (dev == RT_NULL)
        {
            rt_kprintf("Please probe mlx90384 first!\n");
            return ;
        }

        if (argc == 3)
        {
            num = atoi(argv[2]);
        }

        while (num --)
        {
            mlx90384_get_lin_phase(dev, &angle_1);
            mlx90384_get_driftc_phase(dev, &angle_2);
            mlx90384_get_sc_phase(dev, &p);
            mlx90384_get_temp(dev, &t);
            mlx90384_get_speed(dev, &speed);
            rt_kprintf("data:%d.%d,%d.%d,%d.%d,%d.%d,%d.%d\n", (int)(angle_1 * 100) / 100, (int)(angle_1 * 100) % 100,
                                                               (int)(angle_2 * 100) / 100, (int)(angle_2 * 100) % 100,
                                                               (int)(p * 100) / 100, (int)(p * 100) % 100,
                                                               (int)(speed * 100) / 100, (int)(speed * 100) % 100,
                                                               (int)(t * 100) / 100, (int)(t * 100) % 100);
            rt_thread_mdelay(100);
        }
    }
    else if (!strcmp(argv[1], "temp"))
    {
        float temp;

        if (dev == RT_NULL)
        {
            rt_kprintf("Please probe mlx90384 first!\n");
            return ;
        }

        mlx90384_get_temp(dev, &temp);
        rt_kprintf("temp = %d.%d\n", (int)(temp * 100) / 100, (int)(temp * 100) % 100);
    }
    else if (!strcmp(argv[1], "config"))
    {
        union mlx90384_config_reg config;

        if (dev == RT_NULL)
        {
            rt_kprintf("Please probe mlx90384 first!\n");
            return ;
        }
        mlx90384_get_config(dev, &config);

        rt_kprintf("config:%x\n", config.word_val);
    }
    else
    {
        rt_kprintf("Unknown command, please enter 'mlx90384' get help information!\n");
    }
}
#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(mlx90384, mlx90384 sensor function);
#endif

