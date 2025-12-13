/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-30     lgnq         the first version
 */

#include <sensor_melexis_mlx90384.h>

#define DBG_TAG "sensor.mlx.mlx90384"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <stdlib.h>

#define mlx_dev ((struct mlx90384_device *)sensor->parent.user_data)

rt_uint16_t sample_freq = 100;

static rt_size_t mlx90384_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    mlx90384_get_lin_phase(mlx_dev, buf);

    return 1;

//    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
//    {
//        return mlx90384_get_angular(mlx_dev, buf);
//    }
//    else
//        return 0;
}

rt_err_t mlx90384_get_info(rt_sensor_t sensor)
{
    rt_err_t result = RT_EOK;

    rt_uint16_t a_ver;
    rt_uint16_t d_ver_h;
    rt_uint32_t d_ver;
    rt_uint16_t zero_position;
    union mlx90384_config_reg config;

    if (mlx_dev == RT_NULL)
    {
        rt_kprintf("Please probe mlx90384 first!\n");
        return -1;
    }

    mlx90384_get_config(mlx_dev, &config);
    mlx90384_get_zero_position(mlx_dev, &zero_position);
    mlx90384_get_digital_version(mlx_dev, &d_ver_h);
    mlx90384_get_analog_version(mlx_dev, &a_ver);
    d_ver = (a_ver>>8) + (d_ver_h<<16);

    rt_kprintf("aversion:%x\n", a_ver&0xFF);
    rt_kprintf("dversion:%x\n", d_ver);
    rt_kprintf("zeroposition:%x\n", zero_position);
    rt_kprintf("config:%x\n", config.word_val);

    return result;
}

static rt_err_t mlx90384_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(uint8_t *)args = mlx_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        break;
    case RT_SENSOR_CTRL_USER_CMD_INFO:
        result = mlx90384_get_info(sensor);
        break;
    case RT_SENSOR_CTRL_USER_CMD_LIN_PHASE:
        result = mlx90384_get_lin_phase(mlx_dev, (float *)args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_DRIFTC_PHASE:
        result = mlx90384_get_driftc_phase(mlx_dev, (float *)args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SC_PHASE:
        result = mlx90384_get_sc_phase(mlx_dev, (float *)args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SPEED:
        result = mlx90384_get_speed(mlx_dev, (rt_int16_t *)args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_TEMP:
        result = mlx90384_get_temp(mlx_dev, (float *)args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SOFT_RESET:
        result = mlx90384_soft_reset(mlx_dev);
        break;
    case RT_SENSOR_CTRL_USER_CMD_GET_ZEROPOSITION:
        result = mlx90384_get_zero_position(mlx_dev, (rt_uint16_t *)args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SET_ZEROPOSITION:
        result = mlx90384_set_zero_position(mlx_dev, *(rt_uint16_t *)args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SET_SENSING_MODE:
        result = mlx90384_set_sensing_mode(mlx_dev, *(rt_uint16_t *)args);
        break;
    default:
        rt_kprintf("unknown MLX90384 CTRL CMD\r\n");
        return -RT_ERROR;
    }

    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    mlx90384_fetch_data,
    mlx90384_control
};

int rt_hw_mlx90384_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    struct mlx90384_device *mlx_dev_temp;
    rt_sensor_t sensor_mag = RT_NULL;

    mlx_dev_temp = mlx90384_init((&cfg->intf)->dev_name, RT_NULL);

    if (mlx_dev_temp == RT_NULL)
    {
        LOG_E("mlx90384 init err!");
        goto __exit;
    }

    {
        sensor_mag = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_mag == RT_NULL)
            goto mag__exit;

        sensor_mag->info.type       = RT_SENSOR_CLASS_MAG;
        sensor_mag->info.vendor     = RT_SENSOR_VENDOR_MELEXIS;
        sensor_mag->info.model      = "mlx90384_mag";
        sensor_mag->info.unit       = RT_SENSOR_UNIT_MGAUSS;
        sensor_mag->info.intf_type  = RT_SENSOR_INTF_SPI;
        sensor_mag->info.range_max  = 49120;
        sensor_mag->info.range_min  = -49120;
        sensor_mag->info.period_min = 100;

        rt_memcpy(&sensor_mag->config, cfg, sizeof(struct rt_sensor_config));
        sensor_mag->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_mag, name, RT_DEVICE_FLAG_RDWR, mlx_dev_temp);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto mag__exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

mag__exit:
    if (sensor_mag)
        rt_free(sensor_mag);

__exit:
    if (mlx_dev_temp)
        mlx90384_deinit(mlx_dev_temp);

    return -RT_ERROR;
}

static void read_mps_entry(void *parameter)
{
    rt_size_t res;
    rt_device_t dev = RT_NULL;

    float lin_phase;
    float driftc_phase;
    float sc_phase;
    rt_int16_t speed;
    float temp;

    dev = rt_device_find(parameter);
    if (dev == RT_NULL)
    {
        rt_kprintf("Can't find device:%s\n", parameter);
        return;
    }

    res = rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
    if (res != RT_EOK)
    {
        if (res == -RT_EBUSY)
        {
            rt_kprintf("device is already opened!\n");
        }
        else
        {
            rt_kprintf("open device failed!\n");
            return;
        }
    }

    while (1)
    {
        res = rt_device_control(dev, RT_SENSOR_CTRL_USER_CMD_LIN_PHASE, &lin_phase);
        if (res != RT_EOK)
        {
            rt_kprintf("read angular value after linearization failed!\n");
            rt_device_close(dev);
            return;
        }

        res = rt_device_control(dev, RT_SENSOR_CTRL_USER_CMD_DRIFTC_PHASE, &driftc_phase);
        if (res != RT_EOK)
        {
            rt_kprintf("read angular value after delay compensation and zero-point offset correction failed!\n");
            rt_device_close(dev);
            return;
        }

        res = rt_device_control(dev, RT_SENSOR_CTRL_USER_CMD_SC_PHASE, &sc_phase);
        if (res != RT_EOK)
        {
            rt_kprintf("read Position value after signal conditioning failed!\n");
            rt_device_close(dev);
            return;
        }

        res = rt_device_control(dev, RT_SENSOR_CTRL_USER_CMD_SPEED, &speed);
        if (res != RT_EOK)
        {
            rt_kprintf("read speed failed!\n");
            rt_device_close(dev);
            return;
        }

        res = rt_device_control(dev, RT_SENSOR_CTRL_USER_CMD_TEMP, &temp);
        if (res != RT_EOK)
        {
            rt_kprintf("read temp failed!\n");
            rt_device_close(dev);
            return;
        }

        //lin_phase, driftc_phase, sc_phase, speed, temp
        rt_kprintf("data:%d.%d,%d.%d,%d.%d,%d.%d,%d.%d\n", (int)(lin_phase * 100) / 100, (int)(lin_phase * 100) % 100,
                                                           (int)(driftc_phase * 100) / 100, (int)(driftc_phase * 100) % 100,
                                                           (int)(sc_phase * 100) / 100, (int)(sc_phase * 100) % 100,
                                                           (int)(speed * 100) / 100, (int)((float)speed * 100) % 100,
                                                           (int)((float)temp * 100) / 100, (int)(temp * 100) % 100);

        rt_thread_mdelay(sample_freq);
    }
}

int rt_hw_mlx90384_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name  = "spi10";

    rt_hw_mlx90384_init("mps", &cfg);

    return 0;
}
INIT_ENV_EXPORT(rt_hw_mlx90384_port);

rt_err_t mlx90384_measurement_onoff(int argc, char **argv)
{
    rt_thread_t mlx90384_thread;

    if (!strcmp(argv[1], "on"))
    {
        mlx90384_thread = rt_thread_create("mlx90384", read_mps_entry, "mag_mps", 1024, RT_THREAD_PRIORITY_MAX / 2, 20);
        if (mlx90384_thread != RT_NULL)
        {
            rt_thread_startup(mlx90384_thread);

            return 0;
        }
    }
    else if (!strcmp(argv[1], "off"))
    {
        mlx90384_thread = rt_thread_find("mlx90384");

        if (mlx90384_thread != RT_NULL)
        {
            rt_thread_delete(mlx90384_thread);

            return 0;
        }
    }

    return -1;
}

rt_err_t mlx90384_set_sample_freq(int argc, char **argv)
{
    rt_size_t res = RT_EOK;

    sample_freq = atoi(argv[1]);
    rt_kprintf("sample freq = %d\r\n", sample_freq);

    return res;
}

rt_err_t mlx90384_ops_ctrl(int argc, char **argv)
{
    rt_size_t res   = RT_EOK;
    rt_device_t dev = RT_NULL;

    rt_uint16_t p = 0;

    if (argc == 3)
        p = atoi(argv[2]);

    dev = rt_device_find("mag_mps");
    if (dev == RT_NULL)
    {
        rt_kprintf("Can't find device:%s\n");
        return -RT_ERROR;
    }

    res = rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
    if (res != RT_EOK)
    {
        if (res == -RT_EBUSY)
        {
            rt_kprintf("device is already opened!\n");
        }
        else
        {
            rt_kprintf("open device failed!\n");
            return -RT_ERROR;
        }
    }

    if (rt_device_control(dev, atoi(argv[1]), &p))
    {
        rt_kprintf("device control set failed, 0x%x!\n", atoi(argv[1]));
        return -RT_ERROR;
    }

    return res;
}

#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(mlx90384_measurement_onoff, mlx90384 sensor function);
    MSH_CMD_EXPORT(mlx90384_set_sample_freq, mlx90384 sensor function);
    MSH_CMD_EXPORT(mlx90384_ops_ctrl, mlx90384 sensor function);
#endif
