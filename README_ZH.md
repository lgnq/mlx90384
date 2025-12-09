# mlx90384

中文页 | [English](README.md)

## 简介

本软件包是为 Melexis 公司的磁位置传感器MLX90382提供的传感器驱动包。并且本软件包新的版本已经对接到了 Sensor 框架，通过 Sensor 框架，开发者可以快速的将此传感器驱动起来。

## 支持情况

| 包含设备         | 加速度计 | 陀螺仪 | 磁力计 |
| ---------------- | -------- | ------ | ------ |
| **通讯接口**     |          |        |        |
| SPI              | √        | √      | √      |
| **工作模式**     |          |        |        |
| 轮询             | √        | √      | √      |
| 中断             |          |        |        |
| FIFO             |          |        |        |
| **电源模式**     |          |        |        |
| 掉电             | √        | √      | √      |
| 低功耗           |          |        |        |
| 普通             | √        | √      | √      |
| 高功耗           |          |        |        |
| **数据输出速率** |          |        |        |
| **测量范围**     | √        | √      | √      |
| **自检**         |          |        |        |
| **多实例**       |          |        |        |

## 使用说明

### 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- SPI 驱动：mlx90382 设备使用 SPI 进行数据通讯，需要系统 SPI 驱动支持；

### 获取软件包

使用 mlx90382 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages  --->
  peripheral libraries and drivers  --->
    sensors drivers  --->
      mlx90382: Universal 6-axis sensor driver package,support: accelerometer, gyroscope.
              Version (latest)  --->
```

**Version**：软件包版本选择

### 使用软件包

MLX90382 软件包初始化函数如下所示：

```
int rt_hw_mlx90384_init(const char *name, struct rt_sensor_config *cfg);
```

该函数需要由用户调用，函数主要完成的功能有，

- 设备配置和初始化（根据传入的配置信息，配置接口设备和中断引脚）；
- 注册相应的传感器设备，完成 MLX90382 设备的注册；

#### 初始化示例

```
#include "sensor_melexis_mlx90384.h"

int rt_hw_mlx90384_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name  = "spi10";

    rt_hw_mlx90384_init("mps", &cfg);

    return 0;
}
INIT_ENV_EXPORT(rt_hw_mlx90384_port);
```

## 注意事项

暂无

## 联系人信息

维护人:

- [Eamon Fang](https://github.com/lgnq) 

- 主页：<https://github.com/lgnq/mlx90382>