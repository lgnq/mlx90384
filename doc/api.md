# API 说明

在 RT-Thread 上编程，使用 mlx90382 软件包就可以轻松完成传感器的配置以及传感器数据的读取，本章介绍 mlx90382 软件包提供的常用 API。

### 初始化函数

```{.c}
struct mlx90382_device *mlx90382_init(const char *dev_name, rt_uint8_t param);
```

使用指定的通信设备（SPI）初始化mlx90382 ，并返回控制句柄。

| 参数              | 描述                                |
|:------------------|:------------------------------------|
|dev_name               | 用于同 mlx90382 通信的设备名（支持 SPI 设备） |
|param | I2C 通信，根据此处传入的 I2C 地址寻找设备（例如：0x68） |
| **返回**          | **描述**                                |
|struct mlx90382_device *                  | mlx90382_device 结构体的指针，它在调用 mlx90382 库的其他函数时使用 |
|NULL                 | 失败                                |

### 反初始化函数

```{.c}
void mlx90382_deinit(struct mlx90382_device *dev);
```

释放 mlx90382 设备占据的内存空间

| 参数     | 描述                        |
| :------- | :-------------------------- |
| dev      | mlx90382_device 结构体的指针 |
| **返回** | **描述**                    |
| 无返回值 |                             |

### 设定参数

```{.c}
rt_err_t mlx90382_set_param(struct mlx90382_device *dev, enum mlx90382_cmd cmd, rt_uint16_t param);
```

为挂载上的 mlx90382 设备设定参数

| 参数              | 描述                                |
|:------------------|:------------------------------------|
|dev               | mlx90382_device 结构体的指针 |
|cmd | 支持的配置选项，详见下面的介绍 |
|param | 设定的具体的参数值 |
| **返回**          | **描述**                                |
|RT_EOK                  | 成功 |
|< 0                 | 失败                                |

cmd 参数指要配置的选项，param 参数表示设定的参数的具体值。详情如下:


### 读取陀螺仪数据   

```{.c}
rt_err_t mlx90382_get_gyro(struct mlx90382_device *dev, struct mlx90382_3axes *gyro);
```

读取陀螺仪数据 （单位： deg/10s）

| 参数     | 描述                                    |
| :------- | :-------------------------------------- |
| dev      | mlx90382_device 结构体的指针             |
| gyro     | 存储 mlx90382 3轴陀螺仪数据 结构体的指针 |
| **返回** | **描述**                                |
| RT_EOK   | 成功                                    |
| < 0      | 失败                                    |

3 轴陀螺仪数据的结构体定义如下

```{.c}
struct mlx90382_3axes
{
    rt_int16_t x;
    rt_int16_t y;
    rt_int16_t z;
};
```

### 读取加速度计数据

```{.c}
rt_err_t mlx90382_get_accel(struct mlx90382_device *dev, struct mlx90382_3axes *accel);
```

读取加速度计数据 （单位： mg）

| 参数     | 描述                                    |
| :------- | :-------------------------------------- |
| dev      | mlx90382_device 结构体的指针             |
| accel    | 存储 mlx90382 3轴加速度数据 结构体的指针 |
| **返回** | **描述**                                |
| RT_EOK   | 成功                                    |
| < 0      | 失败                                    |

## 校准传感器

### 校准陀螺仪

```c
rt_err_t mlx90382_set_gyro_offset(struct mlx90382_device *dev, struct mlx90382_3axes *offset);
```

 校准陀螺仪。注意offset的单位和读取数据的单位不同。

| 参数     | 描述                        |
| :------- | :-------------------------- |
| dev      | mlx90382_device 结构体的指针 |
| offset   | 存储 校准量 结构体的指针    |
| **返回** | **描述**                    |
| RT_EOK   | 成功                        |
| 其他     | 失败                        |

### 校准加速度传感器

```c
rt_err_t mlx90382_set_gyro_offset(struct mlx90382_device *dev, struct mlx90382_3axes *offset);
```

校准加速度传感器。注意offset的单位和读取数据的单位不同。

| 参数     | 描述                        |
| :------- | :-------------------------- |
| dev      | mlx90382_device 结构体的指针 |
| offset   | 存储 校准量 结构体的指针    |
| **返回** | **描述**                    |
| RT_EOK   | 成功                        |
| 其他     | 失败                        |

### 读取温度计数据

```{.c}
rt_err_t mlx90382_get_temp(struct mlx90382_device *dev, float *temp);
```

读取温度计数据 （单位：摄氏度）

| 参数     | 描述                            |
| :------- | :------------------------------ |
| dev      | mlx90382_device 结构体的指针     |
| temp     | 存储 mlx90382 温度数据地址的指针 |
| **返回** | **描述**                        |
| RT_EOK   | 成功                            |
| < 0      | 失败                            |
