# mlx90384

[中文页](README_ZH.md) | English

## Introduction

This software package is a universal sensor driver package for Melexis's resolver sensors. And the new version of this software package has been connected to the Sensor framework, through the Sensor framework, developers can quickly drive this sensor.

## Support

| Contains equipment          | Accelerometer | Gyroscope | Magnetometer |
| --------------------------- | ------------- | --------- | ------------ |
| **Communication Interface** |               |           |              |
| SPI                         | √             | √         | √            |
| **Work Mode**               |               |           |              |
| Polling                     | √             | √         | √            |
| Interruption                |               |           |              |
| FIFO                        |               |           |              |
| **Power Mode**              |               |           |              |
| Power down                  | √             | √         | √            |
| Low power consumption       |               |           |              |
| Normal                      | √             | √         | √            |
| High power consumption      |               |           |              |
| **Data output rate**        |               |           |              |
| **Measuring Range**         | √             | √         | √            |
| **Self-check**              |               |           |              |
| **Multi-instance**          |               |           |              |

## Instructions for use

### Dependence

- RT-Thread 4.0.0+
- Sensor component
- SPI driver: mlx90382 devices use SPI for data communication, and need system SPI driver support;

### Get the package

To use the mlx90382 software package, you need to select it in the RT-Thread package management. The specific path is as follows:

```
RT-Thread online packages  --->
  peripheral libraries and drivers  --->
    sensors drivers  --->
      mlx90382: Universal 3D magnetic sensor driver package.
              Version (latest)  --->
```

**Version**: software package version selection

### Using packages

The initialization function of mlx90382 software package is as follows:

```
int rt_hw_mlx90382_init(const char *name, struct rt_sensor_config *cfg);
```

This function needs to be called by the user. The main functions of the function are:

- Device configuration and initialization (configure interface devices and interrupt pins according to the incoming configuration information);
- Register the corresponding sensor device and complete the registration of the mlx90382 device;

#### Initialization example

```
#include "sensor_melexis_mlx90382.h"

int rt_hw_mlx90382_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name  = "spi10";

    rt_hw_mlx90382_init("mps", &cfg);

    return 0;
}
INIT_ENV_EXPORT(rt_hw_mlx90382_port);
```

```
// in board.c

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }

}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }

}

// 自动初始化实现SPI设备挂载
int mlx90382_spi_device_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    return rt_hw_spi_device_attach("spi1", "spi10", GPIOA, GPIO_PIN_8);    //EAF
}
INIT_DEVICE_EXPORT(mlx90382_spi_device_init);
```

## Precautions

No

## contact information

Maintenance man:

- [Eamon Fang](https://github.com/lgnq) 

- 主页：<https://github.com/lgnq/mlx90382>
