/**
  * @file   mpu9250.h
  * @author Kong Zelun
  * @date   2017-03-01
  * @brief  MPU9250 driver and DMP driver.
  */

/**
  * @region Define to prevent recursive inclusion
  * @{
  */
#ifndef __MPU9250_H
#define __MPU9250_H

/**
  * @region Includes
  * @{
  */
#include "main.h"

#if defined(STM32F103xE)
#include "stm32f1xx_hal.h"
#elif defined(STM32L053xx)
#include "stm32l0xx_hal.h"
#elif defined(STM32L476xx)
#include "stm32l4xx_hal.h"
#endif

/**
  * @}
  */

/**
  * @region Public Defines
  * @{
  */
#define MPU9250
#define MPU_TARGET_STM32
/**
  * @}
  */

/**
  * @region Public Types
  * @{
  */
  
/**
  * @brief
  */
typedef enum
{
  MPU_OK = 0,
  MPU_INIT_ERROR,
  MPU_I2C_ERROR,
  MPU_DMP_ERROR,
  MPU_SELFTEST_ERROR,
  MPU_GETDATA_ERROR,
  MPU_ERROR
}MPU_ErrorTypeDef;

/**
  * @}
  */

/**
  * @region Public Variables 
  * @{
  */

/**
  * @}
  */

/**
  * @region Public Functions Prototype
  * @{
  */
MPU_ErrorTypeDef MPU_Init(I2C_HandleTypeDef *hi2c);
MPU_ErrorTypeDef MPU_GetData(short *gyro, short *accel, short *compass, long *temp, long *quat, double *euler, unsigned long *timestamp);

/**
  * @}
  */

#endif

/**
  * @}
  */
