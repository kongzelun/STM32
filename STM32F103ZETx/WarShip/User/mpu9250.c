/**
  * @file   mpu9250.c
  * @author Kong Zelun
  * @date   2017-03-01
  * @brief  MPU9250 driver and DMP driver.
  */

/**
  * @region Includes
  * @{
  */
#include "mpu9250.h"
#include <math.h>
#include "i2c.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/**
  * @}
  */

/**
  * @region Private Defines
  * @{
  */

/**
  * @}
  */

/**
  * @region Private Types
  * @{
  */
  
/**
  * @brief
  * @member gyro        Gyro data in hardware units.
  * @member accel       Accel data in hardware units.
  * @member quat        3-axis quaternion data in hardware units.
  * @member timestamp   Timestamp in milliseconds.
  * @member sensors     Mask of sensors read from FIFO.
  * @member more        Number of remaining packets.
  */
typedef struct
{
  short               gyro[3];
  
  short               accel[3];
  
  long                quat[4];
  
  unsigned long       timestamp;
  
  short               sensors;
  
  unsigned char       more;
  
}MPU_DataTypeDef;
  
/**
  * @brief
  */
typedef struct __MPU_HandleTypeDef
{
  GPIO_TypeDef        *INT_Port;
  
  GPIO_TypeDef        *FSYNC_Port;
  
  GPIO_TypeDef        *AD0_Port;
  
  uint16_t            FSYNC_Pin;
  
  uint16_t            INT_Pin;
  
  uint16_t            *AD0_Pin;
  
  I2C_HandleTypeDef   *hi2c;
  
  MPU_ErrorTypeDef    status;
  
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
  int8_t              gyro_orientation[9];
  
  uint16_t            rate;
  
  MPU_DataTypeDef     LastData;
  
}MPU_HandleTypeDef;

/**
  * @}
  */

/**
  * @region Exported Variables
  * @{
  */

/**
  * @}
  */

/**
  * @region Global Variables
  * @{
  */
static MPU_HandleTypeDef hmpu9250;

/**
  * @}
  */

/**
  * @region Private Functions Prototype
  * @{
  */
static unsigned short inv_row_2_scale(const signed char *row);
static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
static void run_self_test(void);

/**
  * @}
  */

/**
  * @region Public Functions
  * @{
  */

/**
  * @brief  Initialize MPU sensor.
  * @param  hmpu: MPU handle.
  * @param  hi2c: i2c handle.
  * @retval MPU error type.
  */
MPU_ErrorTypeDef MPU_Init(I2C_HandleTypeDef *hi2c)
{
  hmpu9250.hi2c = hi2c;
  hmpu9250.gyro_orientation[0] = -1;
  hmpu9250.gyro_orientation[1] = 0;
  hmpu9250.gyro_orientation[2] = 0;
  hmpu9250.gyro_orientation[3] = 0;
  hmpu9250.gyro_orientation[4] = -1;
  hmpu9250.gyro_orientation[5] = 0;
  hmpu9250.gyro_orientation[6] = 0;
  hmpu9250.gyro_orientation[7] = 0;
  hmpu9250.gyro_orientation[8] = 1;
  hmpu9250.rate = 200;
  hmpu9250.status = MPU_OK;
  
  if (mpu_init())
  {
    hmpu9250.status = MPU_INIT_ERROR;
  }
  
  if (hmpu9250.status == MPU_OK)
  {
    if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS))
    {
      hmpu9250.status = MPU_ERROR;
    }
  }
  
  if (hmpu9250.status == MPU_OK)
  {
    if(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS))
    {
      hmpu9250.status = MPU_ERROR;
    }
  }
  
  if (hmpu9250.status == MPU_OK)
  {
    if(mpu_set_sample_rate(hmpu9250.rate))
    {
      hmpu9250.status = MPU_ERROR;
    }
  }
  
  if (hmpu9250.status == MPU_OK)
  {
    if(dmp_load_motion_driver_firmware())
    {
      hmpu9250.status = MPU_DMP_ERROR;
    }
  }
  
  if (hmpu9250.status == MPU_OK)
  {
    if(dmp_set_orientation(inv_orientation_matrix_to_scalar(hmpu9250.gyro_orientation)))
    {
      hmpu9250.status = MPU_DMP_ERROR;
    }
  }
  
  if (hmpu9250.status == MPU_OK)
  {
    if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                          DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                          DMP_FEATURE_GYRO_CAL))
    {
      hmpu9250.status = MPU_DMP_ERROR;
    }
  }
  
  if (hmpu9250.status == MPU_OK)
  {
    if(dmp_set_fifo_rate(hmpu9250.rate))
    {
      hmpu9250.status = MPU_DMP_ERROR;
    }
  }
  
  if (hmpu9250.status == MPU_OK)
  {
    if(dmp_set_orientation(inv_orientation_matrix_to_scalar(hmpu9250.gyro_orientation)))
    {
      hmpu9250.status = MPU_DMP_ERROR;
    }
  }

  if (hmpu9250.status == MPU_OK)
  {
    run_self_test();
  }
  
  if (hmpu9250.status == MPU_OK)
  {
    if(mpu_set_dmp_state(1))
    {
      hmpu9250.status = MPU_DMP_ERROR;
    }
  }
  
  return hmpu9250.status;
}

/**
  * @brief      Get one data packet MPU.
  * @param[out] gyro        Gyro data in hardware units.
  * @param[out] accel       Accel data in hardware units.
  * @param[out] quat        3-axis quaternion data in hardware units.
  * @param[out] euler       euler angle, sequence pitch, roll, yaw.
  * @param[out] timestamp   Timestamp in milliseconds.
  * @retval      MPU error type.
  */
MPU_ErrorTypeDef MPU_GetData(short *gyro, short *accel, short *compass, long *temp, long *quat, double *euler, unsigned long *timestamp)
{
  const double q30 = 1073741824.0f;
  double q0, q1, q2, q3;
  
  if (hmpu9250.status == MPU_OK | hmpu9250.status == MPU_GETDATA_ERROR)
  {
    if (dmp_read_fifo(hmpu9250.LastData.gyro, hmpu9250.LastData.accel, hmpu9250.LastData.quat, 
                  &hmpu9250.LastData.timestamp, &hmpu9250.LastData.sensors, &hmpu9250.LastData.more))
    {
      hmpu9250.status = MPU_GETDATA_ERROR;
    }
    else
    {
      hmpu9250.status = MPU_OK;
    }
  }
  
  if (hmpu9250.status == MPU_OK)
  {
    if (mpu_get_compass_reg(compass, timestamp))
    {
      hmpu9250.status = MPU_GETDATA_ERROR;
    }
    else
    {
      hmpu9250.status = MPU_OK;
    }
  }
  
  if (hmpu9250.status == MPU_OK)
  {
    if (mpu_get_temperature(temp, timestamp))
    {
      hmpu9250.status = MPU_GETDATA_ERROR;
    }
    else
    {
      hmpu9250.status = MPU_OK;
    }
  }
  
  if (hmpu9250.status == MPU_OK)
  {
    if (hmpu9250.LastData.sensors & INV_XYZ_GYRO)
    {
      gyro[0] = hmpu9250.LastData.gyro[0];
      gyro[1] = hmpu9250.LastData.gyro[1];
      gyro[2] = hmpu9250.LastData.gyro[2];
    }
    
    if (hmpu9250.LastData.sensors & INV_XYZ_ACCEL)
    {
      accel[0] = hmpu9250.LastData.accel[0];
      accel[1] = hmpu9250.LastData.accel[1];
      accel[2] = hmpu9250.LastData.accel[2];
    }
    
    if (hmpu9250.LastData.sensors & INV_WXYZ_QUAT)
    {
      quat[0] = hmpu9250.LastData.quat[0];
      quat[1] = hmpu9250.LastData.quat[1];
      quat[2] = hmpu9250.LastData.quat[2];
      quat[3] = hmpu9250.LastData.quat[3];
      
      q0 = quat[0] / q30;
      q1 = quat[1] / q30;
      q2 = quat[2] / q30;
      q3 = quat[3] / q30;
      
      euler[0] = asin(-2 * q1 * q3 + 2 * q0* q2) * 57.2957795f; // pitch
      euler[1] = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.2957795f; // roll
      euler[2] = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2957795f; // yaw
    }
    
    *timestamp = hmpu9250.LastData.timestamp;
  }
  
  return hmpu9250.status;
}

/**
  * @brief  Write data to MPU register through I2C.
  * @retval 0 success, -1 fail.
  */
int MPU_ReadData(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
  if (HAL_I2C_Mem_Read(hmpu9250.hi2c, slave_addr << 1, reg_addr, 1, data, length, 10) == HAL_OK)
  {
    return 0;
  }
  else
  {
    hmpu9250.status = MPU_I2C_ERROR;
    return -1;
  }
}

/**
  * @brief  Write data to MPU register through I2C.
  * @retval 0 success, -1 fail.
  */
int MPU_WriteData(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, const uint8_t *data)
{
  if (HAL_I2C_Mem_Write(hmpu9250.hi2c, slave_addr << 1, reg_addr, 1, (uint8_t *)data, length, 10) == HAL_OK)
  {
    return 0;
  }
  else
  {
    hmpu9250.status = MPU_I2C_ERROR;
    return -1;
  }
}

/**
  * @brief  Delay 1 ms.
  * @param  num_ms Number of ms to delay
  */
void MPU_Delay(uint32_t num_ms)
{
  extern void Delay(__IO uint32_t Delay);
  Delay(num_ms << 10);
}

/**
  * @brief  Get timestamp.
  */
void MPU_GetTimeStamp(unsigned long *count)
{
  *count = HAL_GetTick();
}

/**
  * @}
  */

/**
  * @region Private Functions
  * @{
  */

/**
  * @brief These next two functions converts the orientation matrix (see
  *        gyro_orientation) to a scalar representation for use by the DMP.
  */
static unsigned short inv_row_2_scale(const signed char *row)
{
  unsigned short b;

  if (row[0] > 0)
      b = 0;
  else if (row[0] < 0)
      b = 4;
  else if (row[1] > 0)
      b = 1;
  else if (row[1] < 0)
      b = 5;
  else if (row[2] > 0)
      b = 2;
  else if (row[2] < 0)
      b = 6;
  else
      b = 7;// error
  return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
  unsigned short scalar;

  /*
     XYZ  010_001_000 Identity Matrix
     XZY  001_010_000
     YXZ  010_000_001
     YZX  000_010_001
     ZXY  001_000_010
     ZYX  000_001_010
   */

  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;


  return scalar;
}

static void run_self_test(void)
{
  int result;
  long gyro[3], accel[3];

  result = mpu_run_self_test(gyro, accel);
  if (result == 0x7)
  {
    // Test passed. We can trust the gyro data here, so let's push it down to the DMP.
//    float gyro_sens;
//    unsigned short accel_sens;
//    mpu_get_gyro_sens(&gyro_sens);
//    gyro[0] = (long)(gyro[0] * gyro_sens);
//    gyro[1] = (long)(gyro[1] * gyro_sens);
//    gyro[2] = (long)(gyro[2] * gyro_sens);
//    dmp_set_gyro_bias(gyro);
//    
//    mpu_get_accel_sens(&accel_sens);
//    accel[0] *= accel_sens;
//    accel[1] *= accel_sens;
//    accel[2] *= accel_sens;
//    dmp_set_accel_bias(accel);
  }
  else
  {
    hmpu9250.status = MPU_SELFTEST_ERROR;
  }
}

/**
  * @}
  */
