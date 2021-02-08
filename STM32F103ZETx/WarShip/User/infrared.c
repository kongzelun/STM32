/**
  * @file   infrared.c
  * @author Kong Zelun
  * @date   2017年4月14日
  * @brief  
  */

/**
  * @region Includes
  * @{
  */
#include "infrared.h"

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
  * @}
  */

/**
  * @region Global Variables
  * @{
  */
INFRARED_HandleTypeDef hinfrared1 = 
{
  .Init.GPIO_Port = IR_GPIO_Port,
  .Init.GPIO_Pin = IR_Pin,
  .Init.htim = &htim4,
  .Init.TIM_RasingChannel = TIM_CHANNEL_3,
  .Init.TIM_FallingChannel = TIM_CHANNEL_4
};

/**
  * @}
  */

/**
  * @region Private Functions Prototype
  * @{
  */
__STATIC_INLINE void INFRARED_Decode(INFRARED_HandleTypeDef *hinfrared);

/**
  * @}
  */

/**
  * @region Public Functions
  * @{
  */

/**
  * @brief      
  * @param[in]  
  * @return     
  */
void INFRARED_Init(INFRARED_HandleTypeDef *hinfrared)
{
  hinfrared->State = INFRARED_READY;
  
  HAL_TIM_IC_Start_IT(hinfrared->Init.htim, hinfrared->Init.TIM_FallingChannel);
  HAL_TIM_IC_Start_IT(hinfrared->Init.htim, hinfrared->Init.TIM_RasingChannel);
}

/**
  * @brief      
  * @param[in]  
  * @return     
  */
INFRARED_ErrorTypeDef INFRARED_Start(INFRARED_HandleTypeDef *hinfrared)
{
  HAL_StatusTypeDef status;
  
  status = HAL_TIM_IC_Start_IT(hinfrared->Init.htim, hinfrared->Init.TIM_FallingChannel);
  
  if (status == HAL_OK)
  {
    HAL_TIM_IC_Start_IT(hinfrared->Init.htim, hinfrared->Init.TIM_RasingChannel);
  }
  
  if (status == HAL_OK)
  {
    return INFRARED_OK;
  }
  else
  {
    return INFRARED_ERROR;
  }
}

/**
  * @brief      
  * @param[in]  
  * @return     
  */
INFRARED_ErrorTypeDef INFRARED_Stop(INFRARED_HandleTypeDef *hinfrared)
{
  HAL_StatusTypeDef status;
  
  status = HAL_TIM_IC_Stop_IT(hinfrared->Init.htim, hinfrared->Init.TIM_FallingChannel);
  
  if (status == HAL_OK)
  {
    HAL_TIM_IC_Stop_IT(hinfrared->Init.htim, hinfrared->Init.TIM_RasingChannel);
  }
  
  if (status == HAL_OK)
  {
    return INFRARED_OK;
  }
  else
  {
    return INFRARED_ERROR;
  }
}

/**
  * @brief  
  * @param  
  * @param  
  * @retval 
  */
INFRARED_ErrorTypeDef INFRARED_PollForDecoding(INFRARED_HandleTypeDef *hinfrared, uint32_t timeout, uint32_t interval)
{
  uint32_t tickstart = HAL_GetTick();
  
  while (hinfrared->State != INFRARED_DATAREADY)
  {
    if (timeout != HAL_MAX_DELAY)
    {
      if ((timeout == 0) || (HAL_GetTick() - tickstart) > timeout)
      {
        hinfrared->State = INFRARED_TIMEOUT;
      }
    }
    INFRARED_Delay(interval);
  }
  
  return hinfrared->State;
}

/**
  * @brief      
  * @param[in]  
  * @return     
  */
uint32_t INFRARED_GetState(INFRARED_HandleTypeDef *hinfrared)
{
  return hinfrared->State;
}

/**
  * @brief      
  * @param[in]  
  * @return     
  */
uint32_t INFRARED_GetCode(INFRARED_HandleTypeDef *hinfrared)
{
  
  return hinfrared->Code.LastCode;
}

/**
  * @brief      This function should be called in HAL_TIM_IC_CaptureCallback.
  * @param[in]  
  * @return     None.
  */
void INFRARED_RasingEdgeHandler(INFRARED_HandleTypeDef *hinfrared)
{
  hinfrared->Code.LastRisingEdge = HAL_TIM_ReadCapturedValue(hinfrared->Init.htim, hinfrared->Init.TIM_RasingChannel);
  INFRARED_Decode(hinfrared);
  hinfrared->Code.FormerRisingEdge = hinfrared->Code.LastRisingEdge;
}

/**
  * @brief      This function should be called in HAL_TIM_IC_CaptureCallback.
  * @param[in]  
  * @return     None.
  */
void INFRARED_FallingEdgeHandler(INFRARED_HandleTypeDef *hinfrared)
{
  hinfrared->Code.LastFallingEdge = HAL_TIM_ReadCapturedValue(hinfrared->Init.htim, hinfrared->Init.TIM_FallingChannel);
}

/**
  * @brief      
  * @param[in]  
  * @return     None.
  */
__weak void INFRARED_DataReadyCallback(INFRARED_HandleTypeDef *hinfrared)
{
  UNUSED(0);
}

/**
  * @brief      
  * @param[in]  
  * @return     None.
  */
__weak void INFRARED_ErrorCallback(INFRARED_HandleTypeDef *hinfrared)
{
  UNUSED(0);
}

/**
  * @brief      
  * @param[in]  
  * @return     None.
  */
__weak void INFRARED_Delay(__IO uint32_t Delay)
{
  HAL_Delay(Delay);
}

/**
  * @brief  
  * @param  
  * @param  
  * @retval 
  */
__STATIC_INLINE void INFRARED_Decode(INFRARED_HandleTypeDef *hinfrared)
{
  int temp;
  
  if (hinfrared->State == INFRARED_DATAREADY || hinfrared->State == INFRARED_READY)
  {
    temp = hinfrared->Code.LastRisingEdge - hinfrared->Code.LastFallingEdge;
    if (temp < 0)
    {
      temp += hinfrared->Init.htim->Instance->ARR;
    }
    // 9ms
    if (temp > 800 && temp < 1000)
    {
      hinfrared->State = INFRARED_STARTDECODING;
    }
  }
  else if (hinfrared->State == INFRARED_STARTDECODING)
  {
    temp = hinfrared->Code.LastFallingEdge - hinfrared->Code.FormerRisingEdge;
    if (temp < 0)
    {
      temp += hinfrared->Init.htim->Instance->ARR;
    }
    
    // 4.5ms
    if (temp > 400 && temp < 500)
    {
      hinfrared->Code.CodeBitCount = 0;
      hinfrared->Code.LastCode = 0;
      hinfrared->Code.ContinuousCount = 0;
      hinfrared->State = INFRARED_DECODING;
    }
    // 2.5ms
    else if (temp > 200 && temp < 300)
    {
      hinfrared->State = INFRARED_DATAREADY;
      hinfrared->Code.ContinuousCount++;
      if (hinfrared->Code.ContinuousCount > 1)
      {
        INFRARED_DataReadyCallback(hinfrared);
      }
    }
  }
  else if (hinfrared->State == INFRARED_DECODING)
  {
    temp = hinfrared->Code.LastFallingEdge - hinfrared->Code.FormerRisingEdge;
    if (temp < 0)
    {
      temp += hinfrared->Init.htim->Instance->ARR;
    }
    
    hinfrared->Code.LastCode <<= 1;
    
    // 0.56ms 
    if (temp > 0 && temp < 106)
    {
      // bit 0
      hinfrared->Code.LastCode |= 0;
    }
    // 1.68ms
    else if (temp > 118 && temp < 218)
    {
      // bit 1
      hinfrared->Code.LastCode |= 1;
    }
    else
    {
      // error, than goto state INFRARED_READY.
      hinfrared->State = INFRARED_READY;
    }
    
    if (++hinfrared->Code.CodeBitCount == 32)
    {
      hinfrared->State = INFRARED_DATAREADY;
      INFRARED_DataReadyCallback(hinfrared);
    }
  }
}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//  if (htim == hinfrared1.Init.htim)
//  {
//    switch (htim->Channel)
//    {
//      case HAL_TIM_ACTIVE_CHANNEL_1:
//      {
//        break;
//      }
//      case HAL_TIM_ACTIVE_CHANNEL_2:
//      {
//        break;
//      }
//      case HAL_TIM_ACTIVE_CHANNEL_3:
//      {
//        INFRARED_RasingEdgeHandler(&hinfrared1);
//        break;
//      }
//      case HAL_TIM_ACTIVE_CHANNEL_4:
//      {
//        INFRARED_FallingEdgeHandler(&hinfrared1);
//        break;
//      }
//      default:
//        break;
//    }
//  }
//}

/**
  * @}
  */

/**
  * @region Private Functions
  * @{
  */

/**
  * @}
  */
