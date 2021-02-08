/**
  * @file   infrared.h
  * @author Kong Zelun
  * @date   2017年4月14日
  * @brief  
  */

/**
  * @region Define to prevent recursive inclusion
  * @{
  */
#ifndef __INFRARED_H
#define __INFRARED_H

/**
  * @region Includes
  * @{
  */
#include "main.h"
#include "stm32f1xx_hal.h"
#include "tim.h"

/**
  * @}
  */

/**
  * @region Public Defines
  * @{
  */

/**
  * @}
  */

/**
  * @region Public Types
  * @{
  */
typedef enum __INFRARED_ErrorTypeDef
{
  INFRARED_OK = 0,
  INFRARED_READY,
  INFRARED_STARTDECODING,
  INFRARED_DECODING,
  INFRARED_DATAREADY,
  INFRARED_TIMEOUT,
  INFRARED_ERROR,
} INFRARED_ErrorTypeDef;

typedef struct __INFRARED_ConfigTypeDef
{
  GPIO_TypeDef *GPIO_Port;
  
  uint16_t GPIO_Pin;
  
  TIM_HandleTypeDef *htim;
  
  uint32_t TIM_FallingChannel;
  
  uint32_t TIM_RasingChannel;
  
} INFRARED_InitTypeDef;

typedef struct __INFRARED_CodeTypeDef
{
  uint32_t LastCode;
  // The number of continuous sending.
  uint32_t ContinuousCount;
  // The number of bits which have been received.
  uint32_t CodeBitCount;
  
  uint32_t LastFallingEdge;
  
  uint32_t LastRisingEdge;
  
  uint32_t FormerRisingEdge;
  
} INFRARED_CodeTypeDef;

typedef struct __INFRARED_HandleTypeDef
{
  INFRARED_InitTypeDef Init;
  
  INFRARED_CodeTypeDef Code;
  
  __IO INFRARED_ErrorTypeDef State;
  
} INFRARED_HandleTypeDef;

/**
  * @brief
  */

/**
  * @}
  */

/**
  * @region Public Variables 
  * @{
  */
extern INFRARED_HandleTypeDef hinfrared1;

/**
  * @}
  */

/**
  * @region Public Functions Prototype
  * @{
  */
void INFRARED_Init(INFRARED_HandleTypeDef *hinfrared);
INFRARED_ErrorTypeDef INFRARED_Start(INFRARED_HandleTypeDef *hinfrared);
INFRARED_ErrorTypeDef INFRARED_Stop(INFRARED_HandleTypeDef *hinfrared);
INFRARED_ErrorTypeDef INFRARED_PollForDecoding(INFRARED_HandleTypeDef *hinfrared, uint32_t timeout, uint32_t interval);
uint32_t INFRARED_GetCode(INFRARED_HandleTypeDef *hinfrared);
uint32_t INFRARED_GetState(INFRARED_HandleTypeDef *hinfrared);
void INFRARED_RasingEdgeHandler(INFRARED_HandleTypeDef *hinfrared);
void INFRARED_FallingEdgeHandler(INFRARED_HandleTypeDef *hinfrared);
__weak void INFRARED_Delay(__IO uint32_t Delay);
__weak void INFRARED_DataReadyCallback(INFRARED_HandleTypeDef *hinfrared);
__weak void INFRARED_ErrorCallback(INFRARED_HandleTypeDef *hinfrared);

/**
  * @}
  */

#endif

/**
  * @}
  */
