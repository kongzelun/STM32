/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "main.h"
#include "stm32l4xx_hal.h"
#include "user.h"
#include "i2c.h"
#include "mpu9250.h"
#include "adc.h"
#include "usart.h"
#include "cmdline.h"
#include "cpu_utils.h"
#include "iwdg.h"
#include "rtc.h"

#define PRINT(string, ...)                                                          \
  do                                                                                \
  {                                                                                 \
    Packet_TypeDef *packet = (Packet_TypeDef *)osPoolAlloc(packetPoolHandle);       \
    if (packet != NULL)                                                             \
    {                                                                               \
      packet->size = sprintf(packet->pString, ( string ), ##__VA_ARGS__);           \
      if (packet->size <= 0 ||                                                      \
          osMessagePut(printQueueHandle, (uint32_t)packet, osWaitForever) != osOK)  \
      {                                                                             \
        osPoolFree(packetPoolHandle, (void *)packet);                               \
      }                                                                             \
    }                                                                               \
  } while (0)
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId cmdTaskHandle;
uint32_t cmdTaskBuffer[ 128 ];
osStaticThreadDef_t cmdTaskControlBlock;
osThreadId mpuTaskHandle;
uint32_t mpuTaskBuffer[ 256 ];
osStaticThreadDef_t mpuTaskControlBlock;
osThreadId adcTaskHandle;
uint32_t adcTaskBuffer[ 128 ];
osStaticThreadDef_t adcTaskControlBlock;
osThreadId printfGateKeeperTaskHandle;
uint32_t printfGateKeeperTaskBuffer[ 128 ];
osStaticThreadDef_t printfGateKeeperTaskControlBlock;
osMessageQId printQueueHandle;
uint8_t printfQueueBuffer[ 16 * sizeof( void* ) ];
osStaticMessageQDef_t printfQueueControlBlock;

/* USER CODE BEGIN Variables */
__align(4) uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((at(0x10000000)));

osPoolId packetPoolHandle;

OutputCtrl_TypeDef Usage_Output;
OutputCtrl_TypeDef Euler_Output;
OutputCtrl_TypeDef MpuTemp_Output;
OutputCtrl_TypeDef ADC_Output;
OutputCtrl_TypeDef Temp_Output;
OutputCtrl_TypeDef Time_Output;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartCMDTask(void const * argument);
void StartMPUTask(void const * argument);
void StartADCTask(void const * argument);
void StartPrintfGateKeeperTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  Usage_Output = Off;
  Euler_Output = Off;
  MpuTemp_Output = Off;
  ADC_Output = Off;
  Temp_Output = Off;
  Time_Output = Off;
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of cmdTask */
  osThreadStaticDef(cmdTask, StartCMDTask, osPriorityRealtime, 0, 128, cmdTaskBuffer, &cmdTaskControlBlock);
  cmdTaskHandle = osThreadCreate(osThread(cmdTask), NULL);

  /* definition and creation of mpuTask */
  osThreadStaticDef(mpuTask, StartMPUTask, osPriorityHigh, 0, 256, mpuTaskBuffer, &mpuTaskControlBlock);
  mpuTaskHandle = osThreadCreate(osThread(mpuTask), NULL);

  /* definition and creation of adcTask */
  osThreadStaticDef(adcTask, StartADCTask, osPriorityHigh, 0, 128, adcTaskBuffer, &adcTaskControlBlock);
  adcTaskHandle = osThreadCreate(osThread(adcTask), NULL);

  /* definition and creation of printfGateKeeperTask */
  osThreadStaticDef(printfGateKeeperTask, StartPrintfGateKeeperTask, osPriorityBelowNormal, 0, 128, printfGateKeeperTaskBuffer, &printfGateKeeperTaskControlBlock);
  printfGateKeeperTaskHandle = osThreadCreate(osThread(printfGateKeeperTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of printQueue */
  osMessageQStaticDef(printQueue, 16, void*, printfQueueBuffer, &printfQueueControlBlock);
  printQueueHandle = osMessageCreate(osMessageQ(printQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  
  osPoolDef(PacketPool, 16, Packet_TypeDef);
  packetPoolHandle = osPoolCreate(osPool(PacketPool));
  if (packetPoolHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  
  while (HAL_RTC_WaitForSynchro(&hrtc) != HAL_OK)
  {
    
  }

  PRINT("#CMD [%d] System initialization complete ......\r\n", HAL_GetTick());

  /* Infinite loop */
  for(;;)
  {
    HAL_IWDG_Refresh(&hiwdg);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    
    if (SystemMode == Command)
    {
      if (Usage_Output == On)
      {
        PRINT("#CMD [%d] CPU: %d%% Heap: %d Bytes PrintQueue: %d\r\n", HAL_GetTick(),
              osGetCPUUsage(), xPortGetFreeHeapSize(), osMessageAvailableSpace(printQueueHandle));
      }
      else if (Time_Output == On)
      {
        HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BIN);
        
        PRINT("#CMD %04d-%02d-%02d %02d:%02d:%02d.%03d\r\n",
              2000 + RTC_Date.Year, RTC_Date.Month, RTC_Date.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds, RTC_Time.SubSeconds);
      }
    }

    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartCMDTask function */
void StartCMDTask(void const * argument)
{
  /* USER CODE BEGIN StartCMDTask */
  Packet_TypeDef *packet;

  while (HAL_UART_Receive_IT(&huart1, &USART1_Buffer, 1) != HAL_OK)
  {
    osDelay(1);
  }

  __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);

  PRINT("#CMD [%d] Cmd initialization complete ......\r\n", HAL_GetTick());
  
  /* Infinite loop */
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    packet = (Packet_TypeDef *)osPoolAlloc(packetPoolHandle);
    if (packet != NULL)
    {
      CMD_Process(USART1_Buffer, packet->pString, &packet->size);
      if (packet->size <= 0 || osMessagePut(printQueueHandle, (uint32_t)packet, osWaitForever) != osOK)
      {
        osPoolFree(packetPoolHandle, (void *)packet);
      }
    }
    
    while (HAL_UART_Receive_IT(&huart1, &USART1_Buffer, 1) != HAL_OK)
    {
      osDelay(1);
    }
  }
  /* USER CODE END StartCMDTask */
}

/* StartMPUTask function */
void StartMPUTask(void const * argument)
{
  /* USER CODE BEGIN StartMPUTask */
  short gyro[3];
  short accel[3];
  short compass[3];
  long temp;
  long quat[4];
  double euler[3];
  unsigned long timestamp;
  MPU_ErrorTypeDef mpuState;
  int mpuErrorTime = 0;
  uint32_t time = osKernelSysTick();
  
  do
  {
    taskENTER_CRITICAL();
    mpuState = MPU_Init(&hi2c1);
    taskEXIT_CRITICAL();
  } while (mpuState != MPU_OK);

  PRINT("#CMD [%d] MPU initialization complete ......\r\n", HAL_GetTick());

  osDelay(1);
  
  /* Infinite loop */
  for(;;)
  {    
    taskENTER_CRITICAL();
    mpuState = MPU_GetData(gyro, accel, compass, &temp, quat, euler, &timestamp);
    taskEXIT_CRITICAL();
    
    if (mpuState == MPU_OK)
    {
      mpuErrorTime = 0;
      
      if (SystemMode == Output)
      {
        taskENTER_CRITICAL();
        PRINT("#DATA_MPU [%d] %6d %6d %6d %6d %6d %6d %6d %6d %6d\r\n", HAL_GetTick(), accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], compass[0], compass[1], compass[2]);
        taskEXIT_CRITICAL();
      }
      else if (SystemMode == Command)
      {
        if (Euler_Output == On)
        {
          PRINT("#CMD Pitch:\t%5.2lf\tRoll:\t%5.2lf\tYaw:\t%5.2lf\r\n", euler[0], euler[1], euler[2]);
        }
        else if (MpuTemp_Output == On)
        {
          PRINT("#CMD %6.1lf°C\r\n", (double)temp / 100000.0);
          osDelayUntil(&time, 490);
        }
      }
    }
    else
    {
      mpuErrorTime++;

      if (mpuErrorTime > 10)
      {
        HAL_I2C_DeInit(&hi2c1);
        MX_I2C1_Init();
      }

      if (mpuErrorTime > 100)
      {
        do
        {
          taskENTER_CRITICAL();
          mpuState = MPU_Init(&hi2c1);
          taskEXIT_CRITICAL();
        } while (mpuState != MPU_OK);

        mpuErrorTime = 0;
      }
    }

    osDelayUntil(&time, 10);
  }
  /* USER CODE END StartMPUTask */
}

/* StartADCTask function */
void StartADCTask(void const * argument)
{
  /* USER CODE BEGIN StartADCTask */
  uint32_t time = osKernelSysTick();
  int adc_data;

  while (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    
  }
  while (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    
  }

  PRINT("#CMD [%d] ADC initialization complete ......\r\n", HAL_GetTick());

  osDelay(1);

  /* Infinite loop */
  for (;;)
  {
    taskENTER_CRITICAL();
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_data = HAL_ADC_GetValue(&hadc1);
    taskEXIT_CRITICAL();

    if (SystemMode == Output)
    {
      taskENTER_CRITICAL();
      PRINT("#DATA_ADC [%d] %4d\r\n", HAL_GetTick(), adc_data);
      taskEXIT_CRITICAL();
    }
    else if (SystemMode == Command)
    {
      if (ADC_Output == On)
      {
        PRINT("#CMD %4d\r\n", adc_data);
      }
      else if (Temp_Output == On)
      {
        while (HAL_ADCEx_RegularStop(&hadc1) != HAL_OK)
        {
          
        }

        if (HAL_ADCEx_InjectedStart(&hadc1) == HAL_OK)
        {
          taskENTER_CRITICAL();
          HAL_ADCEx_InjectedPollForConversion(&hadc1, HAL_MAX_DELAY);
          adc_data = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
          taskEXIT_CRITICAL();
          
          while (HAL_ADCEx_InjectedStop(&hadc1) != HAL_OK)
          {
            
          }

          //const uint16_t ta_cal1 = *((uint16_t*)0x1FFF75A8);
          //const uint16_t ta_cal2 = *((uint16_t*)0x1FFF75CA);
          PRINT("#CMD %4.0lf°C\r\n", 0.3053 * (((double)adc_data * 1.1) - (double)1037) + 30);

          osDelayUntil(&time, 490);
        }
        else
        {
          PRINT("#CMD Injected ADC start failed ......\r\n");
        }

        while (HAL_ADC_Start(&hadc1) != HAL_OK)
        {
          
        }
      }
    }

    osDelayUntil(&time, 10);
  }
  /* USER CODE END StartADCTask */
}

/* StartPrintfGateKeeperTask function */
void StartPrintfGateKeeperTask(void const * argument)
{
  /* USER CODE BEGIN StartPrintfGateKeeperTask */
  osEvent message;
  Packet_TypeDef *packet;
  
  /* Infinite loop */
  for(;;)
  {
    message = osMessageGet(printQueueHandle, osWaitForever);
    if (message.status == osEventMessage)
    {
      packet = (Packet_TypeDef *)message.value.v;

      //      while (HAL_UART_Transmit(&huart1, (uint8_t *)packet->pString, packet->size, 1000) != HAL_OK)
      //      {
      //        osDelay(1);
      //      }

      if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)packet->pString, packet->size) == HAL_OK)
      {
        // Wait for DMA transmit complete.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      }
      
      osPoolFree(packetPoolHandle, (void *)message.value.v);
    }
  }
  /* USER CODE END StartPrintfGateKeeperTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
