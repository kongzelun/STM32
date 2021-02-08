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
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cpu_utils.h"
#include "usart.h"
#include "cmdline.h"
#include "iwdg.h"
#include "adc.h"
#include "rtc.h"
#include "infrared.h"
#include "oled.h"
#include "DIALOG.h"

//#define PRINT(S, ...)                                                         \
//  do                                                                          \
//  {                                                                           \
//    Packet_TypeDef *packet = (Packet_TypeDef *)osPoolAlloc(packetPoolHandle); \
//    if (packet != NULL)                                                       \
//    {                                                                         \
//      packet->size = sprintf(packet->pString, S, ##__VA_ARGS__);              \
//      if (packet->size <= 0 ||                                                \
//          osMessagePut(printQueueHandle, (uint32_t)packet, 1) != osOK)        \
//      {                                                                       \
//        osPoolFree(packetPoolHandle, (void *)packet);                         \
//      }                                                                       \
//    }                                                                         \
//  } while (0)

//#define PRINT2(S, ...)                                                        \
//  do                                                                          \
//  {                                                                           \
//    Packet_TypeDef *packet = (Packet_TypeDef *)osPoolAlloc(packetPoolHandle); \
//    if (packet != NULL)                                                       \
//    {                                                                         \
//      packet->size = sprintf(packet->pString, S, ##__VA_ARGS__);              \
//      if (packet->size <= 0 ||                                                \
//          osMessagePut(print2QueueHandle, (uint32_t)packet, 1) != osOK)       \
//      {                                                                       \
//        osPoolFree(packetPoolHandle, (void *)packet);                         \
//      }                                                                       \
//    }                                                                         \
//  } while (0)

//#define LCD_SEND(T, S, ...)                                                   \
//  do                                                                          \
//  {                                                                           \
//    Packet_TypeDef *packet = (Packet_TypeDef *)osPoolAlloc(packetPoolHandle); \
//    if (packet != NULL)                                                       \
//    {                                                                         \
//      packet->size = sprintf(packet->pString, S, ##__VA_ARGS__);              \
//      packet->type = T;                                                       \
//      if (packet->size <= 0 ||                                                \
//          osMessagePut(lcdPrintQueueHandle, (uint32_t)packet, 1) != osOK)     \
//      {                                                                       \
//        osPoolFree(packetPoolHandle, (void *)packet);                         \
//      }                                                                       \
//    }                                                                         \
//  } while (0)

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId cmdTaskHandle;
osThreadId mpuTaskHandle;
osThreadId adcTaskHandle;
osThreadId printGateKeeperTaskHandle;
osThreadId print2GateKeeperTaskHandle;
osMessageQId printQueueHandle;
osMessageQId print2QueueHandle;
osSemaphoreId printBinarySemHandle;
osSemaphoreId cmdBinarySemHandle;

/* USER CODE BEGIN Variables */
osThreadId guiTaskHandle;
osThreadId key0TaskHandle;
osThreadId infraredTaskHandle;
osMessageQId lcdPrintQueueHandle;
osMessageQId infraredQueueHandle;
  
const HeapRegion_t HeapRegions[] = {
  { (uint8_t *)0x20008000UL, 0x8000 },
  { (uint8_t *)0x68000000UL, 0x80000 },
  { NULL, 0 }
};

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
void StartPrintGateKeeperTask(void const * argument);
void StartPrint2GateKeeperTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void StartGUITask(void const * argument);
void StartKey0Task(void const * argument);
void StartInfraredTask(void const * argument);

extern WM_HWIN CreateFramewin(void);
extern WM_HWIN CreateWindow(void);
/* USER CODE END FunctionPrototypes */

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

__STATIC_INLINE int print(char *format, ...)
{
  va_list args;
  va_start(args, format);
  
  Packet_TypeDef *packet = (Packet_TypeDef *)osPoolAlloc(packetPoolHandle);
  if (packet != NULL)
  {
    packet->size = vsprintf(packet->pString, format, args);
    if (packet->size <= 0 || osMessagePut(printQueueHandle, (uint32_t)packet, 1) != osOK)
    {
      osPoolFree(packetPoolHandle, (void *)packet);
    }
  }

  va_end(args);
  
  return packet->size;
}

__STATIC_INLINE int print2(char *format, ...)
{
  va_list args;
  va_start(args, format);
  
  Packet_TypeDef *packet = (Packet_TypeDef *)osPoolAlloc(packetPoolHandle);
  if (packet != NULL)
  {
    packet->size = vsprintf(packet->pString, format, args);
    if (packet->size <= 0 || osMessagePut(print2QueueHandle, (uint32_t)packet, 1) != osOK)
    {
      osPoolFree(packetPoolHandle, (void *)packet);
    }
  }

  va_end(args);
  
  return packet->size;
}

__STATIC_INLINE int lcd_sendPacket(PacketType_TypeDef type, char *format, ...)
{
  va_list args;
  va_start(args, format);
  
  Packet_TypeDef *packet = (Packet_TypeDef *)osPoolAlloc(packetPoolHandle);
  if (packet != NULL)
  {
    uint32_t timeout = 1;
    packet->size = vsprintf(packet->pString, format, args);
    packet->type = type;
    
    if (type == PacketType_CMD)
    {
      timeout = osWaitForever;
    }
    
    if (packet->size <= 0 || osMessagePut(lcdPrintQueueHandle, (uint32_t)packet, timeout) != osOK)
    {
      osPoolFree(packetPoolHandle, (void*)packet);
    }
  }
  
  va_end(args);
  
  return packet->size;
}
/* USER CODE END 3 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  
  vPortDefineHeapRegions(HeapRegions);
  
  Usage_Output = Output_OFF;
  Euler_Output = Output_OFF;
  MpuTemp_Output = Output_OFF;
  ADC_Output = Output_OFF;
  Temp_Output = Output_OFF;
  Time_Output = Output_OFF;
  
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of printBinarySem */
  osSemaphoreDef(printBinarySem);
  printBinarySemHandle = osSemaphoreCreate(osSemaphore(printBinarySem), 1);

  /* definition and creation of cmdBinarySem */
  osSemaphoreDef(cmdBinarySem);
  cmdBinarySemHandle = osSemaphoreCreate(osSemaphore(cmdBinarySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of cmdTask */
  osThreadDef(cmdTask, StartCMDTask, osPriorityRealtime, 0, 128);
  cmdTaskHandle = osThreadCreate(osThread(cmdTask), NULL);

  /* definition and creation of mpuTask */
  osThreadDef(mpuTask, StartMPUTask, osPriorityHigh, 0, 256);
  mpuTaskHandle = osThreadCreate(osThread(mpuTask), NULL);

  /* definition and creation of adcTask */
  osThreadDef(adcTask, StartADCTask, osPriorityHigh, 0, 128);
  adcTaskHandle = osThreadCreate(osThread(adcTask), NULL);

  /* definition and creation of printGateKeeperTask */
  osThreadDef(printGateKeeperTask, StartPrintGateKeeperTask, osPriorityIdle, 0, 128);
  printGateKeeperTaskHandle = osThreadCreate(osThread(printGateKeeperTask), NULL);

  /* definition and creation of print2GateKeeperTask */
  osThreadDef(print2GateKeeperTask, StartPrint2GateKeeperTask, osPriorityIdle, 0, 128);
  print2GateKeeperTaskHandle = osThreadCreate(osThread(print2GateKeeperTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(guiTask, StartGUITask, osPriorityBelowNormal, 0, 0x1000);
  guiTaskHandle = osThreadCreate(osThread(guiTask), NULL);
  
  osThreadDef(key0Task, StartKey0Task, osPriorityRealtime, 0, 128);
  key0TaskHandle = osThreadCreate(osThread(key0Task), NULL);
  
  osThreadDef(infraredTask, StartInfraredTask, osPriorityRealtime, 0, 128);
  infraredTaskHandle = osThreadCreate(osThread(infraredTask), NULL);
  
  osPoolDef(PacketPool, 64, Packet_TypeDef);
  packetPoolHandle = osPoolCreate(osPool(PacketPool));
  if (packetPoolHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of printQueue */
  osMessageQDef(printQueue, 16, void*);
  printQueueHandle = osMessageCreate(osMessageQ(printQueue), NULL);

  /* definition and creation of print2Queue */
  osMessageQDef(print2Queue, 16, void*);
  print2QueueHandle = osMessageCreate(osMessageQ(print2Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  osMessageQDef(lcdPrintQueue, 16, void*);
  lcdPrintQueueHandle = osMessageCreate(osMessageQ(lcdPrintQueue), NULL);
  
  osMessageQDef(infraredQueue, 16, uint32_t);
  infraredQueueHandle = osMessageCreate(osMessageQ(infraredQueue), NULL);
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */

  while (HAL_RTC_WaitForSynchro(&hrtc) != HAL_OK)
  {
    osDelay(1);
  }
  
  print2("#CMD [%d] System initialization complete ......\r\n", HAL_GetTick());
  lcd_sendPacket(PacketType_TEXT, "#CMD [%d] System initialization complete ......\r\n", HAL_GetTick());
  
  /* Infinite loop */
  for (;;)
  {
    //HAL_IWDG_Refresh(&hiwdg);
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

    HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BIN);
//    OLED_printf(OLED_CHAR_SIZE_1608, "%04d-%02d-%02d", 2000 + RTC_Date.Year, RTC_Date.Month, RTC_Date.Date);
//    OLED_printf(OLED_CHAR_SIZE_1608, "\r\n%02d:%02d:%02d", RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
//    OLED_printf(OLED_CHAR_SIZE_1608, "\r\n\r\nCPU: %3d%%", osGetCPUUsage());
//    OLED_printf(OLED_CHAR_SIZE_1608, "\r\n\r\n\r\n%6d Bytes", xPortGetFreeHeapSize());
    
    lcd_sendPacket(PacketType_CPU_USAGE, "%d", osGetCPUUsage());
    lcd_sendPacket(PacketType_HEAP_USAGE, "%6d Bytes", xPortGetFreeHeapSize());
    lcd_sendPacket(PacketType_TIME, "%04d-%02d-%02d %02d:%02d:%02d", 2000 + RTC_Date.Year, RTC_Date.Month, RTC_Date.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
    
    if (Usage_Output == Output_ON)
    {
      print2("#CMD [%d] CPU: %d%% Heap: %d Bytes\r", HAL_GetTick(), osGetCPUUsage(), xPortGetFreeHeapSize());
      lcd_sendPacket(PacketType_TEXT_ONELINE, "#CMD [%d] CPU: %d%% Heap: %d Bytes\n", HAL_GetTick(), osGetCPUUsage(), xPortGetFreeHeapSize());
    }
    else if (Time_Output == Output_ON)
    {
      HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);
      HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BIN);
      
      print2("#CMD [%d] %04d-%02d-%02d %02d:%02d:%02d\r", HAL_GetTick(), 2000 + RTC_Date.Year, RTC_Date.Month, RTC_Date.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
      lcd_sendPacket(PacketType_TEXT_ONELINE, "#CMD [%d] %04d-%02d-%02d %02d:%02d:%02d\n", HAL_GetTick(), 2000 + RTC_Date.Year, RTC_Date.Month, RTC_Date.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
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
//  Packet_TypeDef *lcdPacket;

  while (HAL_UART_Receive_IT(&huart2, &USART2_Buffer, 1) != HAL_OK)
  {
    osDelay(1);
  }

  print2("\r\n\r\n#CMD [%d] Cmd initialization complete ......\r\n", HAL_GetTick());
  lcd_sendPacket(PacketType_TEXT, "#CMD [%d] Cmd initialization complete ......\n", HAL_GetTick());
  
  /* Infinite loop */
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    packet = (Packet_TypeDef *)osPoolAlloc(packetPoolHandle);
    if (packet != NULL)
    {
      CMD_Process(USART2_Buffer, packet->pString, &packet->size);
      if (packet->size <= 0 || osMessagePut(print2QueueHandle, (uint32_t)packet, osWaitForever) != osOK)
      {
        osPoolFree(packetPoolHandle, (void *)packet);
        packet = NULL;
      }
    }
    
//    lcdPacket = (Packet_TypeDef *)osPoolAlloc(packetPoolHandle);
//    if(lcdPacket != NULL && packet != NULL)
//    {
//      *lcdPacket = *packet;
//      lcdPacket->type = PacketType_TEXT;
//      if (osMessagePut(lcdPrintQueueHandle, (uint32_t)lcdPacket, osWaitForever) != osOK)
//      {
//        osPoolFree(packetPoolHandle, (void *)lcdPacket);
//      }
//    }
    
    while (HAL_UART_Receive_IT(&huart2, &USART2_Buffer, 1) != HAL_OK)
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
  osThreadSuspend(NULL);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartMPUTask */
}

/* StartADCTask function */
void StartADCTask(void const * argument)
{
  /* USER CODE BEGIN StartADCTask */
  uint32_t time = osKernelSysTick();
  int adc_data;

  while (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    osDelay(1);
  }
  while (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    osDelay(1);
  }

  print2("#CMD [%d] ADC initialization complete ......\r\n", HAL_GetTick());
  lcd_sendPacket(PacketType_TEXT, "#CMD [%d] ADC initialization complete ......\n", HAL_GetTick());

  osDelay(1);

  /* Infinite loop */
  for (;;)
  {
    while (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK)
    {
      osDelay(1);
    }

    adc_data = HAL_ADC_GetValue(&hadc1);

    if (ADC_Output == Output_ON)
    {
      print2("#CMD [%d] %4d\r\n", HAL_GetTick(), adc_data);
      lcd_sendPacket(PacketType_ADC, " %4d", adc_data);
    }
    else if (Temp_Output == Output_ON)
    {
      while (HAL_ADC_Stop(&hadc1) != HAL_OK)
      {
        osDelay(1);
      }

      if (HAL_ADCEx_InjectedStart(&hadc1) == HAL_OK)
      {
        HAL_ADCEx_InjectedPollForConversion(&hadc1, HAL_MAX_DELAY);
        adc_data = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        while (HAL_ADCEx_InjectedStop(&hadc1) != HAL_OK)
        {
          osDelay(1);
        }
        
        /**
         *  Temp = ((V25 - Vsensor)/ArgSlope) + 25
         *  V25 = 1.34V ~ 1.52V (typical 1.43V)
         *  ArgSlope = 4.0mV ~ 4.6mV (typical 4.3mV)
        **/
        double temp = (double)(1774 - adc_data) / 17.0 + 25;
        print2("#CMD [%d] %4.1lf°C\r\n", HAL_GetTick(), temp);
        //lcd_sendPacket(PacketType_ADC, " %4.1lf", temp);

        osDelayUntil(&time, 490);
        //osDelay(480);
      }
      else
      {
        print("#CMD [%d] Injected ADC start failed ......\r\n", HAL_GetTick());
      }

      while (HAL_ADC_Start(&hadc1) != HAL_OK)
      {
        osDelay(1);
      }
    }

    osDelayUntil(&time, 10);
  }
  /* USER CODE END StartADCTask */
}

/* StartPrintGateKeeperTask function */
void StartPrintGateKeeperTask(void const * argument)
{
  /* USER CODE BEGIN StartPrintGateKeeperTask */
  osThreadSuspend(NULL);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartPrintGateKeeperTask */
}

/* StartPrint2GateKeeperTask function */
void StartPrint2GateKeeperTask(void const * argument)
{
  /* USER CODE BEGIN StartPrint2GateKeeperTask */
  osEvent message;
  Packet_TypeDef *packet;
  
  /* Infinite loop */
  for(;;)
  {
    message = osMessageGet(print2QueueHandle, osWaitForever);
    if (message.status == osEventMessage)
    {
      packet = (Packet_TypeDef *)message.value.v;

      //      while (HAL_UART_Transmit(&huart1, (uint8_t *)packet->pString, packet->size, 1000) != HAL_OK)
      //      {
      //        osDelay(1);
      //      }

      if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)packet->pString, packet->size) == HAL_OK)
      {
        // Wait for DMA transmit complete.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      }
      
      osPoolFree(packetPoolHandle, (void *)message.value.v);
    }
  }
  /* USER CODE END StartPrint2GateKeeperTask */
}

/* USER CODE BEGIN Application */
void StartGUITask(void const * argument)
{
  int GRAPH_DATA_LENGTH = 300;
  int GRAPH_Y_PIXEL_NUM = 205;
//  float GRAPH_Y_FACTOR = 200.0f / 4095.0f;
  

  WM_HWIN hMainWindow;
  GUI_HWIN hTextTime;
  GUI_HWIN hTextHeapUsage;
  GUI_HWIN hProgBar;
  GUI_HWIN hMultiEdit;
  GUI_HWIN hGraph;
  
  osEvent message;
  Packet_TypeDef *packet;
  short *pGraphData;
  
  GUI_Init();
  
  pGraphData = pvPortMalloc(GRAPH_DATA_LENGTH * sizeof(short));
  if (pGraphData == NULL)
  {
    Error_Handler();
  }
  
  hMainWindow = CreateWindow();
  hTextTime = WM_GetDialogItem(hMainWindow, (GUI_ID_USER + 0x05));
  hTextHeapUsage = WM_GetDialogItem(hMainWindow, (GUI_ID_USER + 0x06));
  hProgBar = WM_GetDialogItem(hMainWindow, (GUI_ID_USER + 0x02));
  
  hMultiEdit = WM_GetDialogItem(hMainWindow, (GUI_ID_USER + 0x03));
  MULTIEDIT_SetTextColor(hMultiEdit, MULTIEDIT_CI_EDIT, GUI_WHITE);
  MULTIEDIT_SetBkColor(hMultiEdit, MULTIEDIT_CI_EDIT, GUI_BLACK);
  //MULTIEDIT_SetBufferSize(hMultiEdit, 1024);
  //MULTIEDIT_SetMaxNumChars(hMultiEdit, 1024);
  //MULTIEDIT_SetPrompt(hMultiEdit, ">> ");
  
  hGraph = WM_GetDialogItem(hMainWindow, (GUI_ID_USER + 0x04));
  GRAPH_DATA_Handle hGD1 = GRAPH_DATA_YT_Create(GUI_RED, GRAPH_DATA_LENGTH, pGraphData, 0);
  GRAPH_AttachData(hGraph, hGD1);
  GRAPH_SCALE_Handle hGSy = GRAPH_SCALE_Create(35, GUI_TA_RIGHT, GRAPH_SCALE_CF_VERTICAL, GRAPH_Y_PIXEL_NUM / 3.3);
  //GRAPH_SCALE_Handle hGSx = GRAPH_SCALE_Create(209, GUI_TA_BOTTOM, GRAPH_SCALE_CF_HORIZONTAL, 50);
  GRAPH_SCALE_SetFactor(hGSy, 3.3f / (float)GRAPH_Y_PIXEL_NUM);
  GRAPH_SCALE_SetTickDist(hGSy, GRAPH_Y_PIXEL_NUM / (3.3f * 2));
  GRAPH_SCALE_SetNumDecs(hGSy, 1);
  GRAPH_AttachScale(hGraph, hGSy);
  //GRAPH_AttachScale(hGraph, hGSx);
  GRAPH_SetGridDistY(hGraph, GRAPH_Y_PIXEL_NUM / (3.3f * 2));
  GRAPH_SetGridVis(hGraph, 1);
  
  WM_BringToTop(hMultiEdit);
  
  /* Infinite loop */
  for (;;)
  {
    message = osMessageGet(lcdPrintQueueHandle, osWaitForever);
    if (message.status == osEventMessage)
    {
      packet = (Packet_TypeDef *)message.value.v;
      switch (packet->type)
      {
        case PacketType_ADC:
        {
          int value = atoi(packet->pString);
          GRAPH_DATA_YT_AddValue(hGD1, (short)(value * GRAPH_Y_PIXEL_NUM / 4095));
          break;
        }
        case PacketType_CPU_USAGE:
        {
          int value = atoi(packet->pString);
          PROGBAR_SetValue(hProgBar, value);
          break;
        }
        case PacketType_HEAP_USAGE:
          TEXT_SetText(hTextHeapUsage, packet->pString);
          break;
        case PacketType_TEXT:
        {
          MULTIEDIT_AddText(hMultiEdit, packet->pString);
          break;
        }
        case PacketType_TEXT_ONELINE:
        {
          //MULTIEDIT_SetPrompt(hMultiEdit, (const char *)packet->pString);
//          MULTIEDIT_SetCursorOffset(hMultiEdit, 0);
//          MULTIEDIT_SetInsertMode(hMultiEdit, 1);
//          MULTIEDIT_AddText(hMultiEdit, packet->pString);
//          MULTIEDIT_SetInsertMode(hMultiEdit, 0);
          MULTIEDIT_AddText(hMultiEdit, packet->pString);
          break;
        }
        case PacketType_TIME:
          TEXT_SetText(hTextTime, packet->pString);
          break;
        case PacketType_CMD:
        {
          if (strcasecmp(packet->pString, "graph") == 0)
          {
            WM_BringToTop(hGraph);
          }
          else if (strcasecmp(packet->pString, "multiedit") == 0)
          {
            WM_BringToTop(hMultiEdit);
          }
          break;
        }
        default:
          break;
      }
      osPoolFree(packetPoolHandle, (void *)message.value.v);
    }
    
    GUI_Exec();
    GUI_X_ExecIdle();
  }
}

void StartKey0Task(void const * argument)
{
  int flag = 0;
  
  /* Infinite loop */
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    if (flag % 2 == 0)
    {
      lcd_sendPacket(PacketType_CMD, "graph");
    }
    else
    {
      lcd_sendPacket(PacketType_CMD, "multiedit");
    }
    
    flag++;
    
    //    for (int times = 0; times <= 10; times++)
    //    {
    //      if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) != GPIO_PIN_RESET)
    //      {
    //        times = 0;
    //      }
    //      osDelay(10);
    //    }
    
    osDelay(500);
    SET_BIT(EXTI->IMR, KEY0_Pin);
  }
}

void StartInfraredTask(void const * argument)
{
  osEvent message;
  
  /* Infinite loop */
  for (;;)
  {
    message = osMessageGet(infraredQueueHandle, osWaitForever);
    
    if (message.status == osEventMessage)
    {
      OLED_printf(OLED_CHAR_SIZE_1608, "%08X", message.value.v);
      switch (message.value.v)
      {
        case 0x00FB38C7:
          // 开关
          HAL_NVIC_SystemReset();
          break;
        case 0x00FB50AF:
          // 返回
          vTaskNotifyGiveFromISR(key0TaskHandle, NULL);
          break;
        case 0x00FB807F:
        {
          // red
          Usage_Output = Output_OFF;
          MpuTemp_Output = Output_OFF;
          Euler_Output = Output_OFF;
          Temp_Output = Output_OFF;
          Time_Output = Output_OFF;
          ADC_Output = (ADC_Output == Output_ON)?(Output_OFF):(Output_ON);
          break;
        }
        default:
          break;
      }
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY0_Pin)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    CLEAR_BIT(EXTI->IMR, KEY0_Pin);
    vTaskNotifyGiveFromISR(key0TaskHandle, &xHigherPriorityTaskWoken);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{  
  if (huart == &huart2)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    vTaskNotifyGiveFromISR(print2GateKeeperTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    
  }
  else if (huart == &huart2)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    vTaskNotifyGiveFromISR(cmdTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == hinfrared1.Init.htim)
  {
    switch (htim->Channel)
    {
      case HAL_TIM_ACTIVE_CHANNEL_1:
      {
        break;
      }
      case HAL_TIM_ACTIVE_CHANNEL_2:
      {
        break;
      }
      case HAL_TIM_ACTIVE_CHANNEL_3:
      {
        INFRARED_RasingEdgeHandler(&hinfrared1);
        break;
      }
      case HAL_TIM_ACTIVE_CHANNEL_4:
      {
        INFRARED_FallingEdgeHandler(&hinfrared1);
        break;
      }
      default:
        break;
    }
  }
}

void INFRARED_DataReadyCallback(INFRARED_HandleTypeDef *hinfrared)
{
  if (hinfrared == &hinfrared1)
  {
    osMessagePut(infraredQueueHandle, hinfrared->Code.LastCode, osWaitForever);
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
