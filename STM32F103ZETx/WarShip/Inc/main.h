/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOE
#define OLED_D0_Pin GPIO_PIN_0
#define OLED_D0_GPIO_Port GPIOC
#define OLED_D1_Pin GPIO_PIN_1
#define OLED_D1_GPIO_Port GPIOC
#define OLED_D2_Pin GPIO_PIN_2
#define OLED_D2_GPIO_Port GPIOC
#define OLED_D3_Pin GPIO_PIN_3
#define OLED_D3_GPIO_Port GPIOC
#define OLED_D4_Pin GPIO_PIN_4
#define OLED_D4_GPIO_Port GPIOC
#define OLED_D5_Pin GPIO_PIN_5
#define OLED_D5_GPIO_Port GPIOC
#define LCD_BL_Pin GPIO_PIN_0
#define LCD_BL_GPIO_Port GPIOB
#define OLED_D6_Pin GPIO_PIN_6
#define OLED_D6_GPIO_Port GPIOC
#define OLED_D7_Pin GPIO_PIN_7
#define OLED_D7_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_3
#define OLED_DC_GPIO_Port GPIOD
#define OLED_CS_Pin GPIO_PIN_6
#define OLED_CS_GPIO_Port GPIOD
#define OLED_RD_Pin GPIO_PIN_13
#define OLED_RD_GPIO_Port GPIOG
#define OLED_WR_Pin GPIO_PIN_14
#define OLED_WR_GPIO_Port GPIOG
#define OLED_RST_Pin GPIO_PIN_15
#define OLED_RST_GPIO_Port GPIOG
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_8
#define BEEP_GPIO_Port GPIOB
#define IR_Pin GPIO_PIN_9
#define IR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define OUTPUT_SIZE 256

typedef enum
{
  Output_ON = 0,
  Output_OFF
} OutputCtrl_TypeDef;

typedef enum
{
  PacketType_TEXT = 0x00,
  PacketType_TEXT_ONELINE,
  PacketType_KEY,
  PacketType_ADC,
  PacketType_CPU_USAGE,
  PacketType_HEAP_USAGE,
  PacketType_TIME,
  PacketType_CMD
}PacketType_TypeDef;

typedef struct
{
  PacketType_TypeDef type;
  unsigned int size;
  char pString[OUTPUT_SIZE];
} Packet_TypeDef;

extern void Error_Handler(void);
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
