/**
  * @file   user.c
  * @author Kong Zelun
  * @date   2017-03-21
  * @brief  Basic io and time control.
  */

/**
  * @region Includes
  * @{
  */
#include "user.h"
#include <stdio.h>
#include "usart.h"

#pragma import(__use_no_semihosting)
/**
  * @}
  */

/**
  * @region Private Types
  * @{
  */
struct __FILE
{
  int handle;
};

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
  * @region Globle Variables
  * @{
  */
FILE __stdout;
FILE __stdin;

/**
  * @}
  */
  
/**
  * @region Public Functions
  * @{
  */
int _sys_exit(int x)
{
  return x;
}

#if defined ( RELEASE )

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int fgetc(FILE *f)
{
  uint8_t ch;
  while (HAL_UART_DMAStop(&huart1) != HAL_OK);
  HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

#elif defined ( DEBUG )

#define ITM_Port8(n)    (*((volatile unsigned char  *)(0xE0000000 + 4 * n)))
#define ITM_Port16(n)   (*((volatile unsigned short *)(0xE0000000 + 4 * n)))
#define ITM_Port32(n)   (*((volatile unsigned long  *)(0xE0000000 + 4 * n)))

#define DEMCR     (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA    (0x01000000)

int fputc(int ch, FILE *f)
{
  if (DEMCR & TRCENA)
  {
    while(ITM_Port32(0) == 0);
    ITM_Port8(0) = ch;
  }
  return ch;
}

#else
  #error fputc is not be implemented.
#endif

void Delay(__IO uint32_t Delay)
{
  for (int i = 0; i < Delay; i++)
  {
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP();
  }
}

/**
  * @}
  */
