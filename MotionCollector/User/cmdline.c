/**
  * @file   cmdline.c
  * @author Kong Zelun
  * @date   2017-03-21
  * @brief  
  */

/**
  * @region Includes
  * @{
  */
#include "cmdline.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"

/**
  * @}
  */

/**
  * @region Private Defines
  * @{
  */

#if !defined(OUTPUT_SIZE)
  #define OUTPUT_SIZE 256
#endif
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
  * @region Exported Variables
  * @{
  */
extern OutputCtrl_TypeDef Usage_Output;
extern OutputCtrl_TypeDef Euler_Output;
extern OutputCtrl_TypeDef MpuTemp_Output;
extern OutputCtrl_TypeDef ADC_Output;
extern OutputCtrl_TypeDef Temp_Output;
extern OutputCtrl_TypeDef Time_Output;

/**
  * @}
  */

/**
  * @region Global Variables
  * @{
  */
static char start[5] = "\r\n>> ";
  
struct
{
  uint8_t buffer[OUTPUT_SIZE];
  int count;
}CMD_Buffer;

enum
{
  CMD_READY = 0,
  CMD_PROCESSING,
  CMD_BUSY
}CMD_State;

/**
  * @}
  */

/**
  * @region Private Functions Prototype
  * @{
  */
static void CMD_Reset(void);
static int CMD_Excute(char *cmd);

/**
  * @}
  */

/**
  * @region Public Functions
  * @{
  */

/**
  * @brief  Process command.
  * @param  None.
  * @retval None.
  */
void CMD_Process(char ch, char *stringToPrint, unsigned int *size)
{
  // stringToPrint 's length.
  uint32_t length = 0;
  
  if (ch == 0x03)
  {
    // Ctrl-C
    Euler_Output = Off;
    MpuTemp_Output = Off;
    Usage_Output = Off;
    ADC_Output = Off;
    Temp_Output = Off;
    Time_Output = Off;
    CMD_Reset();
    length = _sprintf(stringToPrint, "%s", start);
  }
  else if (CMD_State == CMD_READY)
  {
    if (ch >= ' ' && ch <= '~')
    {
      CMD_Buffer.buffer[CMD_Buffer.count] = ch;
      CMD_Buffer.count ++;
      //HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1);
      *stringToPrint = ch;
      length ++;
      CMD_State = CMD_PROCESSING;
    }
    else if (ch == '\r')
    {
      CMD_Reset();
      length = _sprintf(stringToPrint, "%s", start);
    }
  }
  else if (CMD_State == CMD_PROCESSING)
  {
    if (ch == '\r')
    {
      CMD_Buffer.buffer[CMD_Buffer.count] = '\0';
      if (CMD_Excute((char *)CMD_Buffer.buffer))
      {
        //printf("\r\n\'%s\' is not a command. \r\n", CMD_Buffer.buffer);
        CMD_Reset();
        length = _sprintf(stringToPrint, "\r\n\'%s\' is not a command. \r\n%s", CMD_Buffer.buffer, start);
      }
    }
    else if (ch >= ' ' && ch <= '~')
    {
      if (CMD_Buffer.count < OUTPUT_SIZE / 2)
      {
        CMD_Buffer.buffer[CMD_Buffer.count] = ch;
        CMD_Buffer.count ++;
        *stringToPrint = ch;
        length ++;
        CMD_State = CMD_PROCESSING;
      }
    }
    else if (ch == 0x7F)
    {
      // Backspace
      if (CMD_Buffer.count <= 0)
      {
        CMD_Reset();
      }
      else
      {
        CMD_Buffer.count --;
        //HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
        *stringToPrint = ch;
        length ++;
      }
    }
  }
  else
  {
    // CMD_BUSY
    if (ch == '\r')
    {
      length = _sprintf(stringToPrint, "\r\n");
    }
  }
  
  stringToPrint[length] = '\0';
  *size = length;
}

/**
  * @}
  */

/**
  * @region Private Functions
  * @{
  */

/**
  * @brief  Determine whether the string is command, if it is, excute it, or show error.
  * @param  command
  * @retval None.
  */

/**
  * @brief  Start command line.
  * @param  None.
  * @retval None.
  */
static void CMD_Reset(void)
{
  CMD_Buffer.count = 0;
  CMD_State = CMD_READY;
}

static int CMD_Excute(char *cmd)
{
  Usage_Output = Off;
  MpuTemp_Output = Off;
  Euler_Output = Off;
  ADC_Output = Off;
  Temp_Output = Off;
  Time_Output = Off;
  
  printf("\r\n");
  if (strcasecmp(cmd, "usage") == 0)
  {
    Usage_Output = On;
  }
  else if (strcasecmp(cmd, "euler") == 0)
  {
    Euler_Output = On;
  }
  else if (strcasecmp(cmd, "mputemp") == 0)
  {
    MpuTemp_Output = On;
  }
  else if (strcasecmp(cmd, "adc") == 0)
  {
    ADC_Output = On;
  }
  else if (strcasecmp(cmd, "temp") == 0)
  {
    Temp_Output = On;
  }
  else if (strcasecmp(cmd, "time") == 0)
  {
    Time_Output = On;
  }
  else if (strcasecmp(cmd, "reset") == 0)
  {
    NVIC_SystemReset();
  }
  else
  {
    return -1;
  }
  
  CMD_State = CMD_BUSY;
  
  return 0;
}

/**
  * @}
  */
