 /******************************************************************************
  * @file    vcom.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    27-February-2017
  * @brief   manages virtual com port
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
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
  
#include "vcom.h"

#include "stm32f1xx_hal.h"

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define CMD_BUFFER_SIZE     128
#define VCOM_DELAY   		2*1000 //mS

static char mCmdBuffer[CMD_BUFFER_SIZE];
static uint8_t mCmdIndex = 0;

static eVcomStates_t mVcomState = VCOM_OFF;
static bool mTimingOut = false;
static bool mActive = false;
static char *mCommand;
static void (*mCmdCallback)(sTerminalInterface_t *termIf, char *cmd) = 0;


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BUFSIZE 256
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    printf(RED("Error @%s line: %d"), file, line);
    while (1)
    {
    }
}

/* Uart Handle */
static UART_HandleTypeDef UartHandle;
uint8_t vcomOK = 0;

static sTerminalInterface_t mVcomInterface;
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/

void vcom_write(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	char buffer[128];
	vsnprintf(buffer, 128, format, args);
	printf(buffer);
	va_end(args);
}

void vcom_sleep()
{
    mVcomInterface.printf(YELLOW("Terminal going sleep.\n"));
    vcom_DisableRx();
    mActive = false;
}

sTerminalInterface_t *vcom_getInterface()
{
    return &mVcomInterface;
}

void vcom_Init(void (*cmdCallback)(sTerminalInterface_t *termIf, char *cmd))
{
	mCmdCallback = cmdCallback;

    // if the com is already initialized return
    if(!vcomOK)
    {
        /*## Configure the UART peripheral ######################################*/
        /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
        /* UART1 configured as follow:
         - Word Length = 8 Bits
         - Stop Bit = One Stop bit
         - Parity = ODD parity
         - BaudRate = 921600 baud
         - Hardware flow control disabled (RTS and CTS signals)
         */
        UartHandle.Instance        = VCOM_USART;

        UartHandle.Init.BaudRate   = 115200;
        UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
        UartHandle.Init.StopBits   = UART_STOPBITS_1;
        UartHandle.Init.Parity     = UART_PARITY_NONE;
        UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
        UartHandle.Init.Mode       = UART_MODE_TX_RX;

        if(HAL_UART_Init(&UartHandle) != HAL_OK)
        {
            /* Initialization Error */
            _Error_Handler(__FILE__, __LINE__);
        }

        VCOM_USART->CR1 |= USART_CR1_RXNEIE;

        HAL_NVIC_SetPriority(VCOM_USART_IRQn, 0x1, 0);
        HAL_NVIC_EnableIRQ(VCOM_USART_IRQn);

        /* This allows printf to work without \n */
        setbuf(stdout, 0);

        mVcomInterface.printf = vcom_write;
        mVcomInterface.sleep = vcom_sleep;

        vcomOK = 1;
    }
}

void vcom_Start(eVcomStates_t state)
{
   mVcomState = state;

   switch (state)
   {
      case VCOM_NORMAL:
      {
         mActive = true;
         mTimingOut = true;
         mVcomInterface.printf("VCOM started\n");
         mVcomInterface.printf(GREEN("Press enter to enable Terminal..\n"));
      }
      break;
      case VCOM_DEBUG:
      {
         mActive = true;
         mTimingOut = false;
         mVcomInterface.printf("VCOM started\n");
         mVcomInterface.printf(GREEN("Type s to sleep terminal.\n"));
      }
      break;
      case VCOM_OFF:
      {
         mActive = false;
         mVcomInterface.printf(RED("VCOM disabled.\n"));
      }
      break;
   }
}

void vcom_DisableRx(void)
{
    /* Clear the interrupt enable */
    VCOM_USART->CR1 &= ~USART_CR1_RXNEIE;
}

void vcom_DeInit(void)
{
   vcomOK = 0;

#if 1
  HAL_UART_DeInit(&UartHandle);
#endif
}

int _write(int file, char *data, int len)
{
   if(!vcomOK)
      return -1;

   if(HAL_UART_Transmit(&UartHandle,(uint8_t *)data, len, 300)  != HAL_OK)
      return -1;

   if(data[len -1] == '\n')
   {
      uint8_t CR = '\r';

      if(HAL_UART_Transmit(&UartHandle, &CR, 1, 300)  != HAL_OK)
         return -1;
   }

   return len;
}

/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/

  /* Enable USART1 clock */
  VCOM_USART_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  vcom_IoInit( );
}

void vcom_IoInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct={0};
    /* Enable GPIO TX/RX clock */
  VCOM_USART_TX_GPIO_CLK_ENABLE();
  VCOM_USART_RX_GPIO_CLK_ENABLE();
    /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = VCOM_USART_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(VCOM_USART_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = VCOM_USART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(VCOM_USART_RX_GPIO_PORT, &GPIO_InitStruct);
}

void vcom_IoDeInit(void)
{
   GPIO_InitTypeDef GPIO_InitStructure={0};

   VCOM_USART_TX_GPIO_CLK_ENABLE();
   VCOM_USART_RX_GPIO_CLK_ENABLE();

   GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
   GPIO_InitStructure.Pull = GPIO_NOPULL;

   GPIO_InitStructure.Pin =  VCOM_USART_TX_PIN ;
   HAL_GPIO_Init(  VCOM_USART_TX_GPIO_PORT, &GPIO_InitStructure );

   GPIO_InitStructure.Pin =  VCOM_USART_RX_PIN ;
   HAL_GPIO_Init(  VCOM_USART_RX_GPIO_PORT, &GPIO_InitStructure );

   VCOM_USART_CLK_DISABLE();
}

uint8_t vcom_run()
{
    if (!mActive)
        return 0;

    //auto term off
    if(mTimingOut)
    {
    	static uint32_t timeOff = 0;
    	if(!timeOff)
    		timeOff = (HAL_GetTick() + VCOM_DELAY);

    	if(HAL_GetTick() > timeOff)
    	{
    		vcom_sleep();
    		return 0;
    	}
    }

    if (mCommand)
    {
    	if(mTimingOut)
    		mTimingOut = false;

        if(mCmdCallback)
            mCmdCallback(&mVcomInterface, mCommand);

    	free(mCommand);
    	mCommand = 0;
    }

    return 1;
}

void vcom_handleByte(uint8_t byte)
{
	if(!mActive)
        return;

    if (byte == '\r') // || (byte == '\n'))
    {
        mCmdBuffer[mCmdIndex++] = byte;
        mCmdBuffer[mCmdIndex++] = 0;
        if(mCommand)
        	return;
        mCommand = (char *)malloc(mCmdIndex);
        memcpy(mCommand, mCmdBuffer, mCmdIndex);

        mCmdIndex = 0;
    }
    else
        mCmdBuffer[mCmdIndex++] = byte;

    if (mCmdIndex > CMD_BUFFER_SIZE)
        mCmdIndex = 0;
}


/**
  * @brief UART MSP DeInit
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  vcom_IoDeInit( );
}

//int putch(int ch, FILE *f) {
//
//   if(HAL_UART_Transmit(&UartHandle,(uint8_t *)&ch, 1, 300) != HAL_OK)
//      return -1;
//
//   return 1;
//}

void
diag_vdump_buf_with_offset(
      uint8_t    *p,
      uint32_t   s,
      uint8_t    *base
      )
{
    int i, c;
    if ((uint32_t)s > (uint32_t)p) {
        s = (uint32_t)s - (uint32_t)p;
    }
    while ((int)s > 0) {
        if (base) {
            printf("%08X: ", (int)((uint32_t)p - (uint32_t)base));
        } else {
            printf("%08X: ", (int)p);
        }
        for (i = 0;  i < 16;  i++) {
            if (i < (int)s) {
                printf("%02X ", p[i] & 0xFF);
            } else {
                printf("   ");
            }
        if (i == 7) printf(" ");
        }
        printf(" |");
        for (i = 0;  i < 16;  i++) {
            if (i < (int)s) {
                c = p[i] & 0xFF;
                if ((c < 0x20) || (c >= 0x7F)) c = '.';
            } else {
                c = ' ';
            }
            printf("%c", c);
        }
        printf("|\n\r");
        s -= 16;
        p += 16;
    }
}

void
diag_dump_buf_with_offset(
      uint8_t    *p,
      uint32_t   s,
      uint8_t    *base
      )
{
    diag_vdump_buf_with_offset(p, s, base);
}

void diag_dump_buf(void *p, uint32_t s)
{
   diag_dump_buf_with_offset((uint8_t *)p, s, 0);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
