/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: virtual com port driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
 /******************************************************************************
  * @file    vcom.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    27-February-2017
  * @brief   Header for vcom.c module
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VCOM_H__
#define __VCOM_H__

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>

#include "terminal_if.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */ 

 /* --------------------------- USART HW definition -------------------------------*/
#define VCOM_USART                          USART1
#define VCOM_USART_CLK_ENABLE()             __HAL_RCC_USART1_CLK_ENABLE();
#define VCOM_USART_CLK_DISABLE()            __HAL_RCC_USART1_CLK_DISABLE();
#define VCOM_USART_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE();
#define VCOM_USART_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE();
//#define VCOM_USART_FORCE_RESET()           __USART2_FORCE_RESET()
//#define VCOM_USART_RELEASE_RESET			__USART2_RELEASE_RESET()
#define VCOM_USART_TX_PIN                   GPIO_PIN_9
#define VCOM_USART_TX_GPIO_PORT             GPIOA
#define VCOM_USART_RX_PIN                   GPIO_PIN_10
#define VCOM_USART_RX_GPIO_PORT             GPIOA

 /* Definition for USARTx's NVIC */
#define VCOM_USART_IRQn                    USART1_IRQn
#define VCOM_USART_IRQHandler              USART1_IRQHandler

typedef enum
{
    VCOM_NORMAL,
    VCOM_DEBUG,
    VCOM_OFF
}eVcomStates_t;

void _Error_Handler(char *, int);


 /**
* @brief  Init the VCOM.
* @param  Callback for the handle command
* @return None
*/
void vcom_Init(void (*cmdCallback)(sTerminalInterface_t *termIf, char *cmd));

/**
* @brief  Start the VCOM.
* @param  Vcom state from eVcomStates_t
* @return None
*/
void vcom_Start(eVcomStates_t state);

/**
* @brief  Disable the RX interrupt.
* @param  None
* @return None
*/
void vcom_DisableRx(void);

/**
* @brief  DeInit the VCOM.
* @param  None
* @return None
*/
void vcom_DeInit(void);

   /** 
* @brief  Init the VCOM IOs.
* @param  None
* @return None
*/
void vcom_IoInit(void);
  
   /** 
* @brief  DeInit the VCOM IOs.
* @param  None
* @return None
*/
void vcom_IoDeInit(void);

void vcom_handleByte(uint8_t byte);
uint8_t vcom_run();
void vcom_sleep();
sTerminalInterface_t *vcom_getInterface();

void diag_dump_buf(void *p, uint32_t s);

#define PRINTF printf

#define printBuff(_c)  printf(#_c " : 0x%08X\n", (unsigned int)READ_REG(_c));

#ifndef VT100_OFF
#define COLOR(__c,__x)    "\x1b[3" #__c "m" __x "\x1b[0m"
#define COLOR_BOLD(__c,__x)    "\x1b[3" #__c ";1m" __x "\x1b[0m"
#define UNDERLINE(__x) "\x1b[4m" __x "\x1b[0m"
#define CLEAR_SCREEN    "\x1b[2J\x1b[H"
#else
#define COLOR(__c,__x)    __x
#define COLOR_BOLD(__c,__x) __x
#define UNDERLINE(__x) __x
#define CLEAR_SCREEN
#endif
#define RED(__x)        COLOR(1, __x )
#define GREEN(__x)        COLOR(2, __x )
#define YELLOW(__x)        COLOR(3, __x )
#define BLUE(__x)        COLOR(4, __x )
#define MAGENTA(__x)    COLOR(5, __x )
#define CYAN(__x)        COLOR(6, __x )
#define RED_B(__x)        COLOR_BOLD(1, __x )
#define GREEN_B(__x)        COLOR_BOLD(2, __x )
#define YELLOW_B(__x)        COLOR_BOLD(3, __x )
#define BLUE_B(__x)        COLOR_BOLD(4, __x )
#define MAGENTA_B(__x)    COLOR_BOLD(5, __x )
#define CYAN_B(__x)        COLOR_BOLD(6, __x )

#ifdef __cplusplus
}
#endif

#endif /* __VCOM_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
