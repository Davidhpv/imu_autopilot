/******************************************************************************
 * @file:    cmsis_test.h
 * @purpose: CMSIS Cortex-M3 Core Peripheral Access Layer Header File
 * @version: V1.20
 * @date:    29. Apr. 2009
 *----------------------------------------------------------------------------
 *
 * Copyright (C) 2009 ARM Limited. All rights reserved.
 *
 * ARM Limited (ARM) is supplying this software for use with Cortex-M3 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

#ifndef __CMSIS_TEST_H__
#define __CMSIS_TEST_H__

#include <stdint.h>

#ifndef TRUE
  #define TRUE      1
#endif

#ifndef FALSE
  #define FALSE     0
#endif


#define HAL_DEBUG_MSG           1       /*!< set to 1 to enable test protocol output through ITM          */
#define HAL_DEBUG_RET_ON_ERR    FALSE   /*!< Return on Error                                              */
#define SYSTICK_TEST_SECONDS    5       /*!< set to the number of seconds the SysTick Timer should run    */


#if defined  (HAL_DEBUG_MSG) && (HAL_DEBUG_MSG == 1)
  #include <stdio.h>
  #include <stdarg.h>
  
  extern char buf[100];
  extern char *p_buf;
#endif



#if (defined  (HAL_DEBUG_MSG) && (HAL_DEBUG_MSG == 1) )
  #define DBG_PRINTF(...)     {                               \
                                sprintf(buf, __VA_ARGS__);    \
                                dbg_out (buf);                \
                              }
#else
  #define DBG_PRINTF(...)     
#endif


/* CM3 HAL Self test */
uint32_t CM3HAL_SelfTest(void);
void dbg_out (char *s);


#endif
