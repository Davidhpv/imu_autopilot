/**************************************************************************//**
 * @file     core_cm0.c
 * @brief    CMSIS Cortex-M0 Core Peripheral Access Layer Source File
 * @version  V1.40
 * @date     15. February 2010
 *
 * @note
 * Copyright (C) 2009-2010 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

#include <stdint.h>

/* define compiler specific symbols */
#if defined ( __CC_ARM   )
  #define __ASM            __asm                                      /*!< asm keyword for ARM Compiler          */
  #define __INLINE         __inline                                   /*!< inline keyword for ARM Compiler       */

#elif defined ( __ICCARM__ )
  #define __ASM           __asm                                       /*!< asm keyword for IAR Compiler          */
  #define __INLINE        inline                                      /*!< inline keyword for IAR Compiler. Only avaiable in High optimization mode! */

#elif defined   (  __GNUC__  )
  #define __ASM            __asm                                      /*!< asm keyword for GNU Compiler          */
  #define __INLINE         inline                                     /*!< inline keyword for GNU Compiler       */

#elif defined   (  __TASKING__  )
  #define __ASM            __asm                                      /*!< asm keyword for TASKING Compiler      */
  #define __INLINE         inline                                     /*!< inline keyword for TASKING Compiler   */

#endif


/* ##########################  Core Instruction Access  ######################### */

#if defined ( __CC_ARM   ) /*------------------ RealView Compiler ----------------*/

/**
 * @brief  Reverse byte order (16 bit)
 *
 * @param  value  value to reverse
 * @return        reversed value
 *
 * Reverse byte order in unsigned short value
 */
__ASM uint32_t __REV16(uint16_t value)
{
  rev16 r0, r0
  bx lr
}

/**
 * @brief  Reverse byte order in signed short value with sign extension to integer
 *
 * @param  value  value to reverse
 * @return        reversed value
 *
 * Reverse byte order in signed short value with sign extension to integer
 */
__ASM int32_t __REVSH(int16_t value)
{
  revsh r0, r0
  bx lr
}

 /**
 * @brief  Remove the exclusive lock created by ldrex
 *
 * Removes the exclusive lock which is created by ldrex.
 */
#if (__ARMCC_VERSION < 400000)
__ASM void __CLREX(void)
{
  clrex
}
#endif /* __ARMCC_VERSION  */ 


#elif (defined (__ICCARM__)) /*---------------- ICC Compiler ---------------------*/
/* obsolete */
#elif (defined (__GNUC__)) /*------------------ GNU Compiler ---------------------*/
/* obsolete */
#elif (defined (__TASKING__)) /*--------------- TASKING Compiler -----------------*/
/* obsolete */
#endif


/* ###########################  Core Function Access  ########################### */

#if defined ( __CC_ARM   ) /*------------------ RealView Compiler ----------------*/

/**
 * @brief  Return the Control Register value
* 
*  @return Control value
 *
 * Return the content of the control register
 */
#if       (__ARMCC_VERSION <  400000)
__ASM uint32_t __get_CONTROL(void)
{
  mrs r0, control
  bx lr
}
#endif /*  __ARMCC_VERSION  */ 

/**
 * @brief  Set the Control Register value
 *
 * @param  control  Control value
 *
 * Set the control register
 */
#if       (__ARMCC_VERSION <  400000)
__ASM void __set_CONTROL(uint32_t control)
{
  msr control, r0
  bx lr
}
#endif /*  __ARMCC_VERSION  */ 

/**
 * @brief  Get IPSR Register value
 *
 * @return uint32_t IPSR value
 *
 * return the content of the IPSR register
 */
#if       (__ARMCC_VERSION <  400000)
__ASM uint32_t __get_IPSR(void)
{
  mrs r0, ipsr
  bx lr
}
#endif /*  __ARMCC_VERSION  */ 

/**
 * @brief  Get APSR Register value
 *
 * @return uint32_t APSR value
 *
 * return the content of the APSR register
 */
#if       (__ARMCC_VERSION <  400000)
__ASM uint32_t __get_APSR(void)
{
  mrs r0, apsr
  bx lr
}
#endif /*  __ARMCC_VERSION  */ 

/**
 * @brief  Get xPSR Register value
 *
 * @return uint32_t xPSR value
 *
 * return the content of the xPSR register
 */
#if       (__ARMCC_VERSION <  400000)
__ASM uint32_t __get_xPSR(void)
{
  mrs r0, xpsr
  bx lr
}
#endif /*  __ARMCC_VERSION  */ 

/**
 * @brief  Return the Process Stack Pointer
 *
 * @return ProcessStackPointer
 *
 * Return the actual process stack pointer
 */
#if       (__ARMCC_VERSION <  400000)
__ASM uint32_t __get_PSP(void)
{
  mrs r0, psp
  bx lr
}
#endif /*  __ARMCC_VERSION  */ 
 
/**
 * @brief  Set the Process Stack Pointer
 *
 * @param  topOfProcStack  Process Stack Pointer
 *
 * Assign the value ProcessStackPointer to the MSP 
 * (process stack pointer) Cortex processor register
 */
#if       (__ARMCC_VERSION <  400000)
__ASM void __set_PSP(uint32_t topOfProcStack)
{
  msr psp, r0
  bx lr
}
#endif /*  __ARMCC_VERSION  */ 

/**
 * @brief  Return the Main Stack Pointer
 *
 * @return Main Stack Pointer
 *
 * Return the current value of the MSP (main stack pointer)
 * Cortex processor register
 */
#if       (__ARMCC_VERSION <  400000)
__ASM uint32_t __get_MSP(void)
{
  mrs r0, msp
  bx lr
}
#endif /*  __ARMCC_VERSION  */ 
 
/**
 * @brief  Set the Main Stack Pointer
 *
 * @param  topOfMainStack  Main Stack Pointer
 *
 * Assign the value mainStackPointer to the MSP 
 * (main stack pointer) Cortex processor register
 */
#if       (__ARMCC_VERSION <  400000)
__ASM void __set_MSP(uint32_t mainStackPointer)
{
  msr msp, r0
  bx lr
}
#endif /*  __ARMCC_VERSION  */ 
 
/**
 * @brief  Return the Priority Mask value
 *
 * @return PriMask
 *
 * Return state of the priority mask bit from the priority mask register
 */
#if       (__ARMCC_VERSION <  400000)
__ASM uint32_t __get_PRIMASK(void)
{
  mrs r0, primask
  bx lr
}
#endif /*  __ARMCC_VERSION  */ 
 
/**
 * @brief  Set the Priority Mask value
 *
 * @param  priMask  PriMask
 *
 * Set the priority mask bit in the priority mask register
 */
#if       (__ARMCC_VERSION <  400000)
__ASM void __set_PRIMASK(uint32_t priMask)
{
  msr primask, r0
  bx lr
}
#endif /*  __ARMCC_VERSION  */ 
 

#elif (defined (__ICCARM__)) /*---------------- ICC Compiler ---------------------*/
/* obsolete */
#elif (defined (__GNUC__)) /*------------------ GNU Compiler ---------------------*/
/* obsolete */
#elif (defined (__TASKING__)) /*--------------- TASKING Compiler -----------------*/
/* obsolete */
#endif
