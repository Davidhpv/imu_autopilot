/**************************************************************************//**
 * @file     core_cm4_dsp.h
 * @brief    CMSIS Cortex-M4 DSP SIMD Header File
 * @version  V1.00
 * @date     29. January 2010
 *
 * @note
 * Copyright (C) 2010 ARM Limited. All rights reserved.
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

#ifndef __CORE_CM4_DSP_H__
#define __CORE_CM4_DSP_H__

/** @addtogroup CMSIS_CM4_DSP_intrinsics CM4 DSP Intrinsics
  This file defines all CMSIS CM4 DSP intrinsics
  @{
 */

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdint.h>                           /* Include standard types */

#if defined (__ICCARM__)
  #include <intrinsics.h>                     /* IAR Intrinsics   */
#endif


/*******************************************************************************
 *                Hardware Abstraction Layer
 ******************************************************************************/

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


/* ###################  Compiler specific Intrinsics  ########################### */

#if defined ( __CC_ARM   ) /*------------------RealView Compiler -----------------*/
/* ARM armcc specific functions */

/*------ CM4 DSP Intrinsics ------------------------------------------------------*/
#define __SADD8                           __sadd8
#define __QADD8                           __qadd8
#define __SHADD8                          __shadd8
#define __UADD8                           __uadd8
#define __UQADD8                          __uqadd8
#define __UHADD8                          __uhadd8
#define __SSUB8                           __ssub8
#define __QSUB8                           __qsub8
#define __SHSUB8                          __shsub8
#define __USUB8                           __usub8
#define __UQSUB8                          __uqsub8
#define __UHSUB8                          __uhsub8
#define __SADD16                          __sadd16
#define __QADD16                          __qadd16
#define __SHADD16                         __shadd16
#define __UADD16                          __uadd16
#define __UQADD16                         __uqadd16
#define __UHADD16                         __uhadd16
#define __SSUB16                          __ssub16
#define __QSUB16                          __qsub16
#define __SHSUB16                         __shsub16
#define __USUB16                          __usub16
#define __UQSUB16                         __uqsub16
#define __UHSUB16                         __uhsub16
#define __SASX                            __sasx
#define __QASX                            __qasx
#define __SHASX                           __shasx
#define __UASX                            __uasx
#define __UQASX                           __uqasx
#define __UHASX                           __uhasx
#define __SSAX                            __ssax
#define __QSAX                            __qsax
#define __SHSAX                           __shsax
#define __USAX                            __usax
#define __UQSAX                           __uqsax
#define __UHSAX                           __uhsax
#define __USAD8                           __usad8
#define __USADA8                          __usada8
#define __SSAT16                          __ssat16
#define __USAT16                          __usat16
#define __UXTB16                          __uxtb16
#define __UXTAB16                         __uxtab16
#define __SXTB16                          __sxtb16
#define __SXTAB16                         __sxtab16
#define __SMUAD                           __smuad
#define __SMUADX                          __smuadx
#define __SMLAD                           __smlad
#define __SMLADX                          __smladx
#define __SMLALD                          __smlald
#define __SMLALDX                         __smlaldx
#define __SMUSD                           __smusd
#define __SMUSDX                          __smusdx
#define __SMLSD                           __smlsd
#define __SMLSDX                          __smlsdx
#define __SMLSLD                          __smlsld
#define __SMLSLDX                         __smlsldx
#define __SEL                             __sel
/*-- End CM4 Intrinsics ----------------------------------------------------------*/



#elif (defined (__ICCARM__)) /*------------------ ICC Compiler -------------------*/
/* IAR iccarm specific functions */

/*------ CM4 DSP Intrinsics ------------------------------------------------------*/
/* not yet supported */
/*-- End CM4 Intrinsics ----------------------------------------------------------*/

#elif (defined (__GNUC__)) /*------------------ GNU Compiler ---------------------*/
/* GNU gcc specific functions */

/*------ CM4 DSP Intrinsics ------------------------------------------------------*/
static __INLINE uint32_t __SADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("sadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __QADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("qadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __SHADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("shadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UQADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uqadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UHADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uhadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}


static __INLINE uint32_t __SSUB8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("ssub8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __QSUB8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("qsub8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __SHSUB8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("shsub8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __USUB8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("usub8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UQSUB8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uqsub8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UHSUB8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uhsub8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}


static __INLINE uint32_t __SADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("sadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __QADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("qadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __SHADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("shadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UQADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uqadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UHADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uhadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}


static __INLINE uint32_t __SSUB16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("ssub16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __QSUB16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("qsub16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __SHSUB16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("shsub16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __USUB16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("usub16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UQSUB16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uqsub16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UHSUB16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uhsub16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}


static __INLINE uint32_t __SASX(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("sasx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __QASX(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("qasx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __SHASX(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("shasx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UASX(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uasx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UQASX(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uqasx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UHASX(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uhasx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}


static __INLINE uint32_t __SSAX(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("ssax %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __QSAX(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("qsax %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __SHSAX(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("shsax %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __USAX(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("usax %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UQSAX(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uqsax %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __UHSAX(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uhsax %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}


static __INLINE uint32_t __USAD8(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("usad8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}


static __INLINE uint32_t __USADA8(uint32_t op1, uint32_t op2, uint32_t op3)
{
  uint32_t result=0;
  
   __ASM volatile ("usada8 %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
   return(result);
}


#if 0
static __INLINE uint32_t __SSAT16(uint32_t op1, const uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("ssat16 %0, %1, %2" : "=r" (result) :  "I" (8), "r" (op1) );
   return(result);
}

static __INLINE uint32_t __USAT16(uint32_t op1, const uint32_t op2)
{
  uint32_t result=0;
  
//   __ASM volatile ("usat16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}
#endif
#define __SSAT16(ARG1,ARG2) \
({                          \
  uint32_t __RES, __ARG1 = (ARG1); \
  __ASM ("ssat16 %0, %1, %2" : "=r" (__RES) :  "I" (ARG2), "r" (__ARG1) ); \
  __RES; \
 })
  
#define __USAT16(ARG1,ARG2) \
({                          \
  uint32_t __RES, __ARG1 = (ARG1); \
  __ASM ("usat16 %0, %1, %2" : "=r" (__RES) :  "I" (ARG2), "r" (__ARG1) ); \
  __RES; \
 })

static __INLINE uint32_t __UXTB16(uint32_t op1)
{
  uint32_t result=0;
  
   __ASM volatile ("uxtb16 %0, %1" : "=r" (result) : "r" (op1));
   return(result);
}


static __INLINE uint32_t __UXTAB16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("uxtab16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}


static __INLINE uint32_t __SXTB16(uint32_t op1)
{
  uint32_t result=0;
  
   __ASM volatile ("sxtb16 %0, %1" : "=r" (result) : "r" (op1));
   return(result);
}


static __INLINE uint32_t __SXTAB16(uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("sxtab16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}


static __INLINE uint32_t __SMUAD  (uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("smuad %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __SMUADX (uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("smuadx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __SMLAD (uint32_t op1, uint32_t op2, uint32_t op3)
{
  uint32_t result=0;
  
   __ASM volatile ("smlad %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
   return(result);
}

static __INLINE uint32_t __SMLADX (uint32_t op1, uint32_t op2, uint32_t op3)
{
  uint32_t result=0;
  
   __ASM volatile ("smladx %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
   return(result);
}

#if 0
uint64_t __SMLALD (uint32_t op1, uint32_t op2, uint64_t op3) __attribute__( ( naked ) );
uint64_t __SMLALD (uint32_t op1, uint32_t op2, uint64_t op3)
{
//  uint64_t result=0;
  
//   __ASM volatile ("smlald %0, %1, %2, %3" : "=r" (op3) : "r" (op1), "r" (op2), "0" (op3) );
  __ASM volatile ("smlald r2, r3, r0, r1\n\t"
                  "MOV r0, r2 \n\t"
                  "MOV r1, r3 \n\t"
                  "BX  lr     \n\t" : "=r" (op3) : "r" (op1), "r" (op2), "0" (op3) );
//   return(result);
}

uint64_t __SMLALDX(uint32_t op1, uint32_t op2, uint64_t op3) __attribute__( ( naked ) );
uint64_t __SMLALDX(uint32_t op1, uint32_t op2, uint64_t op3)
{
//  uint64_t result=0;
  
//   __ASM volatile ("smlaldx %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
  __ASM volatile ("smlaldx r2, r3, r0, r1\n\t"
                  "MOV r0, r2 \n\t"
                  "MOV r1, r3 \n\t"
                  "BX  lr     \n\t" : "=r" (op3) : "r" (op1), "r" (op2), "0" (op3) );
//   return(result);
}
#endif

#define __SMLALD(ARG1,ARG2,ARG3) \
({ \
  uint32_t __ARG1 = (ARG1), __ARG2 = (ARG2), __ARG3_H = (uint32_t)((ARG3) >> 32), __ARG3_L = (uint32_t)((ARG3) & 0xFFFFFFFF); \
  __ASM volatile ("smlald %0, %1, %2, %3" : "=r" (__ARG3_L), "=r" (__ARG3_H) : "r" (__ARG1), "r" (__ARG2), "0" (__ARG3_L), "1" (__ARG3_H) ); \
  (uint64_t)(((uint64_t)__ARG3_H << 32) | __ARG3_L); \
 })

#define __SMLALDX(ARG1,ARG2,ARG3) \
({ \
  uint32_t __ARG1 = (ARG1), __ARG2 = (ARG2), __ARG3_H = (uint32_t)((ARG3) >> 32), __ARG3_L = (uint32_t)((ARG3) & 0xFFFFFFFF); \
  __ASM volatile ("smlaldx %0, %1, %2, %3" : "=r" (__ARG3_L), "=r" (__ARG3_H) : "r" (__ARG1), "r" (__ARG2), "0" (__ARG3_L), "1" (__ARG3_H) ); \
  (uint64_t)(((uint64_t)__ARG3_H << 32) | __ARG3_L); \
 })


static __INLINE uint32_t __SMUSD  (uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
  __ASM volatile ("smusd %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

static __INLINE uint32_t __SMUSDX (uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("smusdx %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}

static __INLINE uint32_t __SMLSD (uint32_t op1, uint32_t op2, uint32_t op3)
{
  uint32_t result=0;
  
   __ASM volatile ("smlsd %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
   return(result);
}

static __INLINE uint32_t __SMLSDX (uint32_t op1, uint32_t op2, uint32_t op3)
{
  uint32_t result=0;
  
   __ASM volatile ("smlsdx %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
   return(result);
}

#if 0
uint64_t __SMLSLD (uint32_t op1, uint32_t op2, uint64_t op3) __attribute__( ( naked ) );
uint64_t __SMLSLD (uint32_t op1, uint32_t op2, uint64_t op3)
{
//  uint64_t result=0;
  
//   __ASM volatile ("smlsld %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
  __ASM volatile ("smlsld r2, r3, r0, r1\n\t"
                  "MOV r0, r2 \n\t"
                  "MOV r1, r3 \n\t"
                  "BX  lr     \n\t" : "=r" (op3) : "r" (op1), "r" (op2), "0" (op3) );
//   return(result);
}

uint64_t __SMLSLDX(uint32_t op1, uint32_t op2, uint64_t op3) __attribute__( ( naked ) );
uint64_t __SMLSLDX(uint32_t op1, uint32_t op2, uint64_t op3)
{
//  uint64_t result=0;
  
//   __ASM volatile ("smlsldx %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
  __ASM volatile ("smlsldx r2, r3, r0, r1\n\t"
                  "MOV r0, r2 \n\t"
                  "MOV r1, r3 \n\t"
                  "BX  lr     \n\t" : "=r" (op3) : "r" (op1), "r" (op2), "0" (op3) );
//   return(result);
}
#endif

#define __SMLSLD(ARG1,ARG2,ARG3) \
({ \
  uint32_t __ARG1 = (ARG1), __ARG2 = (ARG2), __ARG3_H = (uint32_t)((ARG3) >> 32), __ARG3_L = (uint32_t)((ARG3) & 0xFFFFFFFF); \
  __ASM volatile ("smlsld %0, %1, %2, %3" : "=r" (__ARG3_L), "=r" (__ARG3_H) : "r" (__ARG1), "r" (__ARG2), "0" (__ARG3_L), "1" (__ARG3_H) ); \
  (uint64_t)(((uint64_t)__ARG3_H << 32) | __ARG3_L); \
 })

#define __SMLSLDX(ARG1,ARG2,ARG3) \
({ \
  uint32_t __ARG1 = (ARG1), __ARG2 = (ARG2), __ARG3_H = (uint32_t)((ARG3) >> 32), __ARG3_L = (uint32_t)((ARG3) & 0xFFFFFFFF); \
  __ASM volatile ("smlsldx %0, %1, %2, %3" : "=r" (__ARG3_L), "=r" (__ARG3_H) : "r" (__ARG1), "r" (__ARG2), "0" (__ARG3_L), "1" (__ARG3_H) ); \
  (uint64_t)(((uint64_t)__ARG3_H << 32) | __ARG3_L); \
 })



static __INLINE uint32_t __SEL  (uint32_t op1, uint32_t op2)
{
  uint32_t result=0;
  
   __ASM volatile ("sel %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
   return(result);
}
/*------ CM4 DSP Intrinsics ------------------------------------------------------*/
/*-- End CM4 Intrinsics ----------------------------------------------------------*/


#elif (defined (__TASKING__)) /*------------------ TASKING Compiler --------------*/
/* TASKING carm specific functions */


/*------ CM4 DSP Intrinsics ------------------------------------------------------*/
/* not yet supported */
/*-- End CM4 Intrinsics ----------------------------------------------------------*/


#endif


#ifdef __cplusplus
}
#endif

/*@}*/ /* end of group CMSIS_CM4_DSP_intrinsics */

#endif /* __CORE_CM4_DSP_H__ */
