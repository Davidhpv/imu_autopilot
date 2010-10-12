/**************************************************************************//**
 * @file     core_cm0.h
 * @brief    CMSIS Cortex-M0 Core Peripheral Access Layer Header File
 * @version  V1.40
 * @date     16. February 2010
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

#ifndef __CORE_CM0_H__
#define __CORE_CM0_H__

/** @addtogroup CMSIS_CM0_core_LintCinfiguration CMSIS CM0 Core Lint Configuration
 *
 * List of Lint messages which will be suppressed and not shown:
 *   - not yet checked
 * .
 * Note:  To re-enable a Message, insert a space before 'lint' *
 *
 */


/** @addtogroup CMSIS_CM0_core_definitions CM0 Core Definitions
  This file defines all structures and symbols for CMSIS core:
    - CMSIS version number
    - Cortex-M core registers and bitfields
    - Cortex-M core peripheral base address
  @{
 */

#ifdef __cplusplus
 extern "C" {
#endif 

#define __CM0_CMSIS_VERSION_MAIN  (0x01)                                                       /*!< [31:16] CMSIS HAL main version */
#define __CM0_CMSIS_VERSION_SUB   (0x40)                                                       /*!< [15:0]  CMSIS HAL sub version  */
#define __CM0_CMSIS_VERSION       ((__CM0_CMSIS_VERSION_MAIN << 16) | __CM0_CMSIS_VERSION_SUB) /*!< CMSIS HAL version number       */

#define __CORTEX_M                (0x00)                                                       /*!< Cortex core                    */

#ifndef __NVIC_PRIO_BITS
  #define __NVIC_PRIO_BITS    2               /*!< standard definition for NVIC Priority Bits */
#endif


#include <stdint.h>                           /* Include standard types */



/**
 * IO definitions
 *
 * define access restrictions to peripheral registers
 */

#ifdef __cplusplus
  #define     __I     volatile                /*!< defines 'read only' permissions      */
#else
  #define     __I     volatile const          /*!< defines 'read only' permissions      */
#endif
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */



/*******************************************************************************
 *                 Register Abstraction
 ******************************************************************************/
/** @addtogroup CMSIS_CM0_core_register CMSIS CM0 Core Register
 @{
*/

/** @addtogroup CMSIS_ARM_CORE CMSIS ARM Core
  memory mapped structure for Nested Vectored Interrupt Controller (NVIC)
  @{
 */
typedef union
{
  struct
  {
#if (__CORTEX_M != 0x04)
    uint32_t _reserved0:27;
#else
    uint32_t _reserved0:16;
    uint32_t GE:4;                             /*!< bit: 16..19  Greater than or Equal flags  */
    uint32_t _reserved1:7;
#endif
    uint32_t Q:1;                              /*!< bit:     27  Saturation condition flag    */
    uint32_t V:1;                              /*!< bit:     28  Overflow condition code flag */
    uint32_t C:1;                              /*!< bit:     29  Carry condition code flag    */
    uint32_t Z:1;                              /*!< bit:     30  Zero condition code flag     */
    uint32_t N:1;                              /*!< bit:     31  Negative condition code flag */
  } b;
  uint32_t w;
} APSR_Type;

typedef union
{
  struct
  {
    uint32_t ISR:9;                            /*!< bit: 00..08  Exception number             */
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;

typedef union
{
  struct
  {
    uint32_t ISR:9;                            /*!< bit: 00..08  Exception number             */
#if (__CORTEX_M != 0x04)
    uint32_t _reserved0:15;
#else
    uint32_t _reserved0:7;
    uint32_t GE:4;                             /*!< bit: 16..19  Greater than or Equal flags  */
    uint32_t _reserved1:4;
#endif
    uint32_t T:1;                              /*!< bit:     24  Thumb bit        (read 0)    */
    uint32_t IT:2;                             /*!< bit: 25..26  saved IT state   (read 0)    */
    uint32_t Q:1;                              /*!< bit:     27  Saturation condition flag    */
    uint32_t V:1;                              /*!< bit:     28  Overflow condition code flag */
    uint32_t C:1;                              /*!< bit:     29  Carry condition code flag    */
    uint32_t Z:1;                              /*!< bit:     30  Zero condition code flag     */
    uint32_t N:1;                              /*!< bit:     31  Negative condition code flag */
  } b;
  uint32_t w;
} xPSR_Type;

typedef union
{
  struct
  {
    uint32_t nPRIV:1;                         /*!< bit:     00  Execution privilege in Thread mode */
    uint32_t SPSEL:1;                         /*!< bit:     01  Stack to be used                   */
    uint32_t FPCA:1;                          /*!< bit:     02  FP extension active flag           */
    uint32_t _reserved0:29;
  } b;
  uint32_t w;
} CONTROL_Type;

/*@}*/ /* end of group CMSIS_ARM_CORE */


/** @addtogroup CMSIS_CM0_NVIC CMSIS CM0 NVIC
  memory mapped structure for Nested Vectored Interrupt Controller (NVIC)
  @{
 */
typedef struct
{
  __IO uint32_t ISER[1];                      /*!< (Offset: 0x000) Interrupt Set Enable Register            */
       uint32_t RESERVED0[31];
  __IO uint32_t ICER[1];                      /*!< (Offset: 0x080) Interrupt Clear Enable Register          */
       uint32_t RSERVED1[31];
  __IO uint32_t ISPR[1];                      /*!< (Offset: 0x100) Interrupt Set Pending Register           */
       uint32_t RESERVED2[31];
  __IO uint32_t ICPR[1];                      /*!< (Offset: 0x180) Interrupt Clear Pending Register         */
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  __IO uint32_t IPR[8];                       /*!< (Offset: 0x3EC) Interrupt Priority Register              */
}  NVIC_Type;
/*@}*/ /* end of group CMSIS_CM0_NVIC */


/** @addtogroup CMSIS_CM0_SCB CMSIS CM0 SCB
  memory mapped structure for System Control Block (SCB)
  @{
 */
typedef struct
{
  __I  uint32_t CPUID;                        /*!< Offset: 0x00  CPU ID Base Register                                  */
  __IO uint32_t ICSR;                         /*!< Offset: 0x04  Interrupt Control State Register                      */
       uint32_t RESERVED0;                                      
  __IO uint32_t AIRCR;                        /*!< Offset: 0x0C  Application Interrupt / Reset Control Register        */
  __IO uint32_t SCR;                          /*!< Offset: 0x10  System Control Register                               */
  __IO uint32_t CCR;                          /*!< Offset: 0x14  Configuration Control Register                        */
       uint32_t RESERVED1;                                      
  __IO uint32_t SHP[2];                       /*!< Offset: 0x1C  System Handlers Priority Registers. [0] is RESERVED   */
  __IO uint32_t SHCSR;                        /*!< Offset: 0x24  System Handler Control and State Register             */
       uint32_t RESERVED2[2];                                   
  __IO uint32_t DFSR;                         /*!< Offset: 0x30  Debug Fault Status Register                           */
} SCB_Type;                                                

/* SCB CPUID Register Definitions */
#define SCB_CPUID_IMPLEMENTER_Pos          24                                             /*!< SCB CPUID: IMPLEMENTER Position */
#define SCB_CPUID_IMPLEMENTER_Msk          (0xFFul << SCB_CPUID_IMPLEMENTER_Pos)          /*!< SCB CPUID: IMPLEMENTER Mask */

#define SCB_CPUID_VARIANT_Pos              20                                             /*!< SCB CPUID: VARIANT Position */
#define SCB_CPUID_VARIANT_Msk              (0xFul << SCB_CPUID_VARIANT_Pos)               /*!< SCB CPUID: VARIANT Mask */

#define SCB_CPUID_ARCHITECTURE_Pos         16                                             /*!< SCB CPUID: ARCHITECTURE Position */
#define SCB_CPUID_ARCHITECTURE_Msk         (0xFul << SCB_CPUID_ARCHITECTURE_Pos)          /*!< SCB CPUID: ARCHITECTURE Mask */

#define SCB_CPUID_PARTNO_Pos                4                                             /*!< SCB CPUID: PARTNO Position */
#define SCB_CPUID_PARTNO_Msk               (0xFFFul << SCB_CPUID_PARTNO_Pos)              /*!< SCB CPUID: PARTNO Mask */

#define SCB_CPUID_REVISION_Pos              0                                             /*!< SCB CPUID: REVISION Position */
#define SCB_CPUID_REVISION_Msk             (0xFul << SCB_CPUID_REVISION_Pos)              /*!< SCB CPUID: REVISION Mask */

/* SCB Interrupt Control State Register Definitions */
#define SCB_ICSR_NMIPENDSET_Pos            31                                             /*!< SCB ICSR: NMIPENDSET Position */
#define SCB_ICSR_NMIPENDSET_Msk            (1ul << SCB_ICSR_NMIPENDSET_Pos)               /*!< SCB ICSR: NMIPENDSET Mask */

#define SCB_ICSR_PENDSVSET_Pos             28                                             /*!< SCB ICSR: PENDSVSET Position */
#define SCB_ICSR_PENDSVSET_Msk             (1ul << SCB_ICSR_PENDSVSET_Pos)                /*!< SCB ICSR: PENDSVSET Mask */

#define SCB_ICSR_PENDSVCLR_Pos             27                                             /*!< SCB ICSR: PENDSVCLR Position */
#define SCB_ICSR_PENDSVCLR_Msk             (1ul << SCB_ICSR_PENDSVCLR_Pos)                /*!< SCB ICSR: PENDSVCLR Mask */

#define SCB_ICSR_PENDSTSET_Pos             26                                             /*!< SCB ICSR: PENDSTSET Position */
#define SCB_ICSR_PENDSTSET_Msk             (1ul << SCB_ICSR_PENDSTSET_Pos)                /*!< SCB ICSR: PENDSTSET Mask */

#define SCB_ICSR_PENDSTCLR_Pos             25                                             /*!< SCB ICSR: PENDSTCLR Position */
#define SCB_ICSR_PENDSTCLR_Msk             (1ul << SCB_ICSR_PENDSTCLR_Pos)                /*!< SCB ICSR: PENDSTCLR Mask */

#define SCB_ICSR_ISRPREEMPT_Pos            23                                             /*!< SCB ICSR: ISRPREEMPT Position */
#define SCB_ICSR_ISRPREEMPT_Msk            (1ul << SCB_ICSR_ISRPREEMPT_Pos)               /*!< SCB ICSR: ISRPREEMPT Mask */

#define SCB_ICSR_ISRPENDING_Pos            22                                             /*!< SCB ICSR: ISRPENDING Position */
#define SCB_ICSR_ISRPENDING_Msk            (1ul << SCB_ICSR_ISRPENDING_Pos)               /*!< SCB ICSR: ISRPENDING Mask */

#define SCB_ICSR_VECTPENDING_Pos           12                                             /*!< SCB ICSR: VECTPENDING Position */
#define SCB_ICSR_VECTPENDING_Msk           (0x1FFul << SCB_ICSR_VECTPENDING_Pos)          /*!< SCB ICSR: VECTPENDING Mask */

#define SCB_ICSR_VECTACTIVE_Pos             0                                             /*!< SCB ICSR: VECTACTIVE Position */
#define SCB_ICSR_VECTACTIVE_Msk            (0x1FFul << SCB_ICSR_VECTACTIVE_Pos)           /*!< SCB ICSR: VECTACTIVE Mask */

/* SCB Application Interrupt and Reset Control Register Definitions */
#define SCB_AIRCR_VECTKEY_Pos              16                                             /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFul << SCB_AIRCR_VECTKEY_Pos)            /*!< SCB AIRCR: VECTKEY Mask */

#define SCB_AIRCR_VECTKEYSTAT_Pos          16                                             /*!< SCB AIRCR: VECTKEYSTAT Position */
#define SCB_AIRCR_VECTKEYSTAT_Msk          (0xFFFFul << SCB_AIRCR_VECTKEYSTAT_Pos)        /*!< SCB AIRCR: VECTKEYSTAT Mask */

#define SCB_AIRCR_ENDIANESS_Pos            15                                             /*!< SCB AIRCR: ENDIANESS Position */
#define SCB_AIRCR_ENDIANESS_Msk            (1ul << SCB_AIRCR_ENDIANESS_Pos)               /*!< SCB AIRCR: ENDIANESS Mask */

#define SCB_AIRCR_SYSRESETREQ_Pos           2                                             /*!< SCB AIRCR: SYSRESETREQ Position */
#define SCB_AIRCR_SYSRESETREQ_Msk          (1ul << SCB_AIRCR_SYSRESETREQ_Pos)             /*!< SCB AIRCR: SYSRESETREQ Mask */

#define SCB_AIRCR_VECTCLRACTIVE_Pos         1                                             /*!< SCB AIRCR: VECTCLRACTIVE Position */
#define SCB_AIRCR_VECTCLRACTIVE_Msk        (1ul << SCB_AIRCR_VECTCLRACTIVE_Pos)           /*!< SCB AIRCR: VECTCLRACTIVE Mask */

/* SCB System Control Register Definitions */
#define SCB_SCR_SEVONPEND_Pos               4                                             /*!< SCB SCR: SEVONPEND Position */
#define SCB_SCR_SEVONPEND_Msk              (1ul << SCB_SCR_SEVONPEND_Pos)                 /*!< SCB SCR: SEVONPEND Mask */

#define SCB_SCR_SLEEPDEEP_Pos               2                                             /*!< SCB SCR: SLEEPDEEP Position */
#define SCB_SCR_SLEEPDEEP_Msk              (1ul << SCB_SCR_SLEEPDEEP_Pos)                 /*!< SCB SCR: SLEEPDEEP Mask */

#define SCB_SCR_SLEEPONEXIT_Pos             1                                             /*!< SCB SCR: SLEEPONEXIT Position */
#define SCB_SCR_SLEEPONEXIT_Msk            (1ul << SCB_SCR_SLEEPONEXIT_Pos)               /*!< SCB SCR: SLEEPONEXIT Mask */

/* SCB Configuration Control Register Definitions */
#define SCB_CCR_STKALIGN_Pos                9                                             /*!< SCB CCR: STKALIGN Position */
#define SCB_CCR_STKALIGN_Msk               (1ul << SCB_CCR_STKALIGN_Pos)                  /*!< SCB CCR: STKALIGN Mask */

#define SCB_CCR_UNALIGN_TRP_Pos             3                                             /*!< SCB CCR: UNALIGN_TRP Position */
#define SCB_CCR_UNALIGN_TRP_Msk            (1ul << SCB_CCR_UNALIGN_TRP_Pos)               /*!< SCB CCR: UNALIGN_TRP Mask */

/* SCB System Handler Control and State Register Definitions */
#define SCB_SHCSR_SVCALLPENDED_Pos         15                                             /*!< SCB SHCSR: SVCALLPENDED Position */
#define SCB_SHCSR_SVCALLPENDED_Msk         (1ul << SCB_SHCSR_SVCALLPENDED_Pos)            /*!< SCB SHCSR: SVCALLPENDED Mask */

/* SCB Debug Fault Status Register Definitions */
#define SCB_DFSR_EXTERNAL_Pos               4                                             /*!< SCB DFSR: EXTERNAL Position */
#define SCB_DFSR_EXTERNAL_Msk              (1ul << SCB_DFSR_EXTERNAL_Pos)                 /*!< SCB DFSR: EXTERNAL Mask */

#define SCB_DFSR_VCATCH_Pos                 3                                             /*!< SCB DFSR: VCATCH Position */
#define SCB_DFSR_VCATCH_Msk                (1ul << SCB_DFSR_VCATCH_Pos)                   /*!< SCB DFSR: VCATCH Mask */

#define SCB_DFSR_DWTTRAP_Pos                2                                             /*!< SCB DFSR: DWTTRAP Position */
#define SCB_DFSR_DWTTRAP_Msk               (1ul << SCB_DFSR_DWTTRAP_Pos)                  /*!< SCB DFSR: DWTTRAP Mask */

#define SCB_DFSR_BKPT_Pos                   1                                             /*!< SCB DFSR: BKPT Position */
#define SCB_DFSR_BKPT_Msk                  (1ul << SCB_DFSR_BKPT_Pos)                     /*!< SCB DFSR: BKPT Mask */

#define SCB_DFSR_HALTED_Pos                 0                                             /*!< SCB DFSR: HALTED Position */
#define SCB_DFSR_HALTED_Msk                (1ul << SCB_DFSR_HALTED_Pos)                   /*!< SCB DFSR: HALTED Mask */
/*@}*/ /* end of group CMSIS_CM0_SCB */


/** @addtogroup CMSIS_CM0_SysTick CMSIS CM0 SysTick
  memory mapped structure for SysTick
  @{
 */
typedef struct
{
  __IO uint32_t CTRL;                         /*!< Offset: 0x00  SysTick Control and Status Register */
  __IO uint32_t LOAD;                         /*!< Offset: 0x04  SysTick Reload Value Register       */
  __IO uint32_t VAL;                          /*!< Offset: 0x08  SysTick Current Value Register      */
  __I  uint32_t CALIB;                        /*!< Offset: 0x0C  SysTick Calibration Register        */
} SysTick_Type;

/* SysTick Control / Status Register Definitions */
#define SysTick_CTRL_COUNTFLAG_Pos         16                                             /*!< SysTick CTRL: COUNTFLAG Position */
#define SysTick_CTRL_COUNTFLAG_Msk         (1ul << SysTick_CTRL_COUNTFLAG_Pos)            /*!< SysTick CTRL: COUNTFLAG Mask */

#define SysTick_CTRL_CLKSOURCE_Pos          2                                             /*!< SysTick CTRL: CLKSOURCE Position */
#define SysTick_CTRL_CLKSOURCE_Msk         (1ul << SysTick_CTRL_CLKSOURCE_Pos)            /*!< SysTick CTRL: CLKSOURCE Mask */

#define SysTick_CTRL_TICKINT_Pos            1                                             /*!< SysTick CTRL: TICKINT Position */
#define SysTick_CTRL_TICKINT_Msk           (1ul << SysTick_CTRL_TICKINT_Pos)              /*!< SysTick CTRL: TICKINT Mask */

#define SysTick_CTRL_ENABLE_Pos             0                                             /*!< SysTick CTRL: ENABLE Position */
#define SysTick_CTRL_ENABLE_Msk            (1ul << SysTick_CTRL_ENABLE_Pos)               /*!< SysTick CTRL: ENABLE Mask */

/* SysTick Reload Register Definitions */
#define SysTick_LOAD_RELOAD_Pos             0                                             /*!< SysTick LOAD: RELOAD Position */
#define SysTick_LOAD_RELOAD_Msk            (0xFFFFFFul << SysTick_LOAD_RELOAD_Pos)        /*!< SysTick LOAD: RELOAD Mask */

/* SysTick Current Register Definitions */
#define SysTick_VAL_CURRENT_Pos             0                                             /*!< SysTick VAL: CURRENT Position */
#define SysTick_VAL_CURRENT_Msk            (0xFFFFFFul << SysTick_VAL_CURRENT_Pos)        /*!< SysTick VAL: CURRENT Mask */

/* SysTick Calibration Register Definitions */
#define SysTick_CALIB_NOREF_Pos            31                                             /*!< SysTick CALIB: NOREF Position */
#define SysTick_CALIB_NOREF_Msk            (1ul << SysTick_CALIB_NOREF_Pos)               /*!< SysTick CALIB: NOREF Mask */

#define SysTick_CALIB_SKEW_Pos             30                                             /*!< SysTick CALIB: SKEW Position */
#define SysTick_CALIB_SKEW_Msk             (1ul << SysTick_CALIB_SKEW_Pos)                /*!< SysTick CALIB: SKEW Mask */

#define SysTick_CALIB_TENMS_Pos             0                                             /*!< SysTick CALIB: TENMS Position */
#define SysTick_CALIB_TENMS_Msk            (0xFFFFFFul << SysTick_VAL_CURRENT_Pos)        /*!< SysTick CALIB: TENMS Mask */
/*@}*/ /* end of group CMSIS_CM0_SysTick */


/** @addtogroup CMSIS_CM0_CoreDebug CMSIS CM0 Core Debug
  memory mapped structure for Core Debug Register
  @{
 */
typedef struct
{
  __IO uint32_t DHCSR;                        /*!< Offset: 0x00  Debug Halting Control and Status Register    */
  __O  uint32_t DCRSR;                        /*!< Offset: 0x04  Debug Core Register Selector Register        */
  __IO uint32_t DCRDR;                        /*!< Offset: 0x08  Debug Core Register Data Register            */
  __IO uint32_t DEMCR;                        /*!< Offset: 0x0C  Debug Exception and Monitor Control Register */
} CoreDebug_Type;

/* Debug Halting Control and Status Register */
#define CoreDebug_DHCSR_DBGKEY_Pos         16                                             /*!< CoreDebug DHCSR: DBGKEY Position */
#define CoreDebug_DHCSR_DBGKEY_Msk         (0xFFFFul << CoreDebug_DHCSR_DBGKEY_Pos)       /*!< CoreDebug DHCSR: DBGKEY Mask */

#define CoreDebug_DHCSR_S_RESET_ST_Pos     25                                             /*!< CoreDebug DHCSR: S_RESET_ST Position */
#define CoreDebug_DHCSR_S_RESET_ST_Msk     (1ul << CoreDebug_DHCSR_S_RESET_ST_Pos)        /*!< CoreDebug DHCSR: S_RESET_ST Mask */

#define CoreDebug_DHCSR_S_RETIRE_ST_Pos    24                                             /*!< CoreDebug DHCSR: S_RETIRE_ST Position */
#define CoreDebug_DHCSR_S_RETIRE_ST_Msk    (1ul << CoreDebug_DHCSR_S_RETIRE_ST_Pos)       /*!< CoreDebug DHCSR: S_RETIRE_ST Mask */

#define CoreDebug_DHCSR_S_LOCKUP_Pos       19                                             /*!< CoreDebug DHCSR: S_LOCKUP Position */
#define CoreDebug_DHCSR_S_LOCKUP_Msk       (1ul << CoreDebug_DHCSR_S_LOCKUP_Pos)          /*!< CoreDebug DHCSR: S_LOCKUP Mask */

#define CoreDebug_DHCSR_S_SLEEP_Pos        18                                             /*!< CoreDebug DHCSR: S_SLEEP Position */
#define CoreDebug_DHCSR_S_SLEEP_Msk        (1ul << CoreDebug_DHCSR_S_SLEEP_Pos)           /*!< CoreDebug DHCSR: S_SLEEP Mask */

#define CoreDebug_DHCSR_S_HALT_Pos         17                                             /*!< CoreDebug DHCSR: S_HALT Position */
#define CoreDebug_DHCSR_S_HALT_Msk         (1ul << CoreDebug_DHCSR_S_HALT_Pos)            /*!< CoreDebug DHCSR: S_HALT Mask */

#define CoreDebug_DHCSR_S_REGRDY_Pos       16                                             /*!< CoreDebug DHCSR: S_REGRDY Position */
#define CoreDebug_DHCSR_S_REGRDY_Msk       (1ul << CoreDebug_DHCSR_S_REGRDY_Pos)          /*!< CoreDebug DHCSR: S_REGRDY Mask */

#define CoreDebug_DHCSR_C_MASKINTS_Pos      3                                             /*!< CoreDebug DHCSR: C_MASKINTS Position */
#define CoreDebug_DHCSR_C_MASKINTS_Msk     (1ul << CoreDebug_DHCSR_C_MASKINTS_Pos)        /*!< CoreDebug DHCSR: C_MASKINTS Mask */

#define CoreDebug_DHCSR_C_STEP_Pos          2                                             /*!< CoreDebug DHCSR: C_STEP Position */
#define CoreDebug_DHCSR_C_STEP_Msk         (1ul << CoreDebug_DHCSR_C_STEP_Pos)            /*!< CoreDebug DHCSR: C_STEP Mask */

#define CoreDebug_DHCSR_C_HALT_Pos          1                                             /*!< CoreDebug DHCSR: C_HALT Position */
#define CoreDebug_DHCSR_C_HALT_Msk         (1ul << CoreDebug_DHCSR_C_HALT_Pos)            /*!< CoreDebug DHCSR: C_HALT Mask */

#define CoreDebug_DHCSR_C_DEBUGEN_Pos       0                                             /*!< CoreDebug DHCSR: C_DEBUGEN Position */
#define CoreDebug_DHCSR_C_DEBUGEN_Msk      (1ul << CoreDebug_DHCSR_C_DEBUGEN_Pos)         /*!< CoreDebug DHCSR: C_DEBUGEN Mask */

/* Debug Core Register Selector Register */
#define CoreDebug_DCRSR_REGWnR_Pos         16                                             /*!< CoreDebug DCRSR: REGWnR Position */
#define CoreDebug_DCRSR_REGWnR_Msk         (1ul << CoreDebug_DCRSR_REGWnR_Pos)            /*!< CoreDebug DCRSR: REGWnR Mask */

#define CoreDebug_DCRSR_REGSEL_Pos          0                                             /*!< CoreDebug DCRSR: REGSEL Position */
#define CoreDebug_DCRSR_REGSEL_Msk         (0x1Ful << CoreDebug_DCRSR_REGSEL_Pos)         /*!< CoreDebug DCRSR: REGSEL Mask */

/* Debug Exception and Monitor Control Register */
#define CoreDebug_DEMCR_DWTENA_Pos         24                                             /*!< CoreDebug DEMCR: DWTENA Position */
#define CoreDebug_DEMCR_DWTENA_Msk         (1ul << CoreDebug_DEMCR_DWTENA_Pos)            /*!< CoreDebug DEMCR: DWTENA Mask */

#define CoreDebug_DEMCR_VC_HARDERR_Pos     10                                             /*!< CoreDebug DEMCR: VC_HARDERR Position */
#define CoreDebug_DEMCR_VC_HARDERR_Msk     (1ul << CoreDebug_DEMCR_VC_HARDERR_Pos)        /*!< CoreDebug DEMCR: VC_HARDERR Mask */

#define CoreDebug_DEMCR_VC_CORERESET_Pos    0                                             /*!< CoreDebug DEMCR: VC_CORERESET Position */
#define CoreDebug_DEMCR_VC_CORERESET_Msk   (1ul << CoreDebug_DEMCR_VC_CORERESET_Pos)      /*!< CoreDebug DEMCR: VC_CORERESET Mask */
/*@}*/ /* end of group CMSIS_CM0_CoreDebug */


/* Memory mapping of Cortex-M0 Hardware */
#define SCS_BASE            (0xE000E000)                              /*!< System Control Space Base Address */
#define CoreDebug_BASE      (0xE000EDF0)                              /*!< Core Debug Base Address           */
#define SysTick_BASE        (SCS_BASE +  0x0010)                      /*!< SysTick Base Address              */
#define NVIC_BASE           (SCS_BASE +  0x0100)                      /*!< NVIC Base Address                 */
#define SCB_BASE            (SCS_BASE +  0x0D00)                      /*!< System Control Block Base Address */

#define SCB                 ((SCB_Type *)           SCB_BASE)         /*!< SCB configuration struct          */
#define SysTick             ((SysTick_Type *)       SysTick_BASE)     /*!< SysTick configuration struct      */
#define NVIC                ((NVIC_Type *)          NVIC_BASE)        /*!< NVIC configuration struct         */
#define CoreDebug           ((CoreDebug_Type *)     CoreDebug_BASE)   /*!< Core Debug configuration struct   */

/*@}*/ /* end of group CMSIS_CM0_core_register */


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

/* ##########################  Core Instruction Access  ######################### */
#include "core_cmInstr.h"

/* ###########################  Core Function Access  ########################### */
#include "core_cmFunc.h"


/** @addtogroup CMSIS_CM0_Core_FunctionInterface CMSIS CM0 Core Function Interface
  Core  Function Interface containing:
  - Core NVIC Functions
  - Core SysTick Functions
  - Core Reset Functions
*/
/*@{*/

/* ##########################   NVIC functions  #################################### */

/* Interrupt Priorities are WORD accessible only under ARMv6M                   */
/* The following MACROS handle generation of the register offset and byte masks */
#define _BIT_SHIFT(IRQn)         (  (((uint32_t)(IRQn)       )    &  0x03) * 8 )
#define _SHP_IDX(IRQn)           ( ((((uint32_t)(IRQn) & 0x0F)-8) >>    2)     )
#define _IP_IDX(IRQn)            (   ((uint32_t)(IRQn)            >>    2)     )


/**
 * @brief  Enable Interrupt in NVIC Interrupt Controller
 *
 * @param  IRQn   The positive number of the external interrupt to enable
 *
 * Enable a device specific interupt in the NVIC interrupt controller.
 * The interrupt number cannot be a negative value.
 */
static __INLINE void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  NVIC->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F)); /* enable interrupt */
}

/**
 * @brief  Disable the interrupt line for external interrupt specified
 * 
 * @param  IRQn   The positive number of the external interrupt to disable
 * 
 * Disable a device specific interupt in the NVIC interrupt controller.
 * The interrupt number cannot be a negative value.
 */
static __INLINE void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  NVIC->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F)); /* disable interrupt */
}

/**
 * @brief  Read the interrupt pending bit for a device specific interrupt source
 * 
 * @param  IRQn    The number of the device specifc interrupt
 * @return         1 = interrupt pending, 0 = interrupt not pending
 *
 * Read the pending register in NVIC and return 1 if its status is pending, 
 * otherwise it returns 0
 */
static __INLINE uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((NVIC->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0)); /* Return 1 if pending else 0 */
}

/**
 * @brief  Set the pending bit for an external interrupt
 * 
 * @param  IRQn    The number of the interrupt for set pending
 *
 * Set the pending bit for the specified interrupt.
 * The interrupt number cannot be a negative value.
 */
static __INLINE void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  NVIC->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F)); /* set interrupt pending */
}

/**
 * @brief  Clear the pending bit for an external interrupt
 *
 * @param  IRQn    The number of the interrupt for clear pending
 *
 * Clear the pending bit for the specified interrupt. 
 * The interrupt number cannot be a negative value.
 */
static __INLINE void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  NVIC->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F)); /* Clear pending interrupt */
}

/**
 * @brief  Set the priority for an interrupt
 *
 * @param  IRQn      The number of the interrupt for set priority
 * @param  priority  The priority to set
 *
 * Set the priority for the specified interrupt. The interrupt 
 * number can be positive to specify an external (device specific) 
 * interrupt, or negative to specify an internal (core) interrupt.
 *
 * Note: The priority cannot be set for every core interrupt.
 */
static __INLINE void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    SCB->SHP[_SHP_IDX(IRQn)] = (SCB->SHP[_SHP_IDX(IRQn)] & ~(0xFF << _BIT_SHIFT(IRQn))) | 
        (((priority << (8 - __NVIC_PRIO_BITS)) & 0xFF) << _BIT_SHIFT(IRQn)); }
  else {
    NVIC->IPR[_IP_IDX(IRQn)] = (NVIC->IPR[_IP_IDX(IRQn)] & ~(0xFF << _BIT_SHIFT(IRQn))) |
        (((priority << (8 - __NVIC_PRIO_BITS)) & 0xFF) << _BIT_SHIFT(IRQn)); }
}

/**
 * @brief  Read the priority for an interrupt
 *
 * @param  IRQn      The number of the interrupt for get priority
 * @return           The priority for the interrupt
 *
 * Read the priority for the specified interrupt. The interrupt 
 * number can be positive to specify an external (device specific) 
 * interrupt, or negative to specify an internal (core) interrupt.
 *
 * The returned priority value is automatically aligned to the implemented
 * priority bits of the microcontroller.
 *
 * Note: The priority cannot be set for every core interrupt.
 */
static __INLINE uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)((SCB->SHP[_SHP_IDX(IRQn)] >> _BIT_SHIFT(IRQn) ) >> (8 - __NVIC_PRIO_BITS)));  } /* get priority for Cortex-M0 system interrupts */
  else {
    return((uint32_t)((NVIC->IPR[_IP_IDX(IRQn)] >> _BIT_SHIFT(IRQn) ) >> (8 - __NVIC_PRIO_BITS)));  } /* get priority for device specific interrupts  */
}



/* ##################################    SysTick function  ############################################ */

#if (!defined (__Vendor_SysTickConfig)) || (__Vendor_SysTickConfig == 0)

/**
 * @brief  Initialize and start the SysTick counter and its interrupt.
 *
 * @param   ticks   number of ticks between two interrupts
 * @return  1 = failed, 0 = successful
 *
 * Initialise the system tick timer and its interrupt and start the
 * system tick timer / counter in free running mode to generate 
 * periodical interrupts.
 */
static __INLINE uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > SysTick_LOAD_RELOAD_Msk)  return (1);            /* Reload value impossible */
                                                               
  SysTick->LOAD  = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;      /* set reload register */
  NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Cortex-M0 System Interrupts */
  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | 
                   SysTick_CTRL_TICKINT_Msk   | 
                   SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
  return (0);                                                  /* Function successful */
}

#endif




/* ##################################    Reset function  ############################################ */

/**
 * @brief  Initiate a system reset request.
 *
 * Initiate a system reset request to reset the MCU
 */
static __INLINE void NVIC_SystemReset(void)
{
  __DSB();                                                     /* Ensure all outstanding memory accesses included
                                                                  buffered write are completed before reset */              
  SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)      | 
                 SCB_AIRCR_SYSRESETREQ_Msk);
  __DSB();                                                     /* Ensure completion of memory access */              
  while(1);                                                    /* wait until reset */
}

/*@}*/ /* end of group CMSIS_CM0_Core_FunctionInterface */

#ifdef __cplusplus
}
#endif

/*@}*/ /* end of group CMSIS_CM0_core_definitions */

#endif /* __CORE_CM0_H__ */

/*lint -restore */
