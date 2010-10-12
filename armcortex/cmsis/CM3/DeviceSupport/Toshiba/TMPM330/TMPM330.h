/******************************************************************************
 * @file:    TMPM330.h
 * @purpose: CMSIS Cortex-M3 Core Peripheral Access Layer Header File for the 
 *           TOSHIBA 'TMPM330' Device Series 
 * @version: V2.0.0
 * @date:    2009/10/26
 * 
 * THE SOURCE CODE AND ITS RELATED DOCUMENTATION IS PROVIDED "AS IS". TOSHIBA
 * CORPORATION MAKES NO OTHER WARRANTY OF ANY KIND, WHETHER EXPRESS, IMPLIED OR,
 * STATUTORY AND DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * SATISFACTORY QUALITY, NON INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 * 
 * THE SOURCE CODE AND DOCUMENTATION MAY INCLUDE ERRORS. TOSHIBA CORPORATION
 * RESERVES THE RIGHT TO INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER
 * REVISIONS OF IT, AND TO MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR
 * THE PRODUCTS OR TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME.
 * 
 * TOSHIBA CORPORATION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGE OR LIABILITY ARISING FROM YOUR USE OF THE SOURCE CODE OR
 * ANY DOCUMENTATION, INCLUDING BUT NOT LIMITED TO, LOST REVENUES, DATA OR
 * PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL OR CONSEQUENTIAL NATURE, PUNITIVE
 * DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS ARISING OUT OF OR IN CONNECTION
 * WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF ADVISED OF THE POSSIBILITY OR
 * PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM FOR SUCH DAMAGE IS BASED UPON
 * WARRANTY, CONTRACT, TORT, NEGLIGENCE OR OTHERWISE.
 * 
 * (C)Copyright TOSHIBA CORPORATION 2009 All rights reserved
 ******************************************************************************/

/** @addtogroup TOSHIBA_TX03_MICROCONTROLLER
  * @{
  */

/** @addtogroup TMPM330
  * @{
  */

#ifndef __TMPM330_H__
#define __TMPM330_H__

#ifdef __cplusplus
extern "C" {
#endif 

/** @addtogroup Configuration_of_CMSIS
  * @{
  */

/** Interrupt Number Definition */
typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ****************************************************/
  NonMaskableInt_IRQn           = -14,      /*!< 2 Non Maskable Interrupt                          */
  HardFault_IRQn                = -13,      /*!< 3 Cortex-M3 Hard Fault Interrupt                  */
  MemoryManagement_IRQn         = -12,      /*!< 4 Cortex-M3 Memory Management Interrupt           */
  BusFault_IRQn                 = -11,      /*!< 5 Cortex-M3 Bus Fault Interrupt                   */
  UsageFault_IRQn               = -10,      /*!< 6 Cortex-M3 Usage Fault Interrupt                 */
  SVCall_IRQn                   = -5,       /*!< 11 Cortex-M3 SV Call Interrupt                    */
  DebugMonitor_IRQn             = -4,       /*!< 12 Cortex-M3 Debug Monitor Interrupt              */
  PendSV_IRQn                   = -2,       /*!< 14 Cortex-M3 Pend SV Interrupt                    */
  SysTick_IRQn                  = -1,       /*!< 15 Cortex-M3 System Tick Interrupt                */

/******  TMPM330 Specific Interrupt Numbers ********************************************************/
  INT0_IRQn                     = 0,        /*!< Interrupt pin (PJ0/70 pin)                        */
  INT1_IRQn                     = 1,        /*!< Interrupt pin (PJ1/49 pin)                        */
  INT2_IRQn                     = 2,        /*!< Interrupt pin (PJ2/86 pin)                        */
  INT3_IRQn                     = 3,        /*!< Interrupt pin (PJ3/87 pin)                        */
  INT4_IRQn                     = 4,        /*!< Interrupt pin (PG3/6  pin)                        */
  INT5_IRQn                     = 5,        /*!< Interrupt pin (PF7/19 pin)                        */
  INTRX0_IRQn                   = 6,        /*!< Serial reception (channel.0)                      */
  INTTX0_IRQn                   = 7,        /*!< Serial transmission (channel.0)                   */
  INTRX1_IRQn                   = 8,        /*!< Serial reception (channel.1)                      */
  INTTX1_IRQn                   = 9,        /*!< Serial transmission (channel.1)                   */
  INTSBI0_IRQn                  = 10,       /*!< Serial bus interface 0                            */
  INTSBI1_IRQn                  = 11,       /*!< Serial bus interface 1                            */
  INTCECRX_IRQn                 = 12,       /*!< CEC reception                                     */
  INTCECTX_IRQn                 = 13,       /*!< CEC transmission                                  */
  INTRMCRX0_IRQn                = 14,       /*!< Remote control signal reception (channel.0)       */
  INTADHP_IRQn                  = 15,       /*!< Highest priority AD conversion complete interrupt */
  INTADM0_IRQn                  = 16,       /*!< AD conversion monitoring function interrupt 0     */
  INTADM1_IRQn                  = 17,       /*!< AD conversion monitoring function interrupt 1     */
  INTTB0_IRQn                   = 18,       /*!< 16bit TMRB match detection 0                      */
  INTTB1_IRQn                   = 19,       /*!< 16bit TMRB match detection 1                      */
  INTTB2_IRQn                   = 20,       /*!< 16bit TMRB match detection 2                      */
  INTTB3_IRQn                   = 21,       /*!< 16bit TMRB match detection 3                      */
  INTTB4_IRQn                   = 22,       /*!< 16bit TMRB match detection 4                      */
  INTTB5_IRQn                   = 23,       /*!< 16bit TMRB match detection 5                      */
  INTTB6_IRQn                   = 24,       /*!< 16bit TMRB match detection 6                      */
  INTRTC_IRQn                   = 25,       /*!< Real time clock timer                             */
  INTCAP00_IRQn                 = 26,       /*!< 16bit TMRB input capture 00                       */
  INTCAP01_IRQn                 = 27,       /*!< 16bit TMRB input capture 01                       */
  INTCAP10_IRQn                 = 28,       /*!< 16bit TMRB input capture 10                       */
  INTCAP11_IRQn                 = 29,       /*!< 16bit TMRB input capture 11                       */
  INTCAP50_IRQn                 = 30,       /*!< 16bit TMRB input capture 50                       */
  INTCAP51_IRQn                 = 31,       /*!< 16bit TMRB input capture 51                       */
  INTCAP60_IRQn                 = 32,       /*!< 16bit TMRB input capture 60                       */
  INTCAP61_IRQn                 = 33,       /*!< 16bit TMRB input capture 61                       */
  INT6_IRQn                     = 34,       /*!< Interrupt pin (PJ6/39 pin)                        */
  INT7_IRQn                     = 35,       /*!< Interrupt pin (PJ7/58 pin)                        */
  INTRX2_IRQn                   = 36,       /*!< Serial reception (channel.2)                      */
  INTTX2_IRQn                   = 37,       /*!< Serial transmission (channel.2)                   */
  INTSBI2_IRQn                  = 38,       /*!< Serial bus interface 2                            */
  INTRMCRX1_IRQn                = 39,       /*!< Remote control signal reception (channel.1)       */
  INTTB7_IRQn                   = 40,       /*!< 16bit TMRB match detection 7                      */
  INTTB8_IRQn                   = 41,       /*!< 16bit TMRB match detection 8                      */
  INTTB9_IRQn                   = 42,       /*!< 16bit TMRB match detection 9                      */
  INTCAP20_IRQn                 = 43,       /*!< 16bit TMRB input capture 20                       */
  INTCAP21_IRQn                 = 44,       /*!< 16bit TMRB input capture 21                       */
  INTCAP30_IRQn                 = 45,       /*!< 16bit TMRB input capture 30                       */
  INTCAP31_IRQn                 = 46,       /*!< 16bit TMRB input capture 31                       */
  INTCAP40_IRQn                 = 47,       /*!< 16bit TMRB input capture 40                       */
  INTCAP41_IRQn                 = 48,       /*!< 16bit TMRB input capture 41                       */
  INTAD_IRQn                    = 49        /*!< A/D conversion completion                         */
} IRQn_Type;


/** Processor and Core Peripheral Section */

/* Configuration of the Cortex-M3 Processor and Core Peripherals */
#define __MPU_PRESENT             0         /*!< MPU present or not                                */
#define __NVIC_PRIO_BITS          3         /*!< Number of Bits used for Priority Levels           */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used      */

/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm3.h"                       /* Cortex-M3 processor and core peripherals            */
#include "system_TMPM330.h"                 /* TMPM330 System                                      */

/** @addtogroup Device_Peripheral_registers
  * @{
  */

/** Device Specific Peripheral registers structures */

/**
  * @brief Clock Generator (CG)
  */
typedef struct
{
  __IO uint32_t SYSCR;          /*!< System Control Register              */
  __IO uint32_t OSCCR;          /*!< Oscillation Control Register         */
  __IO uint32_t STBYCR;         /*!< Standby Control Register             */
  __IO uint32_t PLLSEL;         /*!< PLL Selection Register               */
  __IO uint32_t CKSEL;          /*!< System Clock Selection Register      */
  __IO uint32_t ICRCG;          /*!< CG Interrupt Request Clear Register  */
  __I  uint32_t NMIFLG;         /*!< NMI Flag Register                    */
  __IO uint32_t RSTFLG;         /*!< Reset Flag Register                  */
  __IO uint32_t IMCGA;          /*!< CG Interrupt Mode Control Register A */
  __IO uint32_t IMCGB;          /*!< CG Interrupt Mode Control Register B */
  __IO uint32_t IMCGC;          /*!< CG Interrupt Mode Control Register C */
  __IO uint32_t IMCGD;          /*!< CG Interrupt Mode Control Register D */
} CG_TypeDef;

/**
  * @brief General Purpose Input/Output Port (PA)
  */
typedef struct
{
  __IO uint32_t DATA;           /*!< PA Register                      */
  __IO uint32_t CR;             /*!< PA Control Register              */
  __IO uint32_t FR1;            /*!< PA Function Register 1           */
       uint32_t RESERVED0[8];
  __IO uint32_t PUP;            /*!< PA Pull-Up Control Register      */
  __IO uint32_t PDN;            /*!< PA Pull-Down Control Register    */
       uint32_t RESERVED1;
  __IO uint32_t IE;             /*!< PA Input Enable Control Register */
} PA_TypeDef;

/**
  * @brief General Purpose Input/Output Port (PB)
  */
typedef struct
{
  __IO uint32_t DATA;           /*!< PB Register                      */
  __IO uint32_t CR;             /*!< PB Control Register              */
  __IO uint32_t FR1;            /*!< PB Function Register 1           */
       uint32_t RESERVED0[8];
  __IO uint32_t PUP;            /*!< PB Pull-Up Control Register      */
       uint32_t RESERVED1[2];
  __IO uint32_t IE;             /*!< PB Input Enable Control Register */
} PB_TypeDef;

/**
  * @brief General Purpose Input/Output Port (PC)
  */
typedef struct
{
  __I  uint32_t DATA;           /*!< PC Register                      */
       uint32_t RESERVED0[10];
  __IO uint32_t PUP;            /*!< PC Pull-Up Control Register      */
       uint32_t RESERVED1[2];
  __IO uint32_t IE;             /*!< PC Input Enable Control Register */
} PC_TypeDef;

/**
  * @brief General Purpose Input/Output Port (PD)
  */
typedef struct
{
  __I  uint32_t DATA;           /*!< PD Register                      */
  __IO uint32_t RESERVED0;
  __IO uint32_t FR1;            /*!< PD Function Register 1           */
       uint32_t RESERVED1[8];
  __IO uint32_t PUP;            /*!< PD Pull-Up Control Register      */
       uint32_t RESERVED2[2];
  __IO uint32_t IE;             /*!< PD Input Enable Control Register */
} PD_TypeDef;

/**
  * @brief General Purpose Input/Output Port (PE)
  */
typedef struct
{
  __IO uint32_t DATA;           /*!< PE Register                      */
  __IO uint32_t CR;             /*!< PE Control Register              */
  __IO uint32_t FR1;            /*!< PE Function Register 1           */
  __IO uint32_t FR2;            /*!< PE Function Register 2           */
       uint32_t RESERVED0[6];
  __IO uint32_t OD;             /*!< PE Open Drain Control Register   */
  __IO uint32_t PUP;            /*!< PE Pull-Up Control Register      */
       uint32_t RESERVED1[2];
  __IO uint32_t IE;             /*!< PE Input Enable Control Register */
} PE_TypeDef;

/**
  * @brief General Purpose Input/Output Port (PF)
  */
typedef struct
{
  __IO uint32_t DATA;           /*!< PF Register                      */
  __IO uint32_t CR;             /*!< PF Control Register              */
  __IO uint32_t FR1;            /*!< PF Function Register 1           */
  __IO uint32_t FR2;            /*!< PF Function Register 2           */
       uint32_t RESERVED0[6];
  __IO uint32_t OD;             /*!< PF Open Drain Control Register   */
  __IO uint32_t PUP;            /*!< PF Pull-Up Control Register      */
       uint32_t RESERVED1[2];
  __IO uint32_t IE;             /*!< PF Input Enable Control Register */
} PF_TypeDef;

/**
  * @brief General Purpose Input/Output Port (PG)
  */
typedef struct
{
  __IO uint32_t DATA;           /*!< PG Register                      */
  __IO uint32_t CR;             /*!< PG Control Register              */
  __IO uint32_t FR1;            /*!< PG Function Register 1           */
       uint32_t RESERVED0[7];
  __IO uint32_t OD;             /*!< PG Open Drain Control Register   */
  __IO uint32_t PUP;            /*!< PG Pull-Up Control Register      */
       uint32_t RESERVED1[2];
  __IO uint32_t IE;             /*!< PG Input Enable Control Register */
} PG_TypeDef;

/**
  * @brief General Purpose Input/Output Port (PH)
  */
typedef struct
{
  __IO uint32_t DATA;           /*!< PH Register                      */
  __IO uint32_t CR;             /*!< PH Control Register              */
  __IO uint32_t FR1;            /*!< PH Function Register 1           */
       uint32_t RESERVED0[8];
  __IO uint32_t PUP;            /*!< PH Pull-Up Control Register      */
       uint32_t RESERVED1[2];
  __IO uint32_t IE;             /*!< PH Input Enable Control Register */
} PH_TypeDef;

/**
  * @brief General Purpose Input/Output Port (PI)
  */
typedef struct
{
  __IO uint32_t DATA;           /*!< PI Register                      */
  __IO uint32_t CR;             /*!< PI Control Register              */
  __IO uint32_t FR1;            /*!< PI Function Register 1           */
       uint32_t RESERVED0[8];
  __IO uint32_t PUP;            /*!< PI Pull-Up Control Register      */
       uint32_t RESERVED1[2];
  __IO uint32_t IE;             /*!< PI Input Enable Control Register */
} PI_TypeDef;

/**
  * @brief General Purpose Input/Output Port (PJ)
  */
typedef struct
{
  __IO uint32_t DATA;           /*!< PJ Register                      */
  __IO uint32_t CR;             /*!< PJ Control Register              */
  __IO uint32_t FR1;            /*!< PJ Function Register 1           */
       uint32_t RESERVED0[8];
  __IO uint32_t PUP;            /*!< PJ Pull-Up Control Register      */
       uint32_t RESERVED1[2];
  __IO uint32_t IE;             /*!< PJ Input Enable Control Register */
} PJ_TypeDef;

/**
  * @brief General Purpose Input/Output Port (PK)
  */
typedef struct
{
  __IO uint32_t DATA;           /*!< PK Register                      */
  __IO uint32_t CR;             /*!< PK Control Register              */
  __IO uint32_t FR1;            /*!< PK Function Register 1           */
  __IO uint32_t FR2;            /*!< PK Function Register 2           */
       uint32_t RESERVED0[7];
  __IO uint32_t PUP;            /*!< PK Pull-Up Control Register      */
       uint32_t RESERVED1[2];
  __IO uint32_t IE;             /*!< PK Input Enable Control Register */
} PK_TypeDef;

/**
  * @brief 16-bit Timer/Event Counter (TB)
  */
typedef struct
{
  __IO uint32_t EN;             /*!< TB Enable Register            */
  __IO uint32_t RUN;            /*!< TB RUN Register               */
  __IO uint32_t CR;             /*!< TB Control Register           */
  __IO uint32_t MOD;            /*!< TB Mode Register              */
  __IO uint32_t FFCR;           /*!< TB Flip-Flop Control Register */
  __I  uint32_t ST;             /*!< TB Status Register            */
  __I  uint32_t IM;             /*!< TB Interrupt Mask Register    */
  __I  uint32_t UC;             /*!< TB Read Capture Register      */
  __IO uint32_t RG0;            /*!< TB RG0 Timer Register         */
  __IO uint32_t RG1;            /*!< TB RG1 Timer Register         */
  __I  uint32_t CP0;            /*!< TB CP0 Capture Register       */
  __I  uint32_t CP1;            /*!< TB CP1 Capture Register       */
} TB_TypeDef;

/**
  * @brief Serial Bus Interface (SBI)
  */
typedef struct
{
  __IO uint32_t CR0;            /*!< SBI Control Register 0       */
  __IO uint32_t CR1;            /*!< SBI Control Register 1       */
  __IO uint32_t DBR;            /*!< SBI Data Buffer Register     */
  __IO uint32_t I2CAR;          /*!< SBI I2C Bus Address Register */
  union {
  __O  uint32_t CR2;            /*!< SBI Control Register 2       */
  __I  uint32_t SR;             /*!< SBI Status Register          */
  }CR2SR;
  __IO uint32_t BR0;            /*!< SBI Baud Rate Register 0     */
} SBI_TypeDef;
#define CR2 CR2SR.CR2
#define SR  CR2SR.SR

/**
  * @brief Serial Channel (SC)
  */
typedef struct
{
  __IO uint32_t EN;             /*!< SC Enable Register                        */
  __IO uint32_t BUF;            /*!< SC Buffer Register                        */
  __IO uint32_t CR;             /*!< SC Control Register                       */
  __IO uint32_t MOD0;           /*!< SC Mode Control Register 0                */
  __IO uint32_t BRCR;           /*!< SC Baud Rate Generator Control Register   */
  __IO uint32_t BRADD;          /*!< SC Baud Rate Generator Control Register 2 */
  __IO uint32_t MOD1;           /*!< SC Mode Control Register 1                */
  __IO uint32_t MOD2;           /*!< SC Mode Control Register 2                */
  __IO uint32_t RFC;            /*!< SC RX FIFO Configuration Register         */
  __IO uint32_t TFC;            /*!< SC TX FIFO Configuration Register         */
  __I  uint32_t RST;            /*!< SC RX FIFO Status Register                */
  __I  uint32_t TST;            /*!< SC TX FIFO Status Register                */
  __IO uint32_t FCNF;           /*!< SC FIFO Configuration Register            */
} SC_TypeDef;

/**
  * @brief Analog-to-Digital Converter (AD)
  */
typedef struct
{
  __IO uint32_t CLK;            /*!< AD Conversion Clock Setting Register       */
  __IO uint32_t MOD0;           /*!< AD Mode Control Register 0                 */
  __IO uint32_t MOD1;           /*!< AD Mode Control Register 1                 */
  __IO uint32_t MOD2;           /*!< AD Mode Control Register 2                 */
  __IO uint32_t MOD3;           /*!< AD Mode Control Register 3                 */
  __IO uint32_t MOD4;           /*!< AD Mode Control Register 4                 */
  __IO uint32_t MOD5;           /*!< AD Mode Control Register 5                 */
       uint32_t RESERVED0;
  __IO uint32_t BAS;            /*!< AD Conversion Accuracy Setting Register    */
       uint32_t RESERVED1[3];
  __I  uint32_t REG08;          /*!< AD Conversion Result Register 08           */
  __I  uint32_t REG19;          /*!< AD Conversion Result Register 19           */
  __I  uint32_t REG2A;          /*!< AD Conversion Result Register 2A           */
  __I  uint32_t REG3B;          /*!< AD Conversion Result Register 3B           */
  __I  uint32_t REG4C;          /*!< AD Conversion Result Register 4C           */
  __I  uint32_t REG5D;          /*!< AD Conversion Result Register 5D           */
  __I  uint32_t REG6E;          /*!< AD Conversion Result Register 6E           */
  __I  uint32_t REG7F;          /*!< AD Conversion Result Register 7F           */
  __I  uint32_t REGSP;          /*!< AD Conversion Result Register SP           */
  __IO uint32_t CMP0;           /*!< AD Conversion Result Comparison Register 0 */
  __IO uint32_t CMP1;           /*!< AD Conversion Result Comparison Register 1 */
} AD_TypeDef;

/**
  * @brief Watchdog Timer (WD)
  */
typedef struct
{
  __IO uint32_t MOD;            /*!< WD Mode Register    */
  __O  uint32_t CR;             /*!< WD Control Register */
} WD_TypeDef;

/**
  * @brief Real Time Clock (RTC)
  */
typedef struct
{
  __IO uint8_t SECR;            /*!< RTC Second Column Register          */
  __IO uint8_t MINR;            /*!< RTC Minute Column Register          */
  __IO uint8_t HOURR;           /*!< RTC Hour Column Register            */
       uint8_t RESERVED0;
  __IO uint8_t DAYR;            /*!< RTC Day of the Week Column Register */
  __IO uint8_t DATER;           /*!< RTC Day Column Register             */
  __IO uint8_t MONTHR;          /*!< RTC Month Column Register           */
  __IO uint8_t YEARR;           /*!< RTC Year Column Register            */
  __IO uint8_t PAGER;           /*!< RTC PAGE Register                   */
       uint8_t RESERVED1[3];
  __IO uint8_t RESTR;           /*!< RTC Reset Register                  */
} RTC_TypeDef;

/**
  * @brief Consumer Electronics Control (CEC)
  */
typedef struct
{
  __IO uint32_t EN;             /*!< CEC Enable Register                    */
  __IO uint32_t ADD;            /*!< CEC Logical Address Register           */
  __IO uint32_t RESET;          /*!< CEC Software Reset Register            */
  __IO uint32_t REN;            /*!< CEC Receive Enable Register            */
  __I  uint32_t RBUF;           /*!< CEC Receive Buffer Register            */
  __IO uint32_t RCR1;           /*!< CEC Receive Control Register 1         */
  __IO uint32_t RCR2;           /*!< CEC Receive Control Register 2         */
  __IO uint32_t RCR3;           /*!< CEC Receive Control Register 3         */
  __IO uint32_t TEN;            /*!< CEC Transmit Enable Register           */
  __IO uint32_t TBUF;           /*!< CEC Transmit Buffer Register           */
  __IO uint32_t TCR;            /*!< CEC Transmit Control Register          */
  __I  uint32_t RSTAT;          /*!< CEC Receive Interrupt Status Register  */
  __I  uint32_t TSTAT;          /*!< CEC Transmit Interrupt Status Register */
} CEC_TypeDef;

/**
  * @brief Remote Control Signal Preprocessor (RMC)
  */
typedef struct
{
  __IO uint32_t EN;             /*!< RMC Enable Register                */
  __IO uint32_t REN;            /*!< RMC Receive Enable Register        */
  __I  uint32_t RBUF1;          /*!< RMC Receive Data Buffer Register 1 */
  __I  uint32_t RBUF2;          /*!< RMC Receive Data Buffer Register 2 */
  __I  uint32_t RBUF3;          /*!< RMC Receive Data Buffer Register 3 */
  __IO uint32_t RCR1;           /*!< RMC Receive Control Register 1     */
  __IO uint32_t RCR2;           /*!< RMC Receive Control Register 2     */
  __IO uint32_t RCR3;           /*!< RMC Receive Control Register 3     */
  __IO uint32_t RCR4;           /*!< RMC Receive Control Register 4     */
  __I  uint32_t RSTAT;          /*!< RMC Receive Status Register        */
} RMC_TypeDef;

/**
  * @brief Flash Control (FC)
  */
typedef struct
{
  __I  uint32_t SECBIT;         /*!< FC Security Bit Register  */
       uint32_t RESERVED0[7];
  __I  uint32_t FLCS;           /*!< FC Flash Control Register */
} FC_TypeDef;


/* Memory map */
#define FLASH_BASE            (0x00000000UL)
#define RAM_BASE              (0x20000000UL)
#define PERI_BASE             (0x40000000UL)

#define PA_BASE               (PERI_BASE  + 0x00000)
#define PB_BASE               (PERI_BASE  + 0x00040)
#define PC_BASE               (PERI_BASE  + 0x00080)
#define PD_BASE               (PERI_BASE  + 0x000C0)
#define PE_BASE               (PERI_BASE  + 0x00100)
#define PF_BASE               (PERI_BASE  + 0x00140)
#define PG_BASE               (PERI_BASE  + 0x00180)
#define PH_BASE               (PERI_BASE  + 0x001C0)
#define PI_BASE               (PERI_BASE  + 0x00200)
#define PJ_BASE               (PERI_BASE  + 0x00240)
#define PK_BASE               (PERI_BASE  + 0x00280)
#define TB0_BASE              (PERI_BASE  + 0x10000)
#define TB1_BASE              (PERI_BASE  + 0x10040)
#define TB2_BASE              (PERI_BASE  + 0x10080)
#define TB3_BASE              (PERI_BASE  + 0x100C0)
#define TB4_BASE              (PERI_BASE  + 0x10100)
#define TB5_BASE              (PERI_BASE  + 0x10140)
#define TB6_BASE              (PERI_BASE  + 0x10180)
#define TB7_BASE              (PERI_BASE  + 0x101C0)
#define TB8_BASE              (PERI_BASE  + 0x10200)
#define TB9_BASE              (PERI_BASE  + 0x10240)
#define SBI0_BASE             (PERI_BASE  + 0x20000)
#define SBI1_BASE             (PERI_BASE  + 0x20020)
#define SBI2_BASE             (PERI_BASE  + 0x20040)
#define SC0_BASE              (PERI_BASE  + 0x20080)
#define SC1_BASE              (PERI_BASE  + 0x200C0)
#define SC2_BASE              (PERI_BASE  + 0x20100)
#define AD_BASE               (PERI_BASE  + 0x30000)
#define WD_BASE               (PERI_BASE  + 0x40000)
#define RTC_BASE              (PERI_BASE  + 0x40100)
#define CG_BASE               (PERI_BASE  + 0x40200)
#define CEC_BASE              (PERI_BASE  + 0x40300)
#define RMC0_BASE             (PERI_BASE  + 0x40400)
#define RMC1_BASE             (PERI_BASE  + 0x40440)
#define FC_BASE               (PERI_BASE  + 0x40500)


/* Peripheral declaration */
#define PA                    ((     PA_TypeDef *)       PA_BASE)
#define PB                    ((     PB_TypeDef *)       PB_BASE)
#define PC                    ((     PC_TypeDef *)       PC_BASE)
#define PD                    ((     PD_TypeDef *)       PD_BASE)
#define PE                    ((     PE_TypeDef *)       PE_BASE)
#define PF                    ((     PF_TypeDef *)       PF_BASE)
#define PG                    ((     PG_TypeDef *)       PG_BASE)
#define PH                    ((     PH_TypeDef *)       PH_BASE)
#define PI                    ((     PI_TypeDef *)       PI_BASE)
#define PJ                    ((     PJ_TypeDef *)       PJ_BASE)
#define PK                    ((     PK_TypeDef *)       PK_BASE)
#define TB0                   ((     TB_TypeDef *)      TB0_BASE)
#define TB1                   ((     TB_TypeDef *)      TB1_BASE)
#define TB2                   ((     TB_TypeDef *)      TB2_BASE)
#define TB3                   ((     TB_TypeDef *)      TB3_BASE)
#define TB4                   ((     TB_TypeDef *)      TB4_BASE)
#define TB5                   ((     TB_TypeDef *)      TB5_BASE)
#define TB6                   ((     TB_TypeDef *)      TB6_BASE)
#define TB7                   ((     TB_TypeDef *)      TB7_BASE)
#define TB8                   ((     TB_TypeDef *)      TB8_BASE)
#define TB9                   ((     TB_TypeDef *)      TB9_BASE)
#define SBI0                  ((    SBI_TypeDef *)     SBI0_BASE)
#define SBI1                  ((    SBI_TypeDef *)     SBI1_BASE)
#define SBI2                  ((    SBI_TypeDef *)     SBI2_BASE)
#define SC0                   ((     SC_TypeDef *)      SC0_BASE)
#define SC1                   ((     SC_TypeDef *)      SC1_BASE)
#define SC2                   ((     SC_TypeDef *)      SC2_BASE)
#define AD                    ((     AD_TypeDef *)       AD_BASE)
#define WD                    ((     WD_TypeDef *)       WD_BASE)
#define RTC                   ((    RTC_TypeDef *)      RTC_BASE)
#define CG                    ((     CG_TypeDef *)       CG_BASE)
#define CEC                   ((    CEC_TypeDef *)      CEC_BASE)
#define RMC0                  ((    RMC_TypeDef *)     RMC0_BASE)
#define RMC1                  ((    RMC_TypeDef *)     RMC1_BASE)
#define FC                    ((     FC_TypeDef *)       FC_BASE)

/* Bit-Band for Device Specific Peripheral Registers */
#define BITBAND_OFFSET (0x02000000UL)
#define BITBAND_PERI_BASE (PERI_BASE + BITBAND_OFFSET)
#define BITBAND_PERI(addr, bitnum) (BITBAND_PERI_BASE + ((unsigned long)(addr - PERI_BASE) << 5) + (bitnum << 2))

/* General Purpose Input/Output Port (PA) */
#define PA_DATA_PA0           (*((__IO uint32_t *)(BITBAND_PERI(&PA->DATA,0))))
#define PA_DATA_PA1           (*((__IO uint32_t *)(BITBAND_PERI(&PA->DATA,1))))
#define PA_DATA_PA2           (*((__IO uint32_t *)(BITBAND_PERI(&PA->DATA,2))))
#define PA_DATA_PA3           (*((__IO uint32_t *)(BITBAND_PERI(&PA->DATA,3))))
#define PA_DATA_PA4           (*((__IO uint32_t *)(BITBAND_PERI(&PA->DATA,4))))
#define PA_DATA_PA5           (*((__IO uint32_t *)(BITBAND_PERI(&PA->DATA,5))))
#define PA_DATA_PA6           (*((__IO uint32_t *)(BITBAND_PERI(&PA->DATA,6))))
#define PA_DATA_PA7           (*((__IO uint32_t *)(BITBAND_PERI(&PA->DATA,7))))
#define PA_CR_PA0C            (*((__IO uint32_t *)(BITBAND_PERI(&PA->CR,0))))
#define PA_CR_PA1C            (*((__IO uint32_t *)(BITBAND_PERI(&PA->CR,1))))
#define PA_CR_PA2C            (*((__IO uint32_t *)(BITBAND_PERI(&PA->CR,2))))
#define PA_CR_PA3C            (*((__IO uint32_t *)(BITBAND_PERI(&PA->CR,3))))
#define PA_CR_PA4C            (*((__IO uint32_t *)(BITBAND_PERI(&PA->CR,4))))
#define PA_CR_PA5C            (*((__IO uint32_t *)(BITBAND_PERI(&PA->CR,5))))
#define PA_CR_PA6C            (*((__IO uint32_t *)(BITBAND_PERI(&PA->CR,6))))
#define PA_CR_PA7C            (*((__IO uint32_t *)(BITBAND_PERI(&PA->CR,7))))
#define PA_FR1_PA0F1          (*((__IO uint32_t *)(BITBAND_PERI(&PA->FR1,0))))
#define PA_FR1_PA1F1          (*((__IO uint32_t *)(BITBAND_PERI(&PA->FR1,1))))
#define PA_FR1_PA2F1          (*((__IO uint32_t *)(BITBAND_PERI(&PA->FR1,2))))
#define PA_FR1_PA3F1          (*((__IO uint32_t *)(BITBAND_PERI(&PA->FR1,3))))
#define PA_FR1_PA4F1          (*((__IO uint32_t *)(BITBAND_PERI(&PA->FR1,4))))
#define PA_FR1_PA5F1          (*((__IO uint32_t *)(BITBAND_PERI(&PA->FR1,5))))
#define PA_FR1_PA6F1          (*((__IO uint32_t *)(BITBAND_PERI(&PA->FR1,6))))
#define PA_PUP_PA0UP          (*((__IO uint32_t *)(BITBAND_PERI(&PA->PUP,0))))
#define PA_PUP_PA2UP          (*((__IO uint32_t *)(BITBAND_PERI(&PA->PUP,2))))
#define PA_PUP_PA3UP          (*((__IO uint32_t *)(BITBAND_PERI(&PA->PUP,3))))
#define PA_PUP_PA4UP          (*((__IO uint32_t *)(BITBAND_PERI(&PA->PUP,4))))
#define PA_PUP_PA5UP          (*((__IO uint32_t *)(BITBAND_PERI(&PA->PUP,5))))
#define PA_PUP_PA6UP          (*((__IO uint32_t *)(BITBAND_PERI(&PA->PUP,6))))
#define PA_PUP_PA7UP          (*((__IO uint32_t *)(BITBAND_PERI(&PA->PUP,7))))
#define PA_PDN_PA1DN          (*((__IO uint32_t *)(BITBAND_PERI(&PA->PDN,1))))
#define PA_IE_PA0IE           (*((__IO uint32_t *)(BITBAND_PERI(&PA->IE,0))))
#define PA_IE_PA1IE           (*((__IO uint32_t *)(BITBAND_PERI(&PA->IE,1))))
#define PA_IE_PA2IE           (*((__IO uint32_t *)(BITBAND_PERI(&PA->IE,2))))
#define PA_IE_PA3IE           (*((__IO uint32_t *)(BITBAND_PERI(&PA->IE,3))))
#define PA_IE_PA4IE           (*((__IO uint32_t *)(BITBAND_PERI(&PA->IE,4))))
#define PA_IE_PA5IE           (*((__IO uint32_t *)(BITBAND_PERI(&PA->IE,5))))
#define PA_IE_PA6IE           (*((__IO uint32_t *)(BITBAND_PERI(&PA->IE,6))))
#define PA_IE_PA7IE           (*((__IO uint32_t *)(BITBAND_PERI(&PA->IE,7))))

/* General Purpose Input/Output Port (PB) */
#define PB_DATA_PB0           (*((__IO uint32_t *)(BITBAND_PERI(&PB->DATA,0))))
#define PB_DATA_PB1           (*((__IO uint32_t *)(BITBAND_PERI(&PB->DATA,1))))
#define PB_DATA_PB2           (*((__IO uint32_t *)(BITBAND_PERI(&PB->DATA,2))))
#define PB_DATA_PB3           (*((__IO uint32_t *)(BITBAND_PERI(&PB->DATA,3))))
#define PB_DATA_PB4           (*((__IO uint32_t *)(BITBAND_PERI(&PB->DATA,4))))
#define PB_DATA_PB5           (*((__IO uint32_t *)(BITBAND_PERI(&PB->DATA,5))))
#define PB_DATA_PB6           (*((__IO uint32_t *)(BITBAND_PERI(&PB->DATA,6))))
#define PB_DATA_PB7           (*((__IO uint32_t *)(BITBAND_PERI(&PB->DATA,7))))
#define PB_CR_PB0C            (*((__IO uint32_t *)(BITBAND_PERI(&PB->CR,0))))
#define PB_CR_PB1C            (*((__IO uint32_t *)(BITBAND_PERI(&PB->CR,1))))
#define PB_CR_PB2C            (*((__IO uint32_t *)(BITBAND_PERI(&PB->CR,2))))
#define PB_CR_PB3C            (*((__IO uint32_t *)(BITBAND_PERI(&PB->CR,3))))
#define PB_CR_PB4C            (*((__IO uint32_t *)(BITBAND_PERI(&PB->CR,4))))
#define PB_CR_PB5C            (*((__IO uint32_t *)(BITBAND_PERI(&PB->CR,5))))
#define PB_CR_PB6C            (*((__IO uint32_t *)(BITBAND_PERI(&PB->CR,6))))
#define PB_CR_PB7C            (*((__IO uint32_t *)(BITBAND_PERI(&PB->CR,7))))
#define PB_FR1_PB0F1          (*((__IO uint32_t *)(BITBAND_PERI(&PB->FR1,0))))
#define PB_FR1_PB1F1          (*((__IO uint32_t *)(BITBAND_PERI(&PB->FR1,1))))
#define PB_FR1_PB2F1          (*((__IO uint32_t *)(BITBAND_PERI(&PB->FR1,2))))
#define PB_PUP_PB0UP          (*((__IO uint32_t *)(BITBAND_PERI(&PB->PUP,0))))
#define PB_PUP_PB1UP          (*((__IO uint32_t *)(BITBAND_PERI(&PB->PUP,1))))
#define PB_PUP_PB2UP          (*((__IO uint32_t *)(BITBAND_PERI(&PB->PUP,2))))
#define PB_PUP_PB3UP          (*((__IO uint32_t *)(BITBAND_PERI(&PB->PUP,3))))
#define PB_PUP_PB4UP          (*((__IO uint32_t *)(BITBAND_PERI(&PB->PUP,4))))
#define PB_PUP_PB5UP          (*((__IO uint32_t *)(BITBAND_PERI(&PB->PUP,5))))
#define PB_PUP_PB6UP          (*((__IO uint32_t *)(BITBAND_PERI(&PB->PUP,6))))
#define PB_PUP_PB7UP          (*((__IO uint32_t *)(BITBAND_PERI(&PB->PUP,7))))
#define PB_IE_PB0IE           (*((__IO uint32_t *)(BITBAND_PERI(&PB->IE,0))))
#define PB_IE_PB1IE           (*((__IO uint32_t *)(BITBAND_PERI(&PB->IE,1))))
#define PB_IE_PB2IE           (*((__IO uint32_t *)(BITBAND_PERI(&PB->IE,2))))
#define PB_IE_PB3IE           (*((__IO uint32_t *)(BITBAND_PERI(&PB->IE,3))))
#define PB_IE_PB4IE           (*((__IO uint32_t *)(BITBAND_PERI(&PB->IE,4))))
#define PB_IE_PB5IE           (*((__IO uint32_t *)(BITBAND_PERI(&PB->IE,5))))
#define PB_IE_PB6IE           (*((__IO uint32_t *)(BITBAND_PERI(&PB->IE,6))))
#define PB_IE_PB7IE           (*((__IO uint32_t *)(BITBAND_PERI(&PB->IE,7))))

/* General Purpose Input/Output Port (PC) */
#define PC_DATA_PC0           (*((__I uint32_t *)(BITBAND_PERI(&PC->DATA,0))))
#define PC_DATA_PC1           (*((__I uint32_t *)(BITBAND_PERI(&PC->DATA,1))))
#define PC_DATA_PC2           (*((__I uint32_t *)(BITBAND_PERI(&PC->DATA,2))))
#define PC_DATA_PC3           (*((__I uint32_t *)(BITBAND_PERI(&PC->DATA,3))))
#define PC_PUP_PC0UP          (*((__IO uint32_t *)(BITBAND_PERI(&PC->PUP,0))))
#define PC_PUP_PC1UP          (*((__IO uint32_t *)(BITBAND_PERI(&PC->PUP,1))))
#define PC_PUP_PC2UP          (*((__IO uint32_t *)(BITBAND_PERI(&PC->PUP,2))))
#define PC_PUP_PC3UP          (*((__IO uint32_t *)(BITBAND_PERI(&PC->PUP,3))))
#define PC_IE_PC0IE           (*((__IO uint32_t *)(BITBAND_PERI(&PC->IE,0))))
#define PC_IE_PC1IE           (*((__IO uint32_t *)(BITBAND_PERI(&PC->IE,1))))
#define PC_IE_PC2IE           (*((__IO uint32_t *)(BITBAND_PERI(&PC->IE,2))))
#define PC_IE_PC3IE           (*((__IO uint32_t *)(BITBAND_PERI(&PC->IE,3))))

/* General Purpose Input/Output Port (PD) */
#define PD_DATA_PD0           (*((__I uint32_t *)(BITBAND_PERI(&PD->DATA,0))))
#define PD_DATA_PD1           (*((__I uint32_t *)(BITBAND_PERI(&PD->DATA,1))))
#define PD_DATA_PD2           (*((__I uint32_t *)(BITBAND_PERI(&PD->DATA,2))))
#define PD_DATA_PD3           (*((__I uint32_t *)(BITBAND_PERI(&PD->DATA,3))))
#define PD_DATA_PD4           (*((__I uint32_t *)(BITBAND_PERI(&PD->DATA,4))))
#define PD_DATA_PD5           (*((__I uint32_t *)(BITBAND_PERI(&PD->DATA,5))))
#define PD_DATA_PD6           (*((__I uint32_t *)(BITBAND_PERI(&PD->DATA,6))))
#define PD_DATA_PD7           (*((__I uint32_t *)(BITBAND_PERI(&PD->DATA,7))))
#define PD_FR1_PD0F1          (*((__IO uint32_t *)(BITBAND_PERI(&PD->FR1,0))))
#define PD_FR1_PD1F1          (*((__IO uint32_t *)(BITBAND_PERI(&PD->FR1,1))))
#define PD_FR1_PD2F1          (*((__IO uint32_t *)(BITBAND_PERI(&PD->FR1,2))))
#define PD_FR1_PD3F1          (*((__IO uint32_t *)(BITBAND_PERI(&PD->FR1,3))))
#define PD_PUP_PD0UP          (*((__IO uint32_t *)(BITBAND_PERI(&PD->PUP,0))))
#define PD_PUP_PD1UP          (*((__IO uint32_t *)(BITBAND_PERI(&PD->PUP,1))))
#define PD_PUP_PD2UP          (*((__IO uint32_t *)(BITBAND_PERI(&PD->PUP,2))))
#define PD_PUP_PD3UP          (*((__IO uint32_t *)(BITBAND_PERI(&PD->PUP,3))))
#define PD_PUP_PD4UP          (*((__IO uint32_t *)(BITBAND_PERI(&PD->PUP,4))))
#define PD_PUP_PD5UP          (*((__IO uint32_t *)(BITBAND_PERI(&PD->PUP,5))))
#define PD_PUP_PD6UP          (*((__IO uint32_t *)(BITBAND_PERI(&PD->PUP,6))))
#define PD_PUP_PD7UP          (*((__IO uint32_t *)(BITBAND_PERI(&PD->PUP,7))))
#define PD_IE_PD0IE           (*((__IO uint32_t *)(BITBAND_PERI(&PD->IE,0))))
#define PD_IE_PD1IE           (*((__IO uint32_t *)(BITBAND_PERI(&PD->IE,1))))
#define PD_IE_PD2IE           (*((__IO uint32_t *)(BITBAND_PERI(&PD->IE,2))))
#define PD_IE_PD3IE           (*((__IO uint32_t *)(BITBAND_PERI(&PD->IE,3))))
#define PD_IE_PD4IE           (*((__IO uint32_t *)(BITBAND_PERI(&PD->IE,4))))
#define PD_IE_PD5IE           (*((__IO uint32_t *)(BITBAND_PERI(&PD->IE,5))))
#define PD_IE_PD6IE           (*((__IO uint32_t *)(BITBAND_PERI(&PD->IE,6))))
#define PD_IE_PD7IE           (*((__IO uint32_t *)(BITBAND_PERI(&PD->IE,7))))

/* General Purpose Input/Output Port (PE) */
#define PE_DATA_PE0           (*((__IO uint32_t *)(BITBAND_PERI(&PE->DATA,0))))
#define PE_DATA_PE1           (*((__IO uint32_t *)(BITBAND_PERI(&PE->DATA,1))))
#define PE_DATA_PE2           (*((__IO uint32_t *)(BITBAND_PERI(&PE->DATA,2))))
#define PE_DATA_PE3           (*((__IO uint32_t *)(BITBAND_PERI(&PE->DATA,3))))
#define PE_DATA_PE4           (*((__IO uint32_t *)(BITBAND_PERI(&PE->DATA,4))))
#define PE_DATA_PE5           (*((__IO uint32_t *)(BITBAND_PERI(&PE->DATA,5))))
#define PE_DATA_PE6           (*((__IO uint32_t *)(BITBAND_PERI(&PE->DATA,6))))
#define PE_CR_PE0C            (*((__IO uint32_t *)(BITBAND_PERI(&PE->CR,0))))
#define PE_CR_PE1C            (*((__IO uint32_t *)(BITBAND_PERI(&PE->CR,1))))
#define PE_CR_PE2C            (*((__IO uint32_t *)(BITBAND_PERI(&PE->CR,2))))
#define PE_CR_PE3C            (*((__IO uint32_t *)(BITBAND_PERI(&PE->CR,3))))
#define PE_CR_PE4C            (*((__IO uint32_t *)(BITBAND_PERI(&PE->CR,4))))
#define PE_CR_PE5C            (*((__IO uint32_t *)(BITBAND_PERI(&PE->CR,5))))
#define PE_CR_PE6C            (*((__IO uint32_t *)(BITBAND_PERI(&PE->CR,6))))
#define PE_FR1_PE0F1          (*((__IO uint32_t *)(BITBAND_PERI(&PE->FR1,0))))
#define PE_FR1_PE1F1          (*((__IO uint32_t *)(BITBAND_PERI(&PE->FR1,1))))
#define PE_FR1_PE2F1          (*((__IO uint32_t *)(BITBAND_PERI(&PE->FR1,2))))
#define PE_FR1_PE3F1          (*((__IO uint32_t *)(BITBAND_PERI(&PE->FR1,3))))
#define PE_FR1_PE4F1          (*((__IO uint32_t *)(BITBAND_PERI(&PE->FR1,4))))
#define PE_FR1_PE5F1          (*((__IO uint32_t *)(BITBAND_PERI(&PE->FR1,5))))
#define PE_FR1_PE6F1          (*((__IO uint32_t *)(BITBAND_PERI(&PE->FR1,6))))
#define PE_FR2_PE2F2          (*((__IO uint32_t *)(BITBAND_PERI(&PE->FR2,2))))
#define PE_FR2_PE6F2          (*((__IO uint32_t *)(BITBAND_PERI(&PE->FR2,6))))
#define PE_OD_PE0OD           (*((__IO uint32_t *)(BITBAND_PERI(&PE->OD,0))))
#define PE_OD_PE1OD           (*((__IO uint32_t *)(BITBAND_PERI(&PE->OD,1))))
#define PE_OD_PE2OD           (*((__IO uint32_t *)(BITBAND_PERI(&PE->OD,2))))
#define PE_OD_PE3OD           (*((__IO uint32_t *)(BITBAND_PERI(&PE->OD,3))))
#define PE_OD_PE4OD           (*((__IO uint32_t *)(BITBAND_PERI(&PE->OD,4))))
#define PE_OD_PE5OD           (*((__IO uint32_t *)(BITBAND_PERI(&PE->OD,5))))
#define PE_OD_PE6OD           (*((__IO uint32_t *)(BITBAND_PERI(&PE->OD,6))))
#define PE_PUP_PE0UP          (*((__IO uint32_t *)(BITBAND_PERI(&PE->PUP,0))))
#define PE_PUP_PE1UP          (*((__IO uint32_t *)(BITBAND_PERI(&PE->PUP,1))))
#define PE_PUP_PE2UP          (*((__IO uint32_t *)(BITBAND_PERI(&PE->PUP,2))))
#define PE_PUP_PE3UP          (*((__IO uint32_t *)(BITBAND_PERI(&PE->PUP,3))))
#define PE_PUP_PE4UP          (*((__IO uint32_t *)(BITBAND_PERI(&PE->PUP,4))))
#define PE_PUP_PE5UP          (*((__IO uint32_t *)(BITBAND_PERI(&PE->PUP,5))))
#define PE_PUP_PE6UP          (*((__IO uint32_t *)(BITBAND_PERI(&PE->PUP,6))))
#define PE_IE_PE0IE           (*((__IO uint32_t *)(BITBAND_PERI(&PE->IE,0))))
#define PE_IE_PE1IE           (*((__IO uint32_t *)(BITBAND_PERI(&PE->IE,1))))
#define PE_IE_PE2IE           (*((__IO uint32_t *)(BITBAND_PERI(&PE->IE,2))))
#define PE_IE_PE3IE           (*((__IO uint32_t *)(BITBAND_PERI(&PE->IE,3))))
#define PE_IE_PE4IE           (*((__IO uint32_t *)(BITBAND_PERI(&PE->IE,4))))
#define PE_IE_PE5IE           (*((__IO uint32_t *)(BITBAND_PERI(&PE->IE,5))))
#define PE_IE_PE6IE           (*((__IO uint32_t *)(BITBAND_PERI(&PE->IE,6))))

/* General Purpose Input/Output Port (PF) */
#define PF_DATA_PF0           (*((__IO uint32_t *)(BITBAND_PERI(&PF->DATA,0))))
#define PF_DATA_PF1           (*((__IO uint32_t *)(BITBAND_PERI(&PF->DATA,1))))
#define PF_DATA_PF2           (*((__IO uint32_t *)(BITBAND_PERI(&PF->DATA,2))))
#define PF_DATA_PF3           (*((__IO uint32_t *)(BITBAND_PERI(&PF->DATA,3))))
#define PF_DATA_PF4           (*((__IO uint32_t *)(BITBAND_PERI(&PF->DATA,4))))
#define PF_DATA_PF5           (*((__IO uint32_t *)(BITBAND_PERI(&PF->DATA,5))))
#define PF_DATA_PF6           (*((__IO uint32_t *)(BITBAND_PERI(&PF->DATA,6))))
#define PF_DATA_PF7           (*((__IO uint32_t *)(BITBAND_PERI(&PF->DATA,7))))
#define PF_CR_PF0C            (*((__IO uint32_t *)(BITBAND_PERI(&PF->CR,0))))
#define PF_CR_PF1C            (*((__IO uint32_t *)(BITBAND_PERI(&PF->CR,1))))
#define PF_CR_PF2C            (*((__IO uint32_t *)(BITBAND_PERI(&PF->CR,2))))
#define PF_CR_PF3C            (*((__IO uint32_t *)(BITBAND_PERI(&PF->CR,3))))
#define PF_CR_PF4C            (*((__IO uint32_t *)(BITBAND_PERI(&PF->CR,4))))
#define PF_CR_PF5C            (*((__IO uint32_t *)(BITBAND_PERI(&PF->CR,5))))
#define PF_CR_PF6C            (*((__IO uint32_t *)(BITBAND_PERI(&PF->CR,6))))
#define PF_CR_PF7C            (*((__IO uint32_t *)(BITBAND_PERI(&PF->CR,7))))
#define PF_FR1_PF0F1          (*((__IO uint32_t *)(BITBAND_PERI(&PF->FR1,0))))
#define PF_FR1_PF1F1          (*((__IO uint32_t *)(BITBAND_PERI(&PF->FR1,1))))
#define PF_FR1_PF2F1          (*((__IO uint32_t *)(BITBAND_PERI(&PF->FR1,2))))
#define PF_FR1_PF3F1          (*((__IO uint32_t *)(BITBAND_PERI(&PF->FR1,3))))
#define PF_FR1_PF4F1          (*((__IO uint32_t *)(BITBAND_PERI(&PF->FR1,4))))
#define PF_FR1_PF5F1          (*((__IO uint32_t *)(BITBAND_PERI(&PF->FR1,5))))
#define PF_FR1_PF6F1          (*((__IO uint32_t *)(BITBAND_PERI(&PF->FR1,6))))
#define PF_FR1_PF7F1          (*((__IO uint32_t *)(BITBAND_PERI(&PF->FR1,7))))
#define PF_FR2_PF2F2          (*((__IO uint32_t *)(BITBAND_PERI(&PF->FR2,2))))
#define PF_OD_PF0OD           (*((__IO uint32_t *)(BITBAND_PERI(&PF->OD,0))))
#define PF_OD_PF1OD           (*((__IO uint32_t *)(BITBAND_PERI(&PF->OD,1))))
#define PF_OD_PF2OD           (*((__IO uint32_t *)(BITBAND_PERI(&PF->OD,2))))
#define PF_OD_PF3OD           (*((__IO uint32_t *)(BITBAND_PERI(&PF->OD,3))))
#define PF_OD_PF4OD           (*((__IO uint32_t *)(BITBAND_PERI(&PF->OD,4))))
#define PF_OD_PF5OD           (*((__IO uint32_t *)(BITBAND_PERI(&PF->OD,5))))
#define PF_OD_PF6OD           (*((__IO uint32_t *)(BITBAND_PERI(&PF->OD,6))))
#define PF_OD_PF7OD           (*((__IO uint32_t *)(BITBAND_PERI(&PF->OD,7))))
#define PF_PUP_PF0UP          (*((__IO uint32_t *)(BITBAND_PERI(&PF->PUP,0))))
#define PF_PUP_PF1UP          (*((__IO uint32_t *)(BITBAND_PERI(&PF->PUP,1))))
#define PF_PUP_PF2UP          (*((__IO uint32_t *)(BITBAND_PERI(&PF->PUP,2))))
#define PF_PUP_PF3UP          (*((__IO uint32_t *)(BITBAND_PERI(&PF->PUP,3))))
#define PF_PUP_PF4UP          (*((__IO uint32_t *)(BITBAND_PERI(&PF->PUP,4))))
#define PF_PUP_PF5UP          (*((__IO uint32_t *)(BITBAND_PERI(&PF->PUP,5))))
#define PF_PUP_PF6UP          (*((__IO uint32_t *)(BITBAND_PERI(&PF->PUP,6))))
#define PF_PUP_PF7UP          (*((__IO uint32_t *)(BITBAND_PERI(&PF->PUP,7))))
#define PF_IE_PF0IE           (*((__IO uint32_t *)(BITBAND_PERI(&PF->IE,0))))
#define PF_IE_PF1IE           (*((__IO uint32_t *)(BITBAND_PERI(&PF->IE,1))))
#define PF_IE_PF2IE           (*((__IO uint32_t *)(BITBAND_PERI(&PF->IE,2))))
#define PF_IE_PF3IE           (*((__IO uint32_t *)(BITBAND_PERI(&PF->IE,3))))
#define PF_IE_PF4IE           (*((__IO uint32_t *)(BITBAND_PERI(&PF->IE,4))))
#define PF_IE_PF5IE           (*((__IO uint32_t *)(BITBAND_PERI(&PF->IE,5))))
#define PF_IE_PF6IE           (*((__IO uint32_t *)(BITBAND_PERI(&PF->IE,6))))
#define PF_IE_PF7IE           (*((__IO uint32_t *)(BITBAND_PERI(&PF->IE,7))))

/* General Purpose Input/Output Port (PG) */
#define PG_DATA_PG0           (*((__IO uint32_t *)(BITBAND_PERI(&PG->DATA,0))))
#define PG_DATA_PG1           (*((__IO uint32_t *)(BITBAND_PERI(&PG->DATA,1))))
#define PG_DATA_PG2           (*((__IO uint32_t *)(BITBAND_PERI(&PG->DATA,2))))
#define PG_DATA_PG3           (*((__IO uint32_t *)(BITBAND_PERI(&PG->DATA,3))))
#define PG_DATA_PG4           (*((__IO uint32_t *)(BITBAND_PERI(&PG->DATA,4))))
#define PG_DATA_PG5           (*((__IO uint32_t *)(BITBAND_PERI(&PG->DATA,5))))
#define PG_DATA_PG6           (*((__IO uint32_t *)(BITBAND_PERI(&PG->DATA,6))))
#define PG_DATA_PG7           (*((__IO uint32_t *)(BITBAND_PERI(&PG->DATA,7))))
#define PG_CR_PG0C            (*((__IO uint32_t *)(BITBAND_PERI(&PG->CR,0))))
#define PG_CR_PG1C            (*((__IO uint32_t *)(BITBAND_PERI(&PG->CR,1))))
#define PG_CR_PG2C            (*((__IO uint32_t *)(BITBAND_PERI(&PG->CR,2))))
#define PG_CR_PG3C            (*((__IO uint32_t *)(BITBAND_PERI(&PG->CR,3))))
#define PG_CR_PG4C            (*((__IO uint32_t *)(BITBAND_PERI(&PG->CR,4))))
#define PG_CR_PG5C            (*((__IO uint32_t *)(BITBAND_PERI(&PG->CR,5))))
#define PG_CR_PG6C            (*((__IO uint32_t *)(BITBAND_PERI(&PG->CR,6))))
#define PG_CR_PG7C            (*((__IO uint32_t *)(BITBAND_PERI(&PG->CR,7))))
#define PG_FR1_PG0F1          (*((__IO uint32_t *)(BITBAND_PERI(&PG->FR1,0))))
#define PG_FR1_PG1F1          (*((__IO uint32_t *)(BITBAND_PERI(&PG->FR1,1))))
#define PG_FR1_PG2F1          (*((__IO uint32_t *)(BITBAND_PERI(&PG->FR1,2))))
#define PG_FR1_PG3F1          (*((__IO uint32_t *)(BITBAND_PERI(&PG->FR1,3))))
#define PG_FR1_PG4F1          (*((__IO uint32_t *)(BITBAND_PERI(&PG->FR1,4))))
#define PG_FR1_PG5F1          (*((__IO uint32_t *)(BITBAND_PERI(&PG->FR1,5))))
#define PG_FR1_PG6F1          (*((__IO uint32_t *)(BITBAND_PERI(&PG->FR1,6))))
#define PG_FR1_PG7F1          (*((__IO uint32_t *)(BITBAND_PERI(&PG->FR1,7))))
#define PG_OD_PG0OD           (*((__IO uint32_t *)(BITBAND_PERI(&PG->OD,0))))
#define PG_OD_PG1OD           (*((__IO uint32_t *)(BITBAND_PERI(&PG->OD,1))))
#define PG_OD_PG2OD           (*((__IO uint32_t *)(BITBAND_PERI(&PG->OD,2))))
#define PG_OD_PG3OD           (*((__IO uint32_t *)(BITBAND_PERI(&PG->OD,3))))
#define PG_OD_PG4OD           (*((__IO uint32_t *)(BITBAND_PERI(&PG->OD,4))))
#define PG_OD_PG5OD           (*((__IO uint32_t *)(BITBAND_PERI(&PG->OD,5))))
#define PG_OD_PG6OD           (*((__IO uint32_t *)(BITBAND_PERI(&PG->OD,6))))
#define PG_OD_PG7OD           (*((__IO uint32_t *)(BITBAND_PERI(&PG->OD,7))))
#define PG_PUP_PG0UP          (*((__IO uint32_t *)(BITBAND_PERI(&PG->PUP,0))))
#define PG_PUP_PG1UP          (*((__IO uint32_t *)(BITBAND_PERI(&PG->PUP,1))))
#define PG_PUP_PG2UP          (*((__IO uint32_t *)(BITBAND_PERI(&PG->PUP,2))))
#define PG_PUP_PG3UP          (*((__IO uint32_t *)(BITBAND_PERI(&PG->PUP,3))))
#define PG_PUP_PG4UP          (*((__IO uint32_t *)(BITBAND_PERI(&PG->PUP,4))))
#define PG_PUP_PG5UP          (*((__IO uint32_t *)(BITBAND_PERI(&PG->PUP,5))))
#define PG_PUP_PG6UP          (*((__IO uint32_t *)(BITBAND_PERI(&PG->PUP,6))))
#define PG_PUP_PG7UP          (*((__IO uint32_t *)(BITBAND_PERI(&PG->PUP,7))))
#define PG_IE_PG0IE           (*((__IO uint32_t *)(BITBAND_PERI(&PG->IE,0))))
#define PG_IE_PG1IE           (*((__IO uint32_t *)(BITBAND_PERI(&PG->IE,1))))
#define PG_IE_PG2IE           (*((__IO uint32_t *)(BITBAND_PERI(&PG->IE,2))))
#define PG_IE_PG3IE           (*((__IO uint32_t *)(BITBAND_PERI(&PG->IE,3))))
#define PG_IE_PG4IE           (*((__IO uint32_t *)(BITBAND_PERI(&PG->IE,4))))
#define PG_IE_PG5IE           (*((__IO uint32_t *)(BITBAND_PERI(&PG->IE,5))))
#define PG_IE_PG6IE           (*((__IO uint32_t *)(BITBAND_PERI(&PG->IE,6))))
#define PG_IE_PG7IE           (*((__IO uint32_t *)(BITBAND_PERI(&PG->IE,7))))

/* General Purpose Input/Output Port (PH) */
#define PH_DATA_PH0           (*((__IO uint32_t *)(BITBAND_PERI(&PH->DATA,0))))
#define PH_DATA_PH1           (*((__IO uint32_t *)(BITBAND_PERI(&PH->DATA,1))))
#define PH_DATA_PH2           (*((__IO uint32_t *)(BITBAND_PERI(&PH->DATA,2))))
#define PH_DATA_PH3           (*((__IO uint32_t *)(BITBAND_PERI(&PH->DATA,3))))
#define PH_DATA_PH4           (*((__IO uint32_t *)(BITBAND_PERI(&PH->DATA,4))))
#define PH_DATA_PH5           (*((__IO uint32_t *)(BITBAND_PERI(&PH->DATA,5))))
#define PH_DATA_PH6           (*((__IO uint32_t *)(BITBAND_PERI(&PH->DATA,6))))
#define PH_DATA_PH7           (*((__IO uint32_t *)(BITBAND_PERI(&PH->DATA,7))))
#define PH_CR_PH0C            (*((__IO uint32_t *)(BITBAND_PERI(&PH->CR,0))))
#define PH_CR_PH1C            (*((__IO uint32_t *)(BITBAND_PERI(&PH->CR,1))))
#define PH_CR_PH2C            (*((__IO uint32_t *)(BITBAND_PERI(&PH->CR,2))))
#define PH_CR_PH3C            (*((__IO uint32_t *)(BITBAND_PERI(&PH->CR,3))))
#define PH_CR_PH4C            (*((__IO uint32_t *)(BITBAND_PERI(&PH->CR,4))))
#define PH_CR_PH5C            (*((__IO uint32_t *)(BITBAND_PERI(&PH->CR,5))))
#define PH_CR_PH6C            (*((__IO uint32_t *)(BITBAND_PERI(&PH->CR,6))))
#define PH_CR_PH7C            (*((__IO uint32_t *)(BITBAND_PERI(&PH->CR,7))))
#define PH_FR1_PH0F1          (*((__IO uint32_t *)(BITBAND_PERI(&PH->FR1,0))))
#define PH_FR1_PH1F1          (*((__IO uint32_t *)(BITBAND_PERI(&PH->FR1,1))))
#define PH_FR1_PH2F1          (*((__IO uint32_t *)(BITBAND_PERI(&PH->FR1,2))))
#define PH_FR1_PH3F1          (*((__IO uint32_t *)(BITBAND_PERI(&PH->FR1,3))))
#define PH_FR1_PH4F1          (*((__IO uint32_t *)(BITBAND_PERI(&PH->FR1,4))))
#define PH_FR1_PH5F1          (*((__IO uint32_t *)(BITBAND_PERI(&PH->FR1,5))))
#define PH_FR1_PH6F1          (*((__IO uint32_t *)(BITBAND_PERI(&PH->FR1,6))))
#define PH_FR1_PH7F1          (*((__IO uint32_t *)(BITBAND_PERI(&PH->FR1,7))))
#define PH_PUP_PH0UP          (*((__IO uint32_t *)(BITBAND_PERI(&PH->PUP,0))))
#define PH_PUP_PH1UP          (*((__IO uint32_t *)(BITBAND_PERI(&PH->PUP,1))))
#define PH_PUP_PH2UP          (*((__IO uint32_t *)(BITBAND_PERI(&PH->PUP,2))))
#define PH_PUP_PH3UP          (*((__IO uint32_t *)(BITBAND_PERI(&PH->PUP,3))))
#define PH_PUP_PH4UP          (*((__IO uint32_t *)(BITBAND_PERI(&PH->PUP,4))))
#define PH_PUP_PH5UP          (*((__IO uint32_t *)(BITBAND_PERI(&PH->PUP,5))))
#define PH_PUP_PH6UP          (*((__IO uint32_t *)(BITBAND_PERI(&PH->PUP,6))))
#define PH_PUP_PH7UP          (*((__IO uint32_t *)(BITBAND_PERI(&PH->PUP,7))))
#define PH_IE_PH0IE           (*((__IO uint32_t *)(BITBAND_PERI(&PH->IE,0))))
#define PH_IE_PH1IE           (*((__IO uint32_t *)(BITBAND_PERI(&PH->IE,1))))
#define PH_IE_PH2IE           (*((__IO uint32_t *)(BITBAND_PERI(&PH->IE,2))))
#define PH_IE_PH3IE           (*((__IO uint32_t *)(BITBAND_PERI(&PH->IE,3))))
#define PH_IE_PH4IE           (*((__IO uint32_t *)(BITBAND_PERI(&PH->IE,4))))
#define PH_IE_PH5IE           (*((__IO uint32_t *)(BITBAND_PERI(&PH->IE,5))))
#define PH_IE_PH6IE           (*((__IO uint32_t *)(BITBAND_PERI(&PH->IE,6))))
#define PH_IE_PH7IE           (*((__IO uint32_t *)(BITBAND_PERI(&PH->IE,7))))

/* General Purpose Input/Output Port (PI) */
#define PI_DATA_PI0           (*((__IO uint32_t *)(BITBAND_PERI(&PI->DATA,0))))
#define PI_DATA_PI1           (*((__IO uint32_t *)(BITBAND_PERI(&PI->DATA,1))))
#define PI_DATA_PI2           (*((__IO uint32_t *)(BITBAND_PERI(&PI->DATA,2))))
#define PI_DATA_PI3           (*((__IO uint32_t *)(BITBAND_PERI(&PI->DATA,3))))
#define PI_DATA_PI4           (*((__IO uint32_t *)(BITBAND_PERI(&PI->DATA,4))))
#define PI_DATA_PI5           (*((__IO uint32_t *)(BITBAND_PERI(&PI->DATA,5))))
#define PI_DATA_PI6           (*((__IO uint32_t *)(BITBAND_PERI(&PI->DATA,6))))
#define PI_DATA_PI7           (*((__IO uint32_t *)(BITBAND_PERI(&PI->DATA,7))))
#define PI_CR_PI0C            (*((__IO uint32_t *)(BITBAND_PERI(&PI->CR,0))))
#define PI_CR_PI1C            (*((__IO uint32_t *)(BITBAND_PERI(&PI->CR,1))))
#define PI_CR_PI2C            (*((__IO uint32_t *)(BITBAND_PERI(&PI->CR,2))))
#define PI_CR_PI3C            (*((__IO uint32_t *)(BITBAND_PERI(&PI->CR,3))))
#define PI_CR_PI4C            (*((__IO uint32_t *)(BITBAND_PERI(&PI->CR,4))))
#define PI_CR_PI5C            (*((__IO uint32_t *)(BITBAND_PERI(&PI->CR,5))))
#define PI_CR_PI6C            (*((__IO uint32_t *)(BITBAND_PERI(&PI->CR,6))))
#define PI_CR_PI7C            (*((__IO uint32_t *)(BITBAND_PERI(&PI->CR,7))))
#define PI_FR1_PI0F1          (*((__IO uint32_t *)(BITBAND_PERI(&PI->FR1,0))))
#define PI_FR1_PI1F1          (*((__IO uint32_t *)(BITBAND_PERI(&PI->FR1,1))))
#define PI_FR1_PI2F1          (*((__IO uint32_t *)(BITBAND_PERI(&PI->FR1,2))))
#define PI_FR1_PI3F1          (*((__IO uint32_t *)(BITBAND_PERI(&PI->FR1,3))))
#define PI_FR1_PI4F1          (*((__IO uint32_t *)(BITBAND_PERI(&PI->FR1,4))))
#define PI_FR1_PI5F1          (*((__IO uint32_t *)(BITBAND_PERI(&PI->FR1,5))))
#define PI_FR1_PI6F1          (*((__IO uint32_t *)(BITBAND_PERI(&PI->FR1,6))))
#define PI_FR1_PI7F1          (*((__IO uint32_t *)(BITBAND_PERI(&PI->FR1,7))))
#define PI_PUP_PI0UP          (*((__IO uint32_t *)(BITBAND_PERI(&PI->PUP,0))))
#define PI_PUP_PI1UP          (*((__IO uint32_t *)(BITBAND_PERI(&PI->PUP,1))))
#define PI_PUP_PI2UP          (*((__IO uint32_t *)(BITBAND_PERI(&PI->PUP,2))))
#define PI_PUP_PI3UP          (*((__IO uint32_t *)(BITBAND_PERI(&PI->PUP,3))))
#define PI_PUP_PI4UP          (*((__IO uint32_t *)(BITBAND_PERI(&PI->PUP,4))))
#define PI_PUP_PI5UP          (*((__IO uint32_t *)(BITBAND_PERI(&PI->PUP,5))))
#define PI_PUP_PI6UP          (*((__IO uint32_t *)(BITBAND_PERI(&PI->PUP,6))))
#define PI_PUP_PI7UP          (*((__IO uint32_t *)(BITBAND_PERI(&PI->PUP,7))))
#define PI_IE_PI0IE           (*((__IO uint32_t *)(BITBAND_PERI(&PI->IE,0))))
#define PI_IE_PI1IE           (*((__IO uint32_t *)(BITBAND_PERI(&PI->IE,1))))
#define PI_IE_PI2IE           (*((__IO uint32_t *)(BITBAND_PERI(&PI->IE,2))))
#define PI_IE_PI3IE           (*((__IO uint32_t *)(BITBAND_PERI(&PI->IE,3))))
#define PI_IE_PI4IE           (*((__IO uint32_t *)(BITBAND_PERI(&PI->IE,4))))
#define PI_IE_PI5IE           (*((__IO uint32_t *)(BITBAND_PERI(&PI->IE,5))))
#define PI_IE_PI6IE           (*((__IO uint32_t *)(BITBAND_PERI(&PI->IE,6))))
#define PI_IE_PI7IE           (*((__IO uint32_t *)(BITBAND_PERI(&PI->IE,7))))

/* General Purpose Input/Output Port (PJ) */
#define PJ_DATA_PJ0           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->DATA,0))))
#define PJ_DATA_PJ1           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->DATA,1))))
#define PJ_DATA_PJ2           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->DATA,2))))
#define PJ_DATA_PJ3           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->DATA,3))))
#define PJ_DATA_PJ4           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->DATA,4))))
#define PJ_DATA_PJ5           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->DATA,5))))
#define PJ_DATA_PJ6           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->DATA,6))))
#define PJ_DATA_PJ7           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->DATA,7))))
#define PJ_CR_PJ0C            (*((__IO uint32_t *)(BITBAND_PERI(&PJ->CR,0))))
#define PJ_CR_PJ1C            (*((__IO uint32_t *)(BITBAND_PERI(&PJ->CR,1))))
#define PJ_CR_PJ2C            (*((__IO uint32_t *)(BITBAND_PERI(&PJ->CR,2))))
#define PJ_CR_PJ3C            (*((__IO uint32_t *)(BITBAND_PERI(&PJ->CR,3))))
#define PJ_CR_PJ4C            (*((__IO uint32_t *)(BITBAND_PERI(&PJ->CR,4))))
#define PJ_CR_PJ5C            (*((__IO uint32_t *)(BITBAND_PERI(&PJ->CR,5))))
#define PJ_CR_PJ6C            (*((__IO uint32_t *)(BITBAND_PERI(&PJ->CR,6))))
#define PJ_CR_PJ7C            (*((__IO uint32_t *)(BITBAND_PERI(&PJ->CR,7))))
#define PJ_FR1_PJ0F1          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->FR1,0))))
#define PJ_FR1_PJ1F1          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->FR1,1))))
#define PJ_FR1_PJ2F1          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->FR1,2))))
#define PJ_FR1_PJ3F1          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->FR1,3))))
#define PJ_FR1_PJ4F1          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->FR1,4))))
#define PJ_FR1_PJ5F1          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->FR1,5))))
#define PJ_FR1_PJ6F1          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->FR1,6))))
#define PJ_FR1_PJ7F1          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->FR1,7))))
#define PJ_PUP_PJ0UP          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->PUP,0))))
#define PJ_PUP_PJ1UP          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->PUP,1))))
#define PJ_PUP_PJ2UP          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->PUP,2))))
#define PJ_PUP_PJ3UP          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->PUP,3))))
#define PJ_PUP_PJ4UP          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->PUP,4))))
#define PJ_PUP_PJ5UP          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->PUP,5))))
#define PJ_PUP_PJ6UP          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->PUP,6))))
#define PJ_PUP_PJ7UP          (*((__IO uint32_t *)(BITBAND_PERI(&PJ->PUP,7))))
#define PJ_IE_PJ0IE           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->IE,0))))
#define PJ_IE_PJ1IE           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->IE,1))))
#define PJ_IE_PJ2IE           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->IE,2))))
#define PJ_IE_PJ3IE           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->IE,3))))
#define PJ_IE_PJ4IE           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->IE,4))))
#define PJ_IE_PJ5IE           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->IE,5))))
#define PJ_IE_PJ6IE           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->IE,6))))
#define PJ_IE_PJ7IE           (*((__IO uint32_t *)(BITBAND_PERI(&PJ->IE,7))))

/* General Purpose Input/Output Port (PK) */
#define PK_DATA_PK0           (*((__IO uint32_t *)(BITBAND_PERI(&PK->DATA,0))))
#define PK_DATA_PK1           (*((__IO uint32_t *)(BITBAND_PERI(&PK->DATA,1))))
#define PK_DATA_PK2           (*((__IO uint32_t *)(BITBAND_PERI(&PK->DATA,2))))
#define PK_CR_PK0C            (*((__IO uint32_t *)(BITBAND_PERI(&PK->CR,0))))
#define PK_CR_PK1C            (*((__IO uint32_t *)(BITBAND_PERI(&PK->CR,1))))
#define PK_CR_PK2C            (*((__IO uint32_t *)(BITBAND_PERI(&PK->CR,2))))
#define PK_FR1_PK0F1          (*((__IO uint32_t *)(BITBAND_PERI(&PK->FR1,0))))
#define PK_FR1_PK1F1          (*((__IO uint32_t *)(BITBAND_PERI(&PK->FR1,1))))
#define PK_FR1_PK2F1          (*((__IO uint32_t *)(BITBAND_PERI(&PK->FR1,2))))
#define PK_FR2_PK1F2          (*((__IO uint32_t *)(BITBAND_PERI(&PK->FR2,1))))
#define PK_PUP_PK1UP          (*((__IO uint32_t *)(BITBAND_PERI(&PK->PUP,1))))
#define PK_PUP_PK2UP          (*((__IO uint32_t *)(BITBAND_PERI(&PK->PUP,2))))
#define PK_IE_PK0IE           (*((__IO uint32_t *)(BITBAND_PERI(&PK->IE,0))))
#define PK_IE_PK1IE           (*((__IO uint32_t *)(BITBAND_PERI(&PK->IE,1))))
#define PK_IE_PK2IE           (*((__IO uint32_t *)(BITBAND_PERI(&PK->IE,2))))

/* 16-bit Timer/Event Counter (TB) */
#define TB0_EN_TB0EN          (*((__IO uint32_t *)(BITBAND_PERI(&TB0->EN,7))))
#define TB0_RUN_TB0RUN        (*((__IO uint32_t *)(BITBAND_PERI(&TB0->RUN,0))))
#define TB0_RUN_TB0PRUN       (*((__IO uint32_t *)(BITBAND_PERI(&TB0->RUN,2))))
#define TB0_CR_I2TB0          (*((__IO uint32_t *)(BITBAND_PERI(&TB0->CR,3))))
#define TB0_CR_TB0SYNC        (*((__IO uint32_t *)(BITBAND_PERI(&TB0->CR,5))))
#define TB0_CR_TB0WBF         (*((__IO uint32_t *)(BITBAND_PERI(&TB0->CR,7))))
#define TB0_MOD_TB0CLE        (*((__IO uint32_t *)(BITBAND_PERI(&TB0->MOD,2))))
#define TB0_MOD_TB0CP0        (*((__O uint32_t *)(BITBAND_PERI(&TB0->MOD,5))))
#define TB0_FFCR_TB0E0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB0->FFCR,2))))
#define TB0_FFCR_TB0E1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB0->FFCR,3))))
#define TB0_FFCR_TB0C0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB0->FFCR,4))))
#define TB0_FFCR_TB0C1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB0->FFCR,5))))
#define TB0_ST_INTTB00        (*((__I uint32_t *)(BITBAND_PERI(&TB0->ST,0))))
#define TB0_ST_INTTB01        (*((__I uint32_t *)(BITBAND_PERI(&TB0->ST,1))))
#define TB0_ST_INTTBOF0       (*((__I uint32_t *)(BITBAND_PERI(&TB0->ST,2))))
#define TB0_IM_TBIM00         (*((__I uint32_t *)(BITBAND_PERI(&TB0->IM,0))))
#define TB0_IM_TBIM01         (*((__I uint32_t *)(BITBAND_PERI(&TB0->IM,1))))
#define TB0_IM_TBIMOF0        (*((__I uint32_t *)(BITBAND_PERI(&TB0->IM,2))))

#define TB1_EN_TB1EN          (*((__IO uint32_t *)(BITBAND_PERI(&TB1->EN,7))))
#define TB1_RUN_TB1RUN        (*((__IO uint32_t *)(BITBAND_PERI(&TB1->RUN,0))))
#define TB1_RUN_TB1PRUN       (*((__IO uint32_t *)(BITBAND_PERI(&TB1->RUN,2))))
#define TB1_CR_I2TB1          (*((__IO uint32_t *)(BITBAND_PERI(&TB1->CR,3))))
#define TB1_CR_TB1SYNC        (*((__IO uint32_t *)(BITBAND_PERI(&TB1->CR,5))))
#define TB1_CR_TB1WBF         (*((__IO uint32_t *)(BITBAND_PERI(&TB1->CR,7))))
#define TB1_MOD_TB1CLE        (*((__IO uint32_t *)(BITBAND_PERI(&TB1->MOD,2))))
#define TB1_MOD_TB1CP0        (*((__O uint32_t *)(BITBAND_PERI(&TB1->MOD,5))))
#define TB1_FFCR_TB1E0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB1->FFCR,2))))
#define TB1_FFCR_TB1E1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB1->FFCR,3))))
#define TB1_FFCR_TB1C0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB1->FFCR,4))))
#define TB1_FFCR_TB1C1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB1->FFCR,5))))
#define TB1_ST_INTTB10        (*((__I uint32_t *)(BITBAND_PERI(&TB1->ST,0))))
#define TB1_ST_INTTB11        (*((__I uint32_t *)(BITBAND_PERI(&TB1->ST,1))))
#define TB1_ST_INTTBOF1       (*((__I uint32_t *)(BITBAND_PERI(&TB1->ST,2))))
#define TB1_IM_TBIM10         (*((__I uint32_t *)(BITBAND_PERI(&TB1->IM,0))))
#define TB1_IM_TBIM11         (*((__I uint32_t *)(BITBAND_PERI(&TB1->IM,1))))
#define TB1_IM_TBIMOF1        (*((__I uint32_t *)(BITBAND_PERI(&TB1->IM,2))))

#define TB2_EN_TB2EN          (*((__IO uint32_t *)(BITBAND_PERI(&TB2->EN,7))))
#define TB2_RUN_TB2RUN        (*((__IO uint32_t *)(BITBAND_PERI(&TB2->RUN,0))))
#define TB2_RUN_TB2PRUN       (*((__IO uint32_t *)(BITBAND_PERI(&TB2->RUN,2))))
#define TB2_CR_I2TB2          (*((__IO uint32_t *)(BITBAND_PERI(&TB2->CR,3))))
#define TB2_CR_TB2SYNC        (*((__IO uint32_t *)(BITBAND_PERI(&TB2->CR,5))))
#define TB2_CR_TB2WBF         (*((__IO uint32_t *)(BITBAND_PERI(&TB2->CR,7))))
#define TB2_MOD_TB2CLE        (*((__IO uint32_t *)(BITBAND_PERI(&TB2->MOD,2))))
#define TB2_MOD_TB2CP0        (*((__O uint32_t *)(BITBAND_PERI(&TB2->MOD,5))))
#define TB2_FFCR_TB2E0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB2->FFCR,2))))
#define TB2_FFCR_TB2E1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB2->FFCR,3))))
#define TB2_FFCR_TB2C0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB2->FFCR,4))))
#define TB2_FFCR_TB2C1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB2->FFCR,5))))
#define TB2_ST_INTTB20        (*((__I uint32_t *)(BITBAND_PERI(&TB2->ST,0))))
#define TB2_ST_INTTB21        (*((__I uint32_t *)(BITBAND_PERI(&TB2->ST,1))))
#define TB2_ST_INTTBOF2       (*((__I uint32_t *)(BITBAND_PERI(&TB2->ST,2))))
#define TB2_IM_TBIM20         (*((__I uint32_t *)(BITBAND_PERI(&TB2->IM,0))))
#define TB2_IM_TBIM21         (*((__I uint32_t *)(BITBAND_PERI(&TB2->IM,1))))
#define TB2_IM_TBIMOF2        (*((__I uint32_t *)(BITBAND_PERI(&TB2->IM,2))))

#define TB3_EN_TB3EN          (*((__IO uint32_t *)(BITBAND_PERI(&TB3->EN,7))))
#define TB3_RUN_TB3RUN        (*((__IO uint32_t *)(BITBAND_PERI(&TB3->RUN,0))))
#define TB3_RUN_TB3PRUN       (*((__IO uint32_t *)(BITBAND_PERI(&TB3->RUN,2))))
#define TB3_CR_I2TB3          (*((__IO uint32_t *)(BITBAND_PERI(&TB3->CR,3))))
#define TB3_CR_TB3SYNC        (*((__IO uint32_t *)(BITBAND_PERI(&TB3->CR,5))))
#define TB3_CR_TB3WBF         (*((__IO uint32_t *)(BITBAND_PERI(&TB3->CR,7))))
#define TB3_MOD_TB3CLE        (*((__IO uint32_t *)(BITBAND_PERI(&TB3->MOD,2))))
#define TB3_MOD_TB3CP0        (*((__O uint32_t *)(BITBAND_PERI(&TB3->MOD,5))))
#define TB3_FFCR_TB3E0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB3->FFCR,2))))
#define TB3_FFCR_TB3E1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB3->FFCR,3))))
#define TB3_FFCR_TB3C0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB3->FFCR,4))))
#define TB3_FFCR_TB3C1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB3->FFCR,5))))
#define TB3_ST_INTTB30        (*((__I uint32_t *)(BITBAND_PERI(&TB3->ST,0))))
#define TB3_ST_INTTB31        (*((__I uint32_t *)(BITBAND_PERI(&TB3->ST,1))))
#define TB3_ST_INTTBOF3       (*((__I uint32_t *)(BITBAND_PERI(&TB3->ST,2))))
#define TB3_IM_TBIM30         (*((__I uint32_t *)(BITBAND_PERI(&TB3->IM,0))))
#define TB3_IM_TBIM31         (*((__I uint32_t *)(BITBAND_PERI(&TB3->IM,1))))
#define TB3_IM_TBIMOF3        (*((__I uint32_t *)(BITBAND_PERI(&TB3->IM,2))))

#define TB4_EN_TB4EN          (*((__IO uint32_t *)(BITBAND_PERI(&TB4->EN,7))))
#define TB4_RUN_TB4RUN        (*((__IO uint32_t *)(BITBAND_PERI(&TB4->RUN,0))))
#define TB4_RUN_TB4PRUN       (*((__IO uint32_t *)(BITBAND_PERI(&TB4->RUN,2))))
#define TB4_CR_I2TB4          (*((__IO uint32_t *)(BITBAND_PERI(&TB4->CR,3))))
#define TB4_CR_TB4SYNC        (*((__IO uint32_t *)(BITBAND_PERI(&TB4->CR,5))))
#define TB4_CR_TB4WBF         (*((__IO uint32_t *)(BITBAND_PERI(&TB4->CR,7))))
#define TB4_MOD_TB4CLE        (*((__IO uint32_t *)(BITBAND_PERI(&TB4->MOD,2))))
#define TB4_MOD_TB4CP0        (*((__O uint32_t *)(BITBAND_PERI(&TB4->MOD,5))))
#define TB4_FFCR_TB4E0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB4->FFCR,2))))
#define TB4_FFCR_TB4E1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB4->FFCR,3))))
#define TB4_FFCR_TB4C0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB4->FFCR,4))))
#define TB4_FFCR_TB4C1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB4->FFCR,5))))
#define TB4_ST_INTTB40        (*((__I uint32_t *)(BITBAND_PERI(&TB4->ST,0))))
#define TB4_ST_INTTB41        (*((__I uint32_t *)(BITBAND_PERI(&TB4->ST,1))))
#define TB4_ST_INTTBOF4       (*((__I uint32_t *)(BITBAND_PERI(&TB4->ST,2))))
#define TB4_IM_TBIM40         (*((__I uint32_t *)(BITBAND_PERI(&TB4->IM,0))))
#define TB4_IM_TBIM41         (*((__I uint32_t *)(BITBAND_PERI(&TB4->IM,1))))
#define TB4_IM_TBIMOF4        (*((__I uint32_t *)(BITBAND_PERI(&TB4->IM,2))))

#define TB5_EN_TB5EN          (*((__IO uint32_t *)(BITBAND_PERI(&TB5->EN,7))))
#define TB5_RUN_TB5RUN        (*((__IO uint32_t *)(BITBAND_PERI(&TB5->RUN,0))))
#define TB5_RUN_TB5PRUN       (*((__IO uint32_t *)(BITBAND_PERI(&TB5->RUN,2))))
#define TB5_CR_I2TB5          (*((__IO uint32_t *)(BITBAND_PERI(&TB5->CR,3))))
#define TB5_CR_TB5SYNC        (*((__IO uint32_t *)(BITBAND_PERI(&TB5->CR,5))))
#define TB5_CR_TB5WBF         (*((__IO uint32_t *)(BITBAND_PERI(&TB5->CR,7))))
#define TB5_MOD_TB5CLE        (*((__IO uint32_t *)(BITBAND_PERI(&TB5->MOD,2))))
#define TB5_MOD_TB5CP0        (*((__O uint32_t *)(BITBAND_PERI(&TB5->MOD,5))))
#define TB5_FFCR_TB5E0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB5->FFCR,2))))
#define TB5_FFCR_TB5E1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB5->FFCR,3))))
#define TB5_FFCR_TB5C0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB5->FFCR,4))))
#define TB5_FFCR_TB5C1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB5->FFCR,5))))
#define TB5_ST_INTTB50        (*((__I uint32_t *)(BITBAND_PERI(&TB5->ST,0))))
#define TB5_ST_INTTB51        (*((__I uint32_t *)(BITBAND_PERI(&TB5->ST,1))))
#define TB5_ST_INTTBOF5       (*((__I uint32_t *)(BITBAND_PERI(&TB5->ST,2))))
#define TB5_IM_TBIM50         (*((__I uint32_t *)(BITBAND_PERI(&TB5->IM,0))))
#define TB5_IM_TBIM51         (*((__I uint32_t *)(BITBAND_PERI(&TB5->IM,1))))
#define TB5_IM_TBIMOF5        (*((__I uint32_t *)(BITBAND_PERI(&TB5->IM,2))))

#define TB6_EN_TB6EN          (*((__IO uint32_t *)(BITBAND_PERI(&TB6->EN,7))))
#define TB6_RUN_TB6RUN        (*((__IO uint32_t *)(BITBAND_PERI(&TB6->RUN,0))))
#define TB6_RUN_TB6PRUN       (*((__IO uint32_t *)(BITBAND_PERI(&TB6->RUN,2))))
#define TB6_CR_I2TB6          (*((__IO uint32_t *)(BITBAND_PERI(&TB6->CR,3))))
#define TB6_CR_TB6SYNC        (*((__IO uint32_t *)(BITBAND_PERI(&TB6->CR,5))))
#define TB6_CR_TB6WBF         (*((__IO uint32_t *)(BITBAND_PERI(&TB6->CR,7))))
#define TB6_MOD_TB6CLE        (*((__IO uint32_t *)(BITBAND_PERI(&TB6->MOD,2))))
#define TB6_MOD_TB6CP0        (*((__O uint32_t *)(BITBAND_PERI(&TB6->MOD,5))))
#define TB6_FFCR_TB6E0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB6->FFCR,2))))
#define TB6_FFCR_TB6E1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB6->FFCR,3))))
#define TB6_FFCR_TB6C0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB6->FFCR,4))))
#define TB6_FFCR_TB6C1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB6->FFCR,5))))
#define TB6_ST_INTTB60        (*((__I uint32_t *)(BITBAND_PERI(&TB6->ST,0))))
#define TB6_ST_INTTB61        (*((__I uint32_t *)(BITBAND_PERI(&TB6->ST,1))))
#define TB6_ST_INTTBOF6       (*((__I uint32_t *)(BITBAND_PERI(&TB6->ST,2))))
#define TB6_IM_TBIM60         (*((__I uint32_t *)(BITBAND_PERI(&TB6->IM,0))))
#define TB6_IM_TBIM61         (*((__I uint32_t *)(BITBAND_PERI(&TB6->IM,1))))
#define TB6_IM_TBIMOF6        (*((__I uint32_t *)(BITBAND_PERI(&TB6->IM,2))))

#define TB7_EN_TB7EN          (*((__IO uint32_t *)(BITBAND_PERI(&TB7->EN,7))))
#define TB7_RUN_TB7RUN        (*((__IO uint32_t *)(BITBAND_PERI(&TB7->RUN,0))))
#define TB7_RUN_TB7PRUN       (*((__IO uint32_t *)(BITBAND_PERI(&TB7->RUN,2))))
#define TB7_CR_I2TB7          (*((__IO uint32_t *)(BITBAND_PERI(&TB7->CR,3))))
#define TB7_CR_TB7SYNC        (*((__IO uint32_t *)(BITBAND_PERI(&TB7->CR,5))))
#define TB7_CR_TB7WBF         (*((__IO uint32_t *)(BITBAND_PERI(&TB7->CR,7))))
#define TB7_MOD_TB7CLE        (*((__IO uint32_t *)(BITBAND_PERI(&TB7->MOD,2))))
#define TB7_MOD_TB7CP0        (*((__O uint32_t *)(BITBAND_PERI(&TB7->MOD,5))))
#define TB7_FFCR_TB7E0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB7->FFCR,2))))
#define TB7_FFCR_TB7E1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB7->FFCR,3))))
#define TB7_FFCR_TB7C0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB7->FFCR,4))))
#define TB7_FFCR_TB7C1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB7->FFCR,5))))
#define TB7_ST_INTTB70        (*((__I uint32_t *)(BITBAND_PERI(&TB7->ST,0))))
#define TB7_ST_INTTB71        (*((__I uint32_t *)(BITBAND_PERI(&TB7->ST,1))))
#define TB7_ST_INTTBOF7       (*((__I uint32_t *)(BITBAND_PERI(&TB7->ST,2))))
#define TB7_IM_TBIM70         (*((__I uint32_t *)(BITBAND_PERI(&TB7->IM,0))))
#define TB7_IM_TBIM71         (*((__I uint32_t *)(BITBAND_PERI(&TB7->IM,1))))
#define TB7_IM_TBIMOF7        (*((__I uint32_t *)(BITBAND_PERI(&TB7->IM,2))))

#define TB8_EN_TB8EN          (*((__IO uint32_t *)(BITBAND_PERI(&TB8->EN,7))))
#define TB8_RUN_TB8RUN        (*((__IO uint32_t *)(BITBAND_PERI(&TB8->RUN,0))))
#define TB8_RUN_TB8PRUN       (*((__IO uint32_t *)(BITBAND_PERI(&TB8->RUN,2))))
#define TB8_CR_I2TB8          (*((__IO uint32_t *)(BITBAND_PERI(&TB8->CR,3))))
#define TB8_CR_TB8SYNC        (*((__IO uint32_t *)(BITBAND_PERI(&TB8->CR,5))))
#define TB8_CR_TB8WBF         (*((__IO uint32_t *)(BITBAND_PERI(&TB8->CR,7))))
#define TB8_MOD_TB8CLE        (*((__IO uint32_t *)(BITBAND_PERI(&TB8->MOD,2))))
#define TB8_MOD_TB8CP0        (*((__O uint32_t *)(BITBAND_PERI(&TB8->MOD,5))))
#define TB8_FFCR_TB8E0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB8->FFCR,2))))
#define TB8_FFCR_TB8E1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB8->FFCR,3))))
#define TB8_FFCR_TB8C0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB8->FFCR,4))))
#define TB8_FFCR_TB8C1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB8->FFCR,5))))
#define TB8_ST_INTTB80        (*((__I uint32_t *)(BITBAND_PERI(&TB8->ST,0))))
#define TB8_ST_INTTB81        (*((__I uint32_t *)(BITBAND_PERI(&TB8->ST,1))))
#define TB8_ST_INTTBOF8       (*((__I uint32_t *)(BITBAND_PERI(&TB8->ST,2))))
#define TB8_IM_TBIM80         (*((__I uint32_t *)(BITBAND_PERI(&TB8->IM,0))))
#define TB8_IM_TBIM81         (*((__I uint32_t *)(BITBAND_PERI(&TB8->IM,1))))
#define TB8_IM_TBIMOF8        (*((__I uint32_t *)(BITBAND_PERI(&TB8->IM,2))))

#define TB9_EN_TB9EN          (*((__IO uint32_t *)(BITBAND_PERI(&TB9->EN,7))))
#define TB9_RUN_TB9RUN        (*((__IO uint32_t *)(BITBAND_PERI(&TB9->RUN,0))))
#define TB9_RUN_TB9PRUN       (*((__IO uint32_t *)(BITBAND_PERI(&TB9->RUN,2))))
#define TB9_CR_I2TB9          (*((__IO uint32_t *)(BITBAND_PERI(&TB9->CR,3))))
#define TB9_CR_TB9SYNC        (*((__IO uint32_t *)(BITBAND_PERI(&TB9->CR,5))))
#define TB9_CR_TB9WBF         (*((__IO uint32_t *)(BITBAND_PERI(&TB9->CR,7))))
#define TB9_MOD_TB9CLE        (*((__IO uint32_t *)(BITBAND_PERI(&TB9->MOD,2))))
#define TB9_MOD_TB9CP0        (*((__O uint32_t *)(BITBAND_PERI(&TB9->MOD,5))))
#define TB9_FFCR_TB9E0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB9->FFCR,2))))
#define TB9_FFCR_TB9E1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB9->FFCR,3))))
#define TB9_FFCR_TB9C0T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB9->FFCR,4))))
#define TB9_FFCR_TB9C1T1      (*((__IO uint32_t *)(BITBAND_PERI(&TB9->FFCR,5))))
#define TB9_ST_INTTB90        (*((__I uint32_t *)(BITBAND_PERI(&TB9->ST,0))))
#define TB9_ST_INTTB91        (*((__I uint32_t *)(BITBAND_PERI(&TB9->ST,1))))
#define TB9_ST_INTTBOF9       (*((__I uint32_t *)(BITBAND_PERI(&TB9->ST,2))))
#define TB9_IM_TBIM90         (*((__I uint32_t *)(BITBAND_PERI(&TB9->IM,0))))
#define TB9_IM_TBIM91         (*((__I uint32_t *)(BITBAND_PERI(&TB9->IM,1))))
#define TB9_IM_TBIMOF9        (*((__I uint32_t *)(BITBAND_PERI(&TB9->IM,2))))

/* Serial Bus Interface (SBI) */
#define SBI0_CR0_SBIEN        (*((__IO uint32_t *)(BITBAND_PERI(&SBI0->CR0,7))))
#define SBI0_CR1_SWRMON       (*((__I uint32_t *)(BITBAND_PERI(&SBI0->CR1,0))))
#define SBI0_CR1_ACK          (*((__IO uint32_t *)(BITBAND_PERI(&SBI0->CR1,4))))
#define SBI0_I2CAR_ALS        (*((__IO uint32_t *)(BITBAND_PERI(&SBI0->I2CAR,0))))
#define SBI0_CR2_PIN          (*((__O uint32_t *)(BITBAND_PERI(&SBI0->CR2,4))))
#define SBI0_CR2_BB           (*((__O uint32_t *)(BITBAND_PERI(&SBI0->CR2,5))))
#define SBI0_CR2_TRX          (*((__O uint32_t *)(BITBAND_PERI(&SBI0->CR2,6))))
#define SBI0_CR2_MST          (*((__O uint32_t *)(BITBAND_PERI(&SBI0->CR2,7))))
#define SBI0_SR_LRB           (*((__I uint32_t *)(BITBAND_PERI(&SBI0->SR,0))))
#define SBI0_SR_ADO           (*((__I uint32_t *)(BITBAND_PERI(&SBI0->SR,1))))
#define SBI0_SR_AAS           (*((__I uint32_t *)(BITBAND_PERI(&SBI0->SR,2))))
#define SBI0_SR_AL            (*((__I uint32_t *)(BITBAND_PERI(&SBI0->SR,3))))
#define SBI0_SR_PIN           (*((__I uint32_t *)(BITBAND_PERI(&SBI0->SR,4))))
#define SBI0_SR_BB            (*((__I uint32_t *)(BITBAND_PERI(&SBI0->SR,5))))
#define SBI0_SR_TRX           (*((__I uint32_t *)(BITBAND_PERI(&SBI0->SR,6))))
#define SBI0_SR_MST           (*((__I uint32_t *)(BITBAND_PERI(&SBI0->SR,7))))
#define SBI0_BR0_I2SBI0       (*((__IO uint32_t *)(BITBAND_PERI(&SBI0->BR0,6))))

#define SBI1_CR0_SBIEN        (*((__IO uint32_t *)(BITBAND_PERI(&SBI1->CR0,7))))
#define SBI1_CR1_SWRMON       (*((__I uint32_t *)(BITBAND_PERI(&SBI1->CR1,0))))
#define SBI1_CR1_ACK          (*((__IO uint32_t *)(BITBAND_PERI(&SBI1->CR1,4))))
#define SBI1_I2CAR_ALS        (*((__IO uint32_t *)(BITBAND_PERI(&SBI1->I2CAR,0))))
#define SBI1_CR2_PIN          (*((__O uint32_t *)(BITBAND_PERI(&SBI1->CR2,4))))
#define SBI1_CR2_BB           (*((__O uint32_t *)(BITBAND_PERI(&SBI1->CR2,5))))
#define SBI1_CR2_TRX          (*((__O uint32_t *)(BITBAND_PERI(&SBI1->CR2,6))))
#define SBI1_CR2_MST          (*((__O uint32_t *)(BITBAND_PERI(&SBI1->CR2,7))))
#define SBI1_SR_LRB           (*((__I uint32_t *)(BITBAND_PERI(&SBI1->SR,0))))
#define SBI1_SR_ADO           (*((__I uint32_t *)(BITBAND_PERI(&SBI1->SR,1))))
#define SBI1_SR_AAS           (*((__I uint32_t *)(BITBAND_PERI(&SBI1->SR,2))))
#define SBI1_SR_AL            (*((__I uint32_t *)(BITBAND_PERI(&SBI1->SR,3))))
#define SBI1_SR_PIN           (*((__I uint32_t *)(BITBAND_PERI(&SBI1->SR,4))))
#define SBI1_SR_BB            (*((__I uint32_t *)(BITBAND_PERI(&SBI1->SR,5))))
#define SBI1_SR_TRX           (*((__I uint32_t *)(BITBAND_PERI(&SBI1->SR,6))))
#define SBI1_SR_MST           (*((__I uint32_t *)(BITBAND_PERI(&SBI1->SR,7))))
#define SBI1_BR0_I2SBI0       (*((__IO uint32_t *)(BITBAND_PERI(&SBI1->BR0,6))))

#define SBI2_CR0_SBIEN        (*((__IO uint32_t *)(BITBAND_PERI(&SBI2->CR0,7))))
#define SBI2_CR1_SWRMON       (*((__I uint32_t *)(BITBAND_PERI(&SBI2->CR1,0))))
#define SBI2_CR1_ACK          (*((__IO uint32_t *)(BITBAND_PERI(&SBI2->CR1,4))))
#define SBI2_I2CAR_ALS        (*((__IO uint32_t *)(BITBAND_PERI(&SBI2->I2CAR,0))))
#define SBI2_CR2_PIN          (*((__O uint32_t *)(BITBAND_PERI(&SBI2->CR2,4))))
#define SBI2_CR2_BB           (*((__O uint32_t *)(BITBAND_PERI(&SBI2->CR2,5))))
#define SBI2_CR2_TRX          (*((__O uint32_t *)(BITBAND_PERI(&SBI2->CR2,6))))
#define SBI2_CR2_MST          (*((__O uint32_t *)(BITBAND_PERI(&SBI2->CR2,7))))
#define SBI2_SR_LRB           (*((__I uint32_t *)(BITBAND_PERI(&SBI2->SR,0))))
#define SBI2_SR_ADO           (*((__I uint32_t *)(BITBAND_PERI(&SBI2->SR,1))))
#define SBI2_SR_AAS           (*((__I uint32_t *)(BITBAND_PERI(&SBI2->SR,2))))
#define SBI2_SR_AL            (*((__I uint32_t *)(BITBAND_PERI(&SBI2->SR,3))))
#define SBI2_SR_PIN           (*((__I uint32_t *)(BITBAND_PERI(&SBI2->SR,4))))
#define SBI2_SR_BB            (*((__I uint32_t *)(BITBAND_PERI(&SBI2->SR,5))))
#define SBI2_SR_TRX           (*((__I uint32_t *)(BITBAND_PERI(&SBI2->SR,6))))
#define SBI2_SR_MST           (*((__I uint32_t *)(BITBAND_PERI(&SBI2->SR,7))))
#define SBI2_BR0_I2SBI0       (*((__IO uint32_t *)(BITBAND_PERI(&SBI2->BR0,6))))

/* Serial Channel (SC) */
#define SC0_EN_SIOE           (*((__IO uint32_t *)(BITBAND_PERI(&SC0->EN,0))))
#define SC0_CR_IOC            (*((__IO uint32_t *)(BITBAND_PERI(&SC0->CR,0))))
#define SC0_CR_SCLKS          (*((__IO uint32_t *)(BITBAND_PERI(&SC0->CR,1))))
#define SC0_CR_FERR           (*((__I uint32_t *)(BITBAND_PERI(&SC0->CR,2))))
#define SC0_CR_PERR           (*((__I uint32_t *)(BITBAND_PERI(&SC0->CR,3))))
#define SC0_CR_OERR           (*((__I uint32_t *)(BITBAND_PERI(&SC0->CR,4))))
#define SC0_CR_PE             (*((__IO uint32_t *)(BITBAND_PERI(&SC0->CR,5))))
#define SC0_CR_EVEN           (*((__IO uint32_t *)(BITBAND_PERI(&SC0->CR,6))))
#define SC0_CR_RB8            (*((__I uint32_t *)(BITBAND_PERI(&SC0->CR,7))))
#define SC0_MOD0_WU           (*((__IO uint32_t *)(BITBAND_PERI(&SC0->MOD0,4))))
#define SC0_MOD0_RXE          (*((__IO uint32_t *)(BITBAND_PERI(&SC0->MOD0,5))))
#define SC0_MOD0_CTSE         (*((__IO uint32_t *)(BITBAND_PERI(&SC0->MOD0,6))))
#define SC0_MOD0_TB8          (*((__IO uint32_t *)(BITBAND_PERI(&SC0->MOD0,7))))
#define SC0_BRCR_BR0ADDE      (*((__IO uint32_t *)(BITBAND_PERI(&SC0->BRCR,6))))
#define SC0_MOD1_TXE          (*((__IO uint32_t *)(BITBAND_PERI(&SC0->MOD1,4))))
#define SC0_MOD1_I2S0         (*((__IO uint32_t *)(BITBAND_PERI(&SC0->MOD1,7))))
#define SC0_MOD2_WBUF         (*((__IO uint32_t *)(BITBAND_PERI(&SC0->MOD2,2))))
#define SC0_MOD2_DRCHG        (*((__IO uint32_t *)(BITBAND_PERI(&SC0->MOD2,3))))
#define SC0_MOD2_SBLEN        (*((__IO uint32_t *)(BITBAND_PERI(&SC0->MOD2,4))))
#define SC0_MOD2_TXRUN        (*((__I uint32_t *)(BITBAND_PERI(&SC0->MOD2,5))))
#define SC0_MOD2_RBFLL        (*((__I uint32_t *)(BITBAND_PERI(&SC0->MOD2,6))))
#define SC0_MOD2_TBEMP        (*((__I uint32_t *)(BITBAND_PERI(&SC0->MOD2,7))))
#define SC0_RFC_RFIS          (*((__IO uint32_t *)(BITBAND_PERI(&SC0->RFC,6))))
#define SC0_RFC_RFCS          (*((__O uint32_t *)(BITBAND_PERI(&SC0->RFC,7))))
#define SC0_TFC_TFIS          (*((__IO uint32_t *)(BITBAND_PERI(&SC0->TFC,6))))
#define SC0_TFC_TFCS          (*((__O uint32_t *)(BITBAND_PERI(&SC0->TFC,7))))
#define SC0_RST_ROR           (*((__I uint32_t *)(BITBAND_PERI(&SC0->RST,7))))
#define SC0_TST_TUR           (*((__I uint32_t *)(BITBAND_PERI(&SC0->TST,7))))
#define SC0_FCNF_CNFG         (*((__IO uint32_t *)(BITBAND_PERI(&SC0->FCNF,0))))
#define SC0_FCNF_RXTXCNT      (*((__IO uint32_t *)(BITBAND_PERI(&SC0->FCNF,1))))
#define SC0_FCNF_RFIE         (*((__IO uint32_t *)(BITBAND_PERI(&SC0->FCNF,2))))
#define SC0_FCNF_TFIE         (*((__IO uint32_t *)(BITBAND_PERI(&SC0->FCNF,3))))
#define SC0_FCNF_RFST         (*((__IO uint32_t *)(BITBAND_PERI(&SC0->FCNF,4))))

#define SC1_EN_SIOE           (*((__IO uint32_t *)(BITBAND_PERI(&SC1->EN,0))))
#define SC1_CR_IOC            (*((__IO uint32_t *)(BITBAND_PERI(&SC1->CR,0))))
#define SC1_CR_SCLKS          (*((__IO uint32_t *)(BITBAND_PERI(&SC1->CR,1))))
#define SC1_CR_FERR           (*((__I uint32_t *)(BITBAND_PERI(&SC1->CR,2))))
#define SC1_CR_PERR           (*((__I uint32_t *)(BITBAND_PERI(&SC1->CR,3))))
#define SC1_CR_OERR           (*((__I uint32_t *)(BITBAND_PERI(&SC1->CR,4))))
#define SC1_CR_PE             (*((__IO uint32_t *)(BITBAND_PERI(&SC1->CR,5))))
#define SC1_CR_EVEN           (*((__IO uint32_t *)(BITBAND_PERI(&SC1->CR,6))))
#define SC1_CR_RB8            (*((__I uint32_t *)(BITBAND_PERI(&SC1->CR,7))))
#define SC1_MOD0_WU           (*((__IO uint32_t *)(BITBAND_PERI(&SC1->MOD0,4))))
#define SC1_MOD0_RXE          (*((__IO uint32_t *)(BITBAND_PERI(&SC1->MOD0,5))))
#define SC1_MOD0_CTSE         (*((__IO uint32_t *)(BITBAND_PERI(&SC1->MOD0,6))))
#define SC1_MOD0_TB8          (*((__IO uint32_t *)(BITBAND_PERI(&SC1->MOD0,7))))
#define SC1_BRCR_BR0ADDE      (*((__IO uint32_t *)(BITBAND_PERI(&SC1->BRCR,6))))
#define SC1_MOD1_TXE          (*((__IO uint32_t *)(BITBAND_PERI(&SC1->MOD1,4))))
#define SC1_MOD1_I2S0         (*((__IO uint32_t *)(BITBAND_PERI(&SC1->MOD1,7))))
#define SC1_MOD2_WBUF         (*((__IO uint32_t *)(BITBAND_PERI(&SC1->MOD2,2))))
#define SC1_MOD2_DRCHG        (*((__IO uint32_t *)(BITBAND_PERI(&SC1->MOD2,3))))
#define SC1_MOD2_SBLEN        (*((__IO uint32_t *)(BITBAND_PERI(&SC1->MOD2,4))))
#define SC1_MOD2_TXRUN        (*((__I uint32_t *)(BITBAND_PERI(&SC1->MOD2,5))))
#define SC1_MOD2_RBFLL        (*((__I uint32_t *)(BITBAND_PERI(&SC1->MOD2,6))))
#define SC1_MOD2_TBEMP        (*((__I uint32_t *)(BITBAND_PERI(&SC1->MOD2,7))))
#define SC1_RFC_RFIS          (*((__IO uint32_t *)(BITBAND_PERI(&SC1->RFC,6))))
#define SC1_RFC_RFCS          (*((__O uint32_t *)(BITBAND_PERI(&SC1->RFC,7))))
#define SC1_TFC_TFIS          (*((__IO uint32_t *)(BITBAND_PERI(&SC1->TFC,6))))
#define SC1_TFC_TFCS          (*((__O uint32_t *)(BITBAND_PERI(&SC1->TFC,7))))
#define SC1_RST_ROR           (*((__I uint32_t *)(BITBAND_PERI(&SC1->RST,7))))
#define SC1_TST_TUR           (*((__I uint32_t *)(BITBAND_PERI(&SC1->TST,7))))
#define SC1_FCNF_CNFG         (*((__IO uint32_t *)(BITBAND_PERI(&SC1->FCNF,0))))
#define SC1_FCNF_RXTXCNT      (*((__IO uint32_t *)(BITBAND_PERI(&SC1->FCNF,1))))
#define SC1_FCNF_RFIE         (*((__IO uint32_t *)(BITBAND_PERI(&SC1->FCNF,2))))
#define SC1_FCNF_TFIE         (*((__IO uint32_t *)(BITBAND_PERI(&SC1->FCNF,3))))
#define SC1_FCNF_RFST         (*((__IO uint32_t *)(BITBAND_PERI(&SC1->FCNF,4))))

#define SC2_EN_SIOE           (*((__IO uint32_t *)(BITBAND_PERI(&SC2->EN,0))))
#define SC2_CR_IOC            (*((__IO uint32_t *)(BITBAND_PERI(&SC2->CR,0))))
#define SC2_CR_SCLKS          (*((__IO uint32_t *)(BITBAND_PERI(&SC2->CR,1))))
#define SC2_CR_FERR           (*((__I uint32_t *)(BITBAND_PERI(&SC2->CR,2))))
#define SC2_CR_PERR           (*((__I uint32_t *)(BITBAND_PERI(&SC2->CR,3))))
#define SC2_CR_OERR           (*((__I uint32_t *)(BITBAND_PERI(&SC2->CR,4))))
#define SC2_CR_PE             (*((__IO uint32_t *)(BITBAND_PERI(&SC2->CR,5))))
#define SC2_CR_EVEN           (*((__IO uint32_t *)(BITBAND_PERI(&SC2->CR,6))))
#define SC2_CR_RB8            (*((__I uint32_t *)(BITBAND_PERI(&SC2->CR,7))))
#define SC2_MOD0_WU           (*((__IO uint32_t *)(BITBAND_PERI(&SC2->MOD0,4))))
#define SC2_MOD0_RXE          (*((__IO uint32_t *)(BITBAND_PERI(&SC2->MOD0,5))))
#define SC2_MOD0_CTSE         (*((__IO uint32_t *)(BITBAND_PERI(&SC2->MOD0,6))))
#define SC2_MOD0_TB8          (*((__IO uint32_t *)(BITBAND_PERI(&SC2->MOD0,7))))
#define SC2_BRCR_BR0ADDE      (*((__IO uint32_t *)(BITBAND_PERI(&SC2->BRCR,6))))
#define SC2_MOD1_TXE          (*((__IO uint32_t *)(BITBAND_PERI(&SC2->MOD1,4))))
#define SC2_MOD1_I2S0         (*((__IO uint32_t *)(BITBAND_PERI(&SC2->MOD1,7))))
#define SC2_MOD2_WBUF         (*((__IO uint32_t *)(BITBAND_PERI(&SC2->MOD2,2))))
#define SC2_MOD2_DRCHG        (*((__IO uint32_t *)(BITBAND_PERI(&SC2->MOD2,3))))
#define SC2_MOD2_SBLEN        (*((__IO uint32_t *)(BITBAND_PERI(&SC2->MOD2,4))))
#define SC2_MOD2_TXRUN        (*((__I uint32_t *)(BITBAND_PERI(&SC2->MOD2,5))))
#define SC2_MOD2_RBFLL        (*((__I uint32_t *)(BITBAND_PERI(&SC2->MOD2,6))))
#define SC2_MOD2_TBEMP        (*((__I uint32_t *)(BITBAND_PERI(&SC2->MOD2,7))))
#define SC2_RFC_RFIS          (*((__IO uint32_t *)(BITBAND_PERI(&SC2->RFC,6))))
#define SC2_RFC_RFCS          (*((__O uint32_t *)(BITBAND_PERI(&SC2->RFC,7))))
#define SC2_TFC_TFIS          (*((__IO uint32_t *)(BITBAND_PERI(&SC2->TFC,6))))
#define SC2_TFC_TFCS          (*((__O uint32_t *)(BITBAND_PERI(&SC2->TFC,7))))
#define SC2_RST_ROR           (*((__I uint32_t *)(BITBAND_PERI(&SC2->RST,7))))
#define SC2_TST_TUR           (*((__I uint32_t *)(BITBAND_PERI(&SC2->TST,7))))
#define SC2_FCNF_CNFG         (*((__IO uint32_t *)(BITBAND_PERI(&SC2->FCNF,0))))
#define SC2_FCNF_RXTXCNT      (*((__IO uint32_t *)(BITBAND_PERI(&SC2->FCNF,1))))
#define SC2_FCNF_RFIE         (*((__IO uint32_t *)(BITBAND_PERI(&SC2->FCNF,2))))
#define SC2_FCNF_TFIE         (*((__IO uint32_t *)(BITBAND_PERI(&SC2->FCNF,3))))
#define SC2_FCNF_RFST         (*((__IO uint32_t *)(BITBAND_PERI(&SC2->FCNF,4))))

/* Analog-to-Digital Converter (AD) */
#define AD_MOD0_ADS           (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD0,0))))
#define AD_MOD0_SCAN          (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD0,1))))
#define AD_MOD0_REPEAT        (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD0,2))))
#define AD_MOD0_ADBFN         (*((__I uint32_t *)(BITBAND_PERI(&AD->MOD0,6))))
#define AD_MOD0_EOCFN         (*((__I uint32_t *)(BITBAND_PERI(&AD->MOD0,7))))
#define AD_MOD1_ADSCN         (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD1,5))))
#define AD_MOD1_I2AD          (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD1,6))))
#define AD_MOD1_VREFON        (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD1,7))))
#define AD_MOD2_HPADCE        (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD2,5))))
#define AD_MOD2_ADBFHP        (*((__I uint32_t *)(BITBAND_PERI(&AD->MOD2,6))))
#define AD_MOD2_EOCFHP        (*((__I uint32_t *)(BITBAND_PERI(&AD->MOD2,7))))
#define AD_MOD3_ADOBSV        (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD3,0))))
#define AD_MOD3_ADOBIC        (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD3,5))))
#define AD_MOD4_ADHTG         (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD4,4))))
#define AD_MOD4_ADHS          (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD4,5))))
#define AD_MOD4_HADHTG        (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD4,6))))
#define AD_MOD4_HADHS         (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD4,7))))
#define AD_MOD5_ADOBSV        (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD5,0))))
#define AD_MOD5_ADOBIC        (*((__IO uint32_t *)(BITBAND_PERI(&AD->MOD5,5))))
#define AD_REG08_ADR0RF       (*((__I uint32_t *)(BITBAND_PERI(&AD->REG08,0))))
#define AD_REG08_OVR0         (*((__I uint32_t *)(BITBAND_PERI(&AD->REG08,1))))
#define AD_REG19_ADR1RF       (*((__I uint32_t *)(BITBAND_PERI(&AD->REG19,0))))
#define AD_REG19_OVR1         (*((__I uint32_t *)(BITBAND_PERI(&AD->REG19,1))))
#define AD_REG2A_ADR2RF       (*((__I uint32_t *)(BITBAND_PERI(&AD->REG2A,0))))
#define AD_REG2A_OVR2         (*((__I uint32_t *)(BITBAND_PERI(&AD->REG2A,1))))
#define AD_REG3B_ADR3RF       (*((__I uint32_t *)(BITBAND_PERI(&AD->REG3B,0))))
#define AD_REG3B_OVR3         (*((__I uint32_t *)(BITBAND_PERI(&AD->REG3B,1))))
#define AD_REG4C_ADR4RF       (*((__I uint32_t *)(BITBAND_PERI(&AD->REG4C,0))))
#define AD_REG4C_OVR4         (*((__I uint32_t *)(BITBAND_PERI(&AD->REG4C,1))))
#define AD_REG5D_ADR5RF       (*((__I uint32_t *)(BITBAND_PERI(&AD->REG5D,0))))
#define AD_REG5D_OVR5         (*((__I uint32_t *)(BITBAND_PERI(&AD->REG5D,1))))
#define AD_REG6E_ADR6RF       (*((__I uint32_t *)(BITBAND_PERI(&AD->REG6E,0))))
#define AD_REG6E_OVR6         (*((__I uint32_t *)(BITBAND_PERI(&AD->REG6E,1))))
#define AD_REG7F_ADR7RF       (*((__I uint32_t *)(BITBAND_PERI(&AD->REG7F,0))))
#define AD_REG7F_OVR7         (*((__I uint32_t *)(BITBAND_PERI(&AD->REG7F,1))))
#define AD_REGSP_ADRSPRF      (*((__I uint32_t *)(BITBAND_PERI(&AD->REGSP,0))))
#define AD_REGSP_OVRSP        (*((__I uint32_t *)(BITBAND_PERI(&AD->REGSP,1))))

/* Watchdog Timer (WD) */
#define WD_MOD_RESCR          (*((__IO uint32_t *)(BITBAND_PERI(&WD->MOD,1))))
#define WD_MOD_I2WDT          (*((__IO uint32_t *)(BITBAND_PERI(&WD->MOD,2))))
#define WD_MOD_WDTE           (*((__IO uint32_t *)(BITBAND_PERI(&WD->MOD,7))))

/* Real Time Clock (RTC) */
#define RTC_MONTHR_MO0        (*((__IO uint32_t *)(BITBAND_PERI(&RTC->MONTHR,0))))
#define RTC_PAGER_PAGE        (*((__IO uint32_t *)(BITBAND_PERI(&RTC->PAGER,0))))
#define RTC_PAGER_ENAALM      (*((__IO uint32_t *)(BITBAND_PERI(&RTC->PAGER,2))))
#define RTC_PAGER_ENATMR      (*((__IO uint32_t *)(BITBAND_PERI(&RTC->PAGER,3))))
#define RTC_PAGER_ADJUST      (*((__IO uint32_t *)(BITBAND_PERI(&RTC->PAGER,4))))
#define RTC_PAGER_INTENA      (*((__IO uint32_t *)(BITBAND_PERI(&RTC->PAGER,7))))
#define RTC_RESTR_RSTALM      (*((__IO uint32_t *)(BITBAND_PERI(&RTC->RESTR,4))))
#define RTC_RESTR_RSTTMR      (*((__IO uint32_t *)(BITBAND_PERI(&RTC->RESTR,5))))
#define RTC_RESTR_DIS16HZ     (*((__IO uint32_t *)(BITBAND_PERI(&RTC->RESTR,6))))
#define RTC_RESTR_DIS1HZ      (*((__IO uint32_t *)(BITBAND_PERI(&RTC->RESTR,7))))

/* Clock Generator (CG) */
#define CG_SYSCR_FPSEL        (*((__IO uint32_t *)(BITBAND_PERI(&CG->SYSCR,12))))
#define CG_OSCCR_WUEON        (*((__O uint32_t *)(BITBAND_PERI(&CG->OSCCR,0))))
#define CG_OSCCR_WUEF         (*((__I uint32_t *)(BITBAND_PERI(&CG->OSCCR,1))))
#define CG_OSCCR_PLLON        (*((__IO uint32_t *)(BITBAND_PERI(&CG->OSCCR,2))))
#define CG_OSCCR_WUPSEL       (*((__IO uint32_t *)(BITBAND_PERI(&CG->OSCCR,3))))
#define CG_OSCCR_XEN          (*((__IO uint32_t *)(BITBAND_PERI(&CG->OSCCR,8))))
#define CG_OSCCR_XTEN         (*((__IO uint32_t *)(BITBAND_PERI(&CG->OSCCR,9))))
#define CG_STBYCR_RXEN        (*((__IO uint32_t *)(BITBAND_PERI(&CG->STBYCR,8))))
#define CG_STBYCR_RXTEN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->STBYCR,9))))
#define CG_STBYCR_DRVE        (*((__IO uint32_t *)(BITBAND_PERI(&CG->STBYCR,16))))
#define CG_PLLSEL_PLLSEL      (*((__IO uint32_t *)(BITBAND_PERI(&CG->PLLSEL,0))))
#define CG_CKSEL_SYSCKFLG     (*((__I uint32_t *)(BITBAND_PERI(&CG->CKSEL,0))))
#define CG_CKSEL_SYSCK        (*((__IO uint32_t *)(BITBAND_PERI(&CG->CKSEL,1))))
#define CG_RSTFLG_PONRSTF     (*((__IO uint32_t *)(BITBAND_PERI(&CG->RSTFLG,0))))
#define CG_RSTFLG_PINRSTF     (*((__IO uint32_t *)(BITBAND_PERI(&CG->RSTFLG,1))))
#define CG_RSTFLG_WDTRSTF     (*((__IO uint32_t *)(BITBAND_PERI(&CG->RSTFLG,2))))
#define CG_RSTFLG_SYSRSTF     (*((__IO uint32_t *)(BITBAND_PERI(&CG->RSTFLG,4))))
#define CG_IMCGA_INT0EN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGA,0))))
#define CG_IMCGA_INT1EN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGA,8))))
#define CG_IMCGA_INT2EN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGA,16))))
#define CG_IMCGA_INT3EN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGA,24))))
#define CG_IMCGB_INT4EN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGB,0))))
#define CG_IMCGB_INT5EN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGB,8))))
#define CG_IMCGB_INT6EN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGB,16))))
#define CG_IMCGB_INT7EN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGB,24))))
#define CG_IMCGC_INT8EN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGC,0))))
#define CG_IMCGC_INT9EN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGC,8))))
#define CG_IMCGC_INTAEN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGC,16))))
#define CG_IMCGC_INTBEN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGC,24))))
#define CG_IMCGD_INTCEN       (*((__IO uint32_t *)(BITBAND_PERI(&CG->IMCGD,0))))

/* Consumer Electronics Control (CEC) */
#define CEC_EN_CECEN          (*((__IO uint32_t *)(BITBAND_PERI(&CEC->EN,0))))
#define CEC_EN_I2CEC          (*((__IO uint32_t *)(BITBAND_PERI(&CEC->EN,1))))
#define CEC_RESET_CECRESET    (*((__O uint32_t *)(BITBAND_PERI(&CEC->RESET,0))))
#define CEC_REN_CECREN        (*((__IO uint32_t *)(BITBAND_PERI(&CEC->REN,0))))
#define CEC_RBUF_CECEOM       (*((__I uint32_t *)(BITBAND_PERI(&CEC->RBUF,8))))
#define CEC_RBUF_CECACK       (*((__I uint32_t *)(BITBAND_PERI(&CEC->RBUF,9))))
#define CEC_RCR1_CECOTH       (*((__IO uint32_t *)(BITBAND_PERI(&CEC->RCR1,0))))
#define CEC_RCR1_CECRIHLD     (*((__IO uint32_t *)(BITBAND_PERI(&CEC->RCR1,1))))
#define CEC_RCR1_CECACKDIS    (*((__IO uint32_t *)(BITBAND_PERI(&CEC->RCR1,24))))
#define CEC_RCR3_CECWAVEN     (*((__IO uint32_t *)(BITBAND_PERI(&CEC->RCR3,0))))
#define CEC_TEN_CECTEN        (*((__O uint32_t *)(BITBAND_PERI(&CEC->TEN,0))))
#define CEC_TEN_CECTRANS      (*((__I uint32_t *)(BITBAND_PERI(&CEC->TEN,1))))
#define CEC_TBUF_CECTEOM      (*((__IO uint32_t *)(BITBAND_PERI(&CEC->TBUF,0))))
#define CEC_TCR_CECBRD        (*((__IO uint32_t *)(BITBAND_PERI(&CEC->TCR,4))))
#define CEC_RSTAT_CECRIEND    (*((__I uint32_t *)(BITBAND_PERI(&CEC->RSTAT,0))))
#define CEC_RSTAT_CECRISTA    (*((__I uint32_t *)(BITBAND_PERI(&CEC->RSTAT,1))))
#define CEC_RSTAT_CECRIMAX    (*((__I uint32_t *)(BITBAND_PERI(&CEC->RSTAT,2))))
#define CEC_RSTAT_CECRIMIN    (*((__I uint32_t *)(BITBAND_PERI(&CEC->RSTAT,3))))
#define CEC_RSTAT_CECRIACK    (*((__I uint32_t *)(BITBAND_PERI(&CEC->RSTAT,4))))
#define CEC_RSTAT_CECRIOR     (*((__I uint32_t *)(BITBAND_PERI(&CEC->RSTAT,5))))
#define CEC_RSTAT_CECRIWAV    (*((__I uint32_t *)(BITBAND_PERI(&CEC->RSTAT,6))))
#define CEC_TSTAT_CECTISTA    (*((__I uint32_t *)(BITBAND_PERI(&CEC->TSTAT,0))))
#define CEC_TSTAT_CECTIEND    (*((__I uint32_t *)(BITBAND_PERI(&CEC->TSTAT,1))))
#define CEC_TSTAT_CECTIAL     (*((__I uint32_t *)(BITBAND_PERI(&CEC->TSTAT,2))))
#define CEC_TSTAT_CECTIACK    (*((__I uint32_t *)(BITBAND_PERI(&CEC->TSTAT,3))))
#define CEC_TSTAT_CECTIUR     (*((__I uint32_t *)(BITBAND_PERI(&CEC->TSTAT,4))))

/* Remote Control Signal Preprocessor (RMC) */
#define RMC0_EN_RMCEN         (*((__IO uint32_t *)(BITBAND_PERI(&RMC0->EN,0))))
#define RMC0_EN_I2RMC         (*((__IO uint32_t *)(BITBAND_PERI(&RMC0->EN,1))))
#define RMC0_REN_RMCREN       (*((__IO uint32_t *)(BITBAND_PERI(&RMC0->REN,0))))
#define RMC0_RCR2_RMCPHM      (*((__IO uint32_t *)(BITBAND_PERI(&RMC0->RCR2,24))))
#define RMC0_RCR2_RMCLD       (*((__IO uint32_t *)(BITBAND_PERI(&RMC0->RCR2,25))))
#define RMC0_RCR2_RMCEDIEN    (*((__IO uint32_t *)(BITBAND_PERI(&RMC0->RCR2,30))))
#define RMC0_RCR2_RMCLIEN     (*((__IO uint32_t *)(BITBAND_PERI(&RMC0->RCR2,31))))
#define RMC0_RCR4_RMCPO       (*((__IO uint32_t *)(BITBAND_PERI(&RMC0->RCR4,7))))
#define RMC0_RSTAT_RMCRLDR    (*((__I uint32_t *)(BITBAND_PERI(&RMC0->RSTAT,7))))
#define RMC0_RSTAT_RMCEDIF    (*((__I uint32_t *)(BITBAND_PERI(&RMC0->RSTAT,12))))
#define RMC0_RSTAT_RMCDMAXIF  (*((__I uint32_t *)(BITBAND_PERI(&RMC0->RSTAT,13))))
#define RMC0_RSTAT_RMCLOIF    (*((__I uint32_t *)(BITBAND_PERI(&RMC0->RSTAT,14))))
#define RMC0_RSTAT_RMCRLIF    (*((__I uint32_t *)(BITBAND_PERI(&RMC0->RSTAT,15))))

#define RMC1_EN_RMCEN         (*((__IO uint32_t *)(BITBAND_PERI(&RMC1->EN,0))))
#define RMC1_EN_I2RMC         (*((__IO uint32_t *)(BITBAND_PERI(&RMC1->EN,1))))
#define RMC1_REN_RMCREN       (*((__IO uint32_t *)(BITBAND_PERI(&RMC1->REN,0))))
#define RMC1_RCR2_RMCPHM      (*((__IO uint32_t *)(BITBAND_PERI(&RMC1->RCR2,24))))
#define RMC1_RCR2_RMCLD       (*((__IO uint32_t *)(BITBAND_PERI(&RMC1->RCR2,25))))
#define RMC1_RCR2_RMCEDIEN    (*((__IO uint32_t *)(BITBAND_PERI(&RMC1->RCR2,30))))
#define RMC1_RCR2_RMCLIEN     (*((__IO uint32_t *)(BITBAND_PERI(&RMC1->RCR2,31))))
#define RMC1_RCR4_RMCPO       (*((__IO uint32_t *)(BITBAND_PERI(&RMC1->RCR4,7))))
#define RMC1_RSTAT_RMCRLDR    (*((__I uint32_t *)(BITBAND_PERI(&RMC1->RSTAT,7))))
#define RMC1_RSTAT_RMCEDIF    (*((__I uint32_t *)(BITBAND_PERI(&RMC1->RSTAT,12))))
#define RMC1_RSTAT_RMCDMAXIF  (*((__I uint32_t *)(BITBAND_PERI(&RMC1->RSTAT,13))))
#define RMC1_RSTAT_RMCLOIF    (*((__I uint32_t *)(BITBAND_PERI(&RMC1->RSTAT,14))))
#define RMC1_RSTAT_RMCRLIF    (*((__I uint32_t *)(BITBAND_PERI(&RMC1->RSTAT,15))))

/* Flash Control (FC) */
#define FC_SECBIT_SECBIT      (*((__IO uint32_t *)(BITBAND_PERI(&FC->SECBIT,0))))
#define FC_FLCS_RDY_BSY       (*((__I uint32_t *)(BITBAND_PERI(&FC->FLCS,0))))
#define FC_FLCS_BLPRO0        (*((__I uint32_t *)(BITBAND_PERI(&FC->FLCS,16))))
#define FC_FLCS_BLPRO1        (*((__I uint32_t *)(BITBAND_PERI(&FC->FLCS,17))))
#define FC_FLCS_BLPRO2        (*((__I uint32_t *)(BITBAND_PERI(&FC->FLCS,18))))
#define FC_FLCS_BLPRO3        (*((__I uint32_t *)(BITBAND_PERI(&FC->FLCS,19))))
#define FC_FLCS_BLPRO4        (*((__I uint32_t *)(BITBAND_PERI(&FC->FLCS,20))))
#define FC_FLCS_BLPRO5        (*((__I uint32_t *)(BITBAND_PERI(&FC->FLCS,21))))

/** @} */ /* End of group Device_Peripheral_registers */

#ifdef __cplusplus
}
#endif

#endif  /* __TMPM330_H__ */

/** @} */ /* End of group TMPM330 */
/** @} */ /* End of group TOSHIBA_TX03_MICROCONTROLLER */
