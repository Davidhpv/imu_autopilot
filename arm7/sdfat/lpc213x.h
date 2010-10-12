#ifndef __LPC213X_H
#define __LPC213X_H

#include "LPC21xx.h"

#endif
////-----------------------------------------------------------------------------
//
//// Vectored Interrupt Controller (VIC)
//#define VICIRQStatus   (*((volatile U32 *) 0xFFFFF000))
//#define VICFIQStatus   (*((volatile U32 *) 0xFFFFF004))
//#define VICRawIntr     (*((volatile U32 *) 0xFFFFF008))
//#define VICIntSelect   (*((volatile U32 *) 0xFFFFF00C))
//#define VICIntEnable   (*((volatile U32 *) 0xFFFFF010))
//#define VICIntEnClear  (*((volatile U32 *) 0xFFFFF014))
//#define VICSoftInt     (*((volatile U32 *) 0xFFFFF018))
//#define VICSoftIntClr  (*((volatile U32 *) 0xFFFFF01C))
//#define VICProtection  (*((volatile U32 *) 0xFFFFF020))
//#define VICVectAddr    (*((volatile U32 *) 0xFFFFF030))
//#define VICDefVectAddr (*((volatile U32 *) 0xFFFFF034))
//#define VICVectAddr0   (*((volatile U32 *) 0xFFFFF100))
//#define VICVectAddr1   (*((volatile U32 *) 0xFFFFF104))
//#define VICVectAddr2   (*((volatile U32 *) 0xFFFFF108))
//#define VICVectAddr3   (*((volatile U32 *) 0xFFFFF10C))
//#define VICVectAddr4   (*((volatile U32 *) 0xFFFFF110))
//#define VICVectAddr5   (*((volatile U32 *) 0xFFFFF114))
//#define VICVectAddr6   (*((volatile U32 *) 0xFFFFF118))
//#define VICVectAddr7   (*((volatile U32 *) 0xFFFFF11C))
//#define VICVectAddr8   (*((volatile U32 *) 0xFFFFF120))
//#define VICVectAddr9   (*((volatile U32 *) 0xFFFFF124))
//#define VICVectAddr10  (*((volatile U32 *) 0xFFFFF128))
//#define VICVectAddr11  (*((volatile U32 *) 0xFFFFF12C))
//#define VICVectAddr12  (*((volatile U32 *) 0xFFFFF130))
//#define VICVectAddr13  (*((volatile U32 *) 0xFFFFF134))
//#define VICVectAddr14  (*((volatile U32 *) 0xFFFFF138))
//#define VICVectAddr15  (*((volatile U32 *) 0xFFFFF13C))
//#define VICVectCntl0   (*((volatile U32 *) 0xFFFFF200))
//#define VICVectCntl1   (*((volatile U32 *) 0xFFFFF204))
//#define VICVectCntl2   (*((volatile U32 *) 0xFFFFF208))
//#define VICVectCntl3   (*((volatile U32 *) 0xFFFFF20C))
//#define VICVectCntl4   (*((volatile U32 *) 0xFFFFF210))
//#define VICVectCntl5   (*((volatile U32 *) 0xFFFFF214))
//#define VICVectCntl6   (*((volatile U32 *) 0xFFFFF218))
//#define VICVectCntl7   (*((volatile U32 *) 0xFFFFF21C))
//#define VICVectCntl8   (*((volatile U32 *) 0xFFFFF220))
//#define VICVectCntl9   (*((volatile U32 *) 0xFFFFF224))
//#define VICVectCntl10  (*((volatile U32 *) 0xFFFFF228))
//#define VICVectCntl11  (*((volatile U32 *) 0xFFFFF22C))
//#define VICVectCntl12  (*((volatile U32 *) 0xFFFFF230))
//#define VICVectCntl13  (*((volatile U32 *) 0xFFFFF234))
//#define VICVectCntl14  (*((volatile U32 *) 0xFFFFF238))
//#define VICVectCntl15  (*((volatile U32 *) 0xFFFFF23C))
//
//// Pin Connect Block
//#define PINSEL0        (*((volatile U32 *) 0xE002C000))
//#define PINSEL1        (*((volatile U32 *) 0xE002C004))
//#define PINSEL2        (*((volatile U32 *) 0xE002C014))
//
//// General Purpose Input/Output (GPIO)
//#define IOPIN0         (*((volatile U32 *) 0xE0028000))
//#define IOSET0         (*((volatile U32 *) 0xE0028004))
//#define IODIR0         (*((volatile U32 *) 0xE0028008))
//#define IOCLR0         (*((volatile U32 *) 0xE002800C))
//#define IOPIN1         (*((volatile U32 *) 0xE0028010))
//#define IOSET1         (*((volatile U32 *) 0xE0028014))
//#define IODIR1         (*((volatile U32 *) 0xE0028018))
//#define IOCLR1         (*((volatile U32 *) 0xE002801C))
//
//// Memory Accelerator Module (MAM)
//#define MAMCR          (*((volatile U08 *) 0xE01FC000))
//#define MAMTIM         (*((volatile U08 *) 0xE01FC004))
//#define MEMMAP         (*((volatile U08 *) 0xE01FC040))
//
//// Phase Locked Loop (PLL)
//#define PLLCON         (*((volatile U08 *) 0xE01FC080))
//#define PLLCFG         (*((volatile U08 *) 0xE01FC084))
//#define PLLSTAT        (*((volatile U16*) 0xE01FC088))
//#define PLLFEED        (*((volatile U08 *) 0xE01FC08C))
//
//// VPB Divider
//#define VPBDIV         (*((volatile U08 *) 0xE01FC100))
//
//// Power Control
//#define PCON           (*((volatile U08 *) 0xE01FC0C0))
//#define PCONP          (*((volatile U32 *) 0xE01FC0C4))
//
//// External Interrupts
//#define EXTINT         (*((volatile U08 *) 0xE01FC140))
//#define INTWAKE        (*((volatile U08 *) 0xE01FC144))
//#define EXTMODE        (*((volatile U08 *) 0xE01FC148))
//#define EXTPOLAR       (*((volatile U08 *) 0xE01FC14C))
//
//// Reset
//#define RSID           (*((volatile U08 *) 0xE01FC180))
//
//// Code Security / Debugging
//#define CSPR           (*((volatile U08 *) 0xE01FC184))
//
//// Timer 0
//#define T0IR           (*((volatile U32 *) 0xE0004000))
//#define T0TCR          (*((volatile U32 *) 0xE0004004))
//#define T0TC           (*((volatile U32 *) 0xE0004008))
//#define T0PR           (*((volatile U32 *) 0xE000400C))
//#define T0PC           (*((volatile U32 *) 0xE0004010))
//#define T0MCR          (*((volatile U32 *) 0xE0004014))
//#define T0MR0          (*((volatile U32 *) 0xE0004018))
//#define T0MR1          (*((volatile U32 *) 0xE000401C))
//#define T0MR2          (*((volatile U32 *) 0xE0004020))
//#define T0MR3          (*((volatile U32 *) 0xE0004024))
//#define T0CCR          (*((volatile U32 *) 0xE0004028))
//#define T0CR0          (*((volatile U32 *) 0xE000402C))
//#define T0CR1          (*((volatile U32 *) 0xE0004030))
//#define T0CR2          (*((volatile U32 *) 0xE0004034))
//#define T0CR3          (*((volatile U32 *) 0xE0004038))
//#define T0EMR          (*((volatile U32 *) 0xE000403C))
//#define T0CTCR         (*((volatile U32 *) 0xE0004070))
//
//// Timer 1
//#define T1IR           (*((volatile U32 *) 0xE0008000))
//#define T1TCR          (*((volatile U32 *) 0xE0008004))
//#define T1TC           (*((volatile U32 *) 0xE0008008))
//#define T1PR           (*((volatile U32 *) 0xE000800C))
//#define T1PC           (*((volatile U32 *) 0xE0008010))
//#define T1MCR          (*((volatile U32 *) 0xE0008014))
//#define T1MR0          (*((volatile U32 *) 0xE0008018))
//#define T1MR1          (*((volatile U32 *) 0xE000801C))
//#define T1MR2          (*((volatile U32 *) 0xE0008020))
//#define T1MR3          (*((volatile U32 *) 0xE0008024))
//#define T1CCR          (*((volatile U32 *) 0xE0008028))
//#define T1CR0          (*((volatile U32 *) 0xE000802C))
//#define T1CR1          (*((volatile U32 *) 0xE0008030))
//#define T1CR2          (*((volatile U32 *) 0xE0008034))
//#define T1CR3          (*((volatile U32 *) 0xE0008038))
//#define T1EMR          (*((volatile U32 *) 0xE000803C))
//#define T1CTCR         (*((volatile U32 *) 0xE0008070))
//
//// Pulse Width Modulator (PWM)
//#define PWMIR          (*((volatile U32 *) 0xE0014000))
//#define PWMTCR         (*((volatile U32 *) 0xE0014004))
//#define PWMTC          (*((volatile U32 *) 0xE0014008))
//#define PWMPR          (*((volatile U32 *) 0xE001400C))
//#define PWMPC          (*((volatile U32 *) 0xE0014010))
//#define PWMMCR         (*((volatile U32 *) 0xE0014014))
//#define PWMMR0         (*((volatile U32 *) 0xE0014018))
//#define PWMMR1         (*((volatile U32 *) 0xE001401C))
//#define PWMMR2         (*((volatile U32 *) 0xE0014020))
//#define PWMMR3         (*((volatile U32 *) 0xE0014024))
//#define PWMMR4         (*((volatile U32 *) 0xE0014040))
//#define PWMMR5         (*((volatile U32 *) 0xE0014044))
//#define PWMMR6         (*((volatile U32 *) 0xE0014048))
//#define PWMCCR         (*((volatile U32 *) 0xE0014028))
//#define PWMCR0         (*((volatile U32 *) 0xE001402C))
//#define PWMCR1         (*((volatile U32 *) 0xE0014030))
//#define PWMCR2         (*((volatile U32 *) 0xE0014034))
//#define PWMCR3         (*((volatile U32 *) 0xE0014038))
//#define PWMEMR         (*((volatile U32 *) 0xE001403C))
//#define PWMPCR         (*((volatile U32 *) 0xE001404C))
//#define PWMLER         (*((volatile U32 *) 0xE0014050))
//
//// Universal Asynchronous Receiver Transmitter 0 (UART0)
//#define UART0_RBR       (*((volatile U08 *) 0xE000C000))
//#define UART0_THR       (*((volatile U08 *) 0xE000C000))
//#define UART0_IER       (*((volatile U08 *) 0xE000C004))
//#define UART0_IIR       (*((volatile U08 *) 0xE000C008))
//#define UART0_FCR       (*((volatile U08 *) 0xE000C008))
//#define UART0_LCR       (*((volatile U08 *) 0xE000C00C))
//#define UART0_MCR       (*((volatile U08 *) 0xE000C010))
//#define UART0_LSR       (*((volatile U08 *) 0xE000C014))
//#define UART0_MSR       (*((volatile U08 *) 0xE000C018))
//#define UART0_SCR       (*((volatile U08 *) 0xE000C01C))
//#define UART0_DLL       (*((volatile U08 *) 0xE000C000))
//#define UART0_DLM       (*((volatile U08 *) 0xE000C004))
//
//// Universal Asynchronous Receiver Transmitter 1 (UART1)
//#define UART1_RBR       (*((volatile U08 *) 0xE0010000))
//#define UART1_THR       (*((volatile U08 *) 0xE0010000))
//#define UART1_IER       (*((volatile U08 *) 0xE0010004))
//#define UART1_IIR       (*((volatile U08 *) 0xE0010008))
//#define UART1_FCR       (*((volatile U08 *) 0xE0010008))
//#define UART1_LCR       (*((volatile U08 *) 0xE001000C))
//#define UART1_MCR       (*((volatile U08 *) 0xE0010010))
//#define UART1_LSR       (*((volatile U08 *) 0xE0010014))
//#define UART1_MSR       (*((volatile U08 *) 0xE0010018))
//#define UART1_SCR       (*((volatile U08 *) 0xE001001C))
//#define UART1_DLL       (*((volatile U08 *) 0xE0010000))
//#define UART1_DLM       (*((volatile U08 *) 0xE0010004))
//
//// I2C Interface 0
//#define I20CONSET      (*((volatile U08 *) 0xE001C000))
//#define I20STAT        (*((volatile U08 *) 0xE001C004))
//#define I20DAT         (*((volatile U08 *) 0xE001C008))
//#define I20ADR         (*((volatile U08 *) 0xE001C00C))
//#define I20SCLH        (*((volatile U16 *) 0xE001C010))
//#define I20SCLL        (*((volatile U16 *) 0xE001C014))
//#define I20CONCLR      (*((volatile U08 *) 0xE001C018))
//
//// I2C Interface 1
//#define I21CONSET      (*((volatile U08 *) 0xE005C000))
//#define I21STAT        (*((volatile U08 *) 0xE005C004))
//#define I21DAT         (*((volatile U08 *) 0xE005C008))
//#define I21ADR         (*((volatile U08 *) 0xE005C00C))
//#define I21SCLH        (*((volatile U16 *) 0xE005C010))
//#define I21SCLL        (*((volatile U16 *) 0xE005C014))
//#define I21CONCLR      (*((volatile U08 *) 0xE005C018))
//
//// SPI0 (Serial Peripheral Interface 0)
//#define S0SPCR         (*((volatile U08 *) 0xE0020000))
//#define S0SPSR         (*((volatile U08 *) 0xE0020004))
//#define S0SPDR         (*((volatile U08 *) 0xE0020008))
//#define S0SPCCR        (*((volatile U08 *) 0xE002000C))
//#define S0SPTCR        (*((volatile U08 *) 0xE0020010))
//#define S0SPTSR        (*((volatile U08 *) 0xE0020014))
//#define S0SPTOR        (*((volatile U08 *) 0xE0020018))
//#define S0SPINT        (*((volatile U08 *) 0xE002001C))
//
//// SSP Controller
//#define SSPCR0         (*((volatile U16 * ) 0xE0068000))
//#define SSPCR1         (*((volatile U08 * ) 0xE0068004))
//#define SSPDR          (*((volatile U16 * ) 0xE0068008))
//#define SSPSR          (*((volatile U08 * ) 0xE006800C))
//#define SSPCPSR        (*((volatile U08 * ) 0xE0068010))
//#define SSPIMSC        (*((volatile U08 * ) 0xE0068014))
//#define SSPRIS         (*((volatile U08 * ) 0xE0068018))
//#define SSPMIS         (*((volatile U08 * ) 0xE006801C))
//#define SSPICR         (*((volatile U08 * ) 0xE0068020))
////#define SSPDMACR       (*((volatile U08 * ) 0xE0068024)) // DMA wohl Zukunft
//
//// Real Time Clock
//#define ILR            (*((volatile U08 *) 0xE0024000))
//#define CTC            (*((volatile U16 *) 0xE0024004))
//#define CCR            (*((volatile U08 *) 0xE0024008))
//#define CIIR           (*((volatile U08 *) 0xE002400C))
//#define AMR            (*((volatile U08 *) 0xE0024010))
//#define CTIME0         (*((volatile U32 *) 0xE0024014))
//#define CTIME1         (*((volatile U32 *) 0xE0024018))
//#define CTIME2         (*((volatile U32 *) 0xE002401C))
//#define SEC            (*((volatile U08 *) 0xE0024020))
//#define MIN            (*((volatile U08 *) 0xE0024024))
//#define HOUR           (*((volatile U08 *) 0xE0024028))
//#define DOM            (*((volatile U08 *) 0xE002402C))
//#define DOW            (*((volatile U08 *) 0xE0024030))
//#define DOY            (*((volatile U16 *) 0xE0024034))
//#define MONTH          (*((volatile U08 *) 0xE0024038))
//#define YEAR           (*((volatile U16 *) 0xE002403C))
//#define ALSEC          (*((volatile U08 *) 0xE0024060))
//#define ALMIN          (*((volatile U08 *) 0xE0024064))
//#define ALHOUR         (*((volatile U08 *) 0xE0024068))
//#define ALDOM          (*((volatile U08 *) 0xE002406C))
//#define ALDOW          (*((volatile U08 *) 0xE0024070))
//#define ALDOY          (*((volatile U16 *) 0xE0024074))
//#define ALMON          (*((volatile U08 *) 0xE0024078))
//#define ALYEAR         (*((volatile U16 *) 0xE002407C))
//#define PREINT         (*((volatile U16 *) 0xE0024080))
//#define PREFRAC        (*((volatile U16 *) 0xE0024084))
//
//// A/D Converter 0 (AD0)
//#define AD0CR          (*((volatile U32 *) 0xE0034000))
//#define AD0DR          (*((volatile U32 *) 0xE0034004))
//
//// A/D Converter 1 (AD1)
//#define AD1CR          (*((volatile U32 *) 0xE0060000))
//#define AD1DR          (*((volatile U32 *) 0xE0060004))
//
//// D/A Converter
//#define DACR           (*((volatile U32 *) 0xE006C000))
//
//// Watchdog
//#define WDMOD          (*((volatile U08 *) 0xE0000000))
//#define WDTC           (*((volatile U32 *) 0xE0000004))
//#define WDFEED         (*((volatile U08 *) 0xE0000008))
//#define WDTV           (*((volatile U32 *) 0xE000000C))
////-----------------------------------------------------------------------------
//
//// Vectored Interrupt Channels
//#define VIC_WDT     0 // Watchdog
//#define VIC_SW      1 // Software interrupts
//#define VIC_DEBUGRX 2 // Embedded ICE, DbgCommRx
//#define VIC_DEBUGTX 3 // Embedded ICE, DbgCommTx
//#define VIC_TIMER0  4 // Timer 0 (Match 0-3 Capture 0-3)
//#define VIC_TIMER1  5 // Timer 1 (Match 0-3 Capture 0-3)
//#define VIC_UART0   6 // UART 0  (RLS, THRE, RDA, CTI)
//#define VIC_UART1   7 // UART 1  (RLS, THRE, RDA, CTI, MSI)
//#define VIC_PWM0    8 // PWM 0   (Match 0-6 Capture 0-3)
//#define VIC_I2C     9 // I2C 0   (SI)
//#define VIC_SPI    10 // SPI 0   (SPIF, MODF)
//#define VIC_SSP    11 // SSP
//#define VIC_PLL    12 // PLL lock (PLOCK)
//#define VIC_RTC    13 // RTC      (RTCCIF, RTCALF)
//#define VIC_EINT0  14 // External interrupt 0 (EINT0)
//#define VIC_EINT1  15 // External interrupt 1 (EINT1)
//#define VIC_EINT2  16 // External interrupt 2 (EINT2)
//#define VIC_EINT3  17 // External interrupt 3 (EINT3)
//#define VIC_AD0    18 // A/D converter 0
//#define VIC_I2C1   19 // I2C 1
//#define VIC_BOD    20 // Brown out detect
//#define VIC_AD1    21 // A/D converter 1
////-----------------------------------------------------------------------------
//
//#define PCTIM0 BIT1;
//#define PCTIM1 BIT2;
//#define PCURT0 BIT3;
//#define PCURT1 BIT4;
//#define PCPWM0 BIT5;
//#define PCI2C0 BIT7;
//#define PCSPI0 BIT8;
//#define PCRTC  BIT9;
//#define PCSPI1 BIT10;
//#define PCAD0  BIT12;
//#define PCI2C1 BIT19;
//#define PCAD1  BIT20;
////-----------------------------------------------------------------------------
//
//#endif

