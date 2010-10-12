;/*****************************************************************************
; * @file:    startup_SAM3U.s
; * @purpose: CMSIS Cortex-M3 Core Device Startup File 
; *           for the Toshiba 'TMPM330' Device Series 
; * @version: V1.02
; * @date:    31. July 2009
; *----------------------------------------------------------------------------
; *
; * Copyright (C) 2009 ARM Limited. All rights reserved.
; *
; * ARM Limited (ARM) is supplying this software for use with Cortex-Mx 
; * processor based microcontrollers.  This file can be freely distributed 
; * within development tools that are supporting such ARM based processors. 
; *
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; ******************************************************************************/


;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)
	
        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     MemManage_Handler
        DCD     BusFault_Handler
        DCD     UsageFault_Handler
__vector_table_0x1c
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     DebugMon_Handler
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD     SUPC_IRQHandler           ; 16: Supply Controller
        DCD     RSTC_IRQHandler           ; 17: Reset Controller
        DCD     RTC_IRQHandler            ; 18: Real Time Clock
        DCD     RTT_IRQHandler            ; 19: Real Time Timer
        DCD     WDT_IRQHandler            ; 20: Watchdog Timer
        DCD     PMC_IRQHandler            ; 21: Power Management Controller
        DCD     EEFC0_IRQHandler          ; 22: Enhanced Embedded Flash Controller 0
        DCD     EEFC1_IRQHandler          ; 23: Enhanced Embedded Flash Controller 1
        DCD     UART_IRQHandler           ; 24: UART
        DCD     SMC_IRQHandler            ; 25: Static Memory Controller
        DCD     PIOA_IRQHandler           ; 26: Parallel I/O Controller A
        DCD     PIOB_IRQHandler           ; 27: Parallel I/O Controller B
        DCD     PIOC_IRQHandler           ; 28: Parallel I/O Controller C
        DCD     USART0_IRQHandler         ; 29: USART 0
        DCD     USART1_IRQHandler         ; 30: USART 1
        DCD     USART2_IRQHandler         ; 31: USART 2
        DCD     USART3_IRQHandler         ; 32: USART 3
        DCD     HSMCI_IRQHandler          ; 33: High Speed Multimedia Card Interface
        DCD     TWI0_IRQHandler           ; 34: Two-wire Interface 0
        DCD     TWI1_IRQHandler           ; 35: Two-wire Interface 1
        DCD     SPI_IRQHandler            ; 36: Synchronous Peripheral Interface
        DCD     SSC_IRQHandler            ; 37: Synchronous Serial Controller
        DCD     TC0_IRQHandler            ; 38: Timer Counter 0
        DCD     TC1_IRQHandler            ; 39: Timer Counter 1
        DCD     TC2_IRQHandler            ; 40: Timer Counter 2
        DCD     PWM_IRQHandler            ; 41: Pulse Width Modulation Controller
        DCD     ADC12B_IRQHandler         ; 42: 12-bit ADC Controller
        DCD     ADC_IRQHandler            ; 43: 10-bit ADC Controller
        DCD     DMAC_IRQHandler           ; 44: DMA Controller
        DCD     UDPHS_IRQHandler          ; 45: USB Device High Speed
__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size 	EQU 	__Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER(2)
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER(1)
MemManage_Handler
        B MemManage_Handler

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER(1)
BusFault_Handler
        B BusFault_Handler

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER(1)
UsageFault_Handler
        B UsageFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER(1)
DebugMon_Handler
        B DebugMon_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER(1)
SysTick_Handler
        B SysTick_Handler

        PUBWEAK SUPC_IRQHandler
        SECTION .text:CODE:REORDER(1)
SUPC_IRQHandler
        B SUPC_IRQHandler

        PUBWEAK RSTC_IRQHandler
        SECTION .text:CODE:REORDER(1)
RSTC_IRQHandler
        B RSTC_IRQHandler

        PUBWEAK RTC_IRQHandler
        SECTION .text:CODE:REORDER(1)
RTC_IRQHandler
        B RTC_IRQHandler

        PUBWEAK RTT_IRQHandler
        SECTION .text:CODE:REORDER(1)
RTT_IRQHandler
        B RTT_IRQHandler

        PUBWEAK WDT_IRQHandler
        SECTION .text:CODE:REORDER(1)
WDT_IRQHandler
        B WDT_IRQHandler

        PUBWEAK PMC_IRQHandler
        SECTION .text:CODE:REORDER(1)
PMC_IRQHandler
        B PMC_IRQHandler

        PUBWEAK EEFC0_IRQHandler
        SECTION .text:CODE:REORDER(1)
EEFC0_IRQHandler
        B EEFC0_IRQHandler

        PUBWEAK EEFC1_IRQHandler
        SECTION .text:CODE:REORDER(1)
EEFC1_IRQHandler
        B EEFC1_IRQHandler

        PUBWEAK UART_IRQHandler
        SECTION .text:CODE:REORDER(1)
UART_IRQHandler
        B UART_IRQHandler

        PUBWEAK SMC_IRQHandler
        SECTION .text:CODE:REORDER(1)
SMC_IRQHandler
        B SMC_IRQHandler

        PUBWEAK PIOA_IRQHandler
        SECTION .text:CODE:REORDER(1)
PIOA_IRQHandler
        B PIOA_IRQHandler

        PUBWEAK PIOB_IRQHandler
        SECTION .text:CODE:REORDER(1)
PIOB_IRQHandler
        B PIOB_IRQHandler

        PUBWEAK PIOC_IRQHandler
        SECTION .text:CODE:REORDER(1)
PIOC_IRQHandler
        B PIOC_IRQHandler

        PUBWEAK USART0_IRQHandler
        SECTION .text:CODE:REORDER(1)
USART0_IRQHandler
        B USART0_IRQHandler

        PUBWEAK USART1_IRQHandler
        SECTION .text:CODE:REORDER(1)
USART1_IRQHandler
        B USART1_IRQHandler

        PUBWEAK USART2_IRQHandler
        SECTION .text:CODE:REORDER(1)
USART2_IRQHandler
        B USART2_IRQHandler

        PUBWEAK USART3_IRQHandler
        SECTION .text:CODE:REORDER(1)
USART3_IRQHandler
        B USART3_IRQHandler

        PUBWEAK HSMCI_IRQHandler
        SECTION .text:CODE:REORDER(1)
HSMCI_IRQHandler
        B HSMCI_IRQHandler

        PUBWEAK TWI0_IRQHandler
        SECTION .text:CODE:REORDER(1)
TWI0_IRQHandler
        B TWI0_IRQHandler

        PUBWEAK TWI1_IRQHandler
        SECTION .text:CODE:REORDER(1)
TWI1_IRQHandler
        B TWI1_IRQHandler

        PUBWEAK SPI_IRQHandler
        SECTION .text:CODE:REORDER(1)
SPI_IRQHandler
        B SPI_IRQHandler

        PUBWEAK SSC_IRQHandler
        SECTION .text:CODE:REORDER(1)
SSC_IRQHandler
        B SSC_IRQHandler

        PUBWEAK TC0_IRQHandler
        SECTION .text:CODE:REORDER(1)
TC0_IRQHandler
        B TC0_IRQHandler

        PUBWEAK TC1_IRQHandler
        SECTION .text:CODE:REORDER(1)
TC1_IRQHandler
        B TC1_IRQHandler

        PUBWEAK TC2_IRQHandler
        SECTION .text:CODE:REORDER(1)
TC2_IRQHandler
        B TC2_IRQHandler

        PUBWEAK PWM_IRQHandler
        SECTION .text:CODE:REORDER(1)
PWM_IRQHandler
        B PWM_IRQHandler

        PUBWEAK ADC12B_IRQHandler
        SECTION .text:CODE:REORDER(1)
ADC12B_IRQHandler
        B ADC12B_IRQHandler

        PUBWEAK ADC_IRQHandler
        SECTION .text:CODE:REORDER(1)
ADC_IRQHandler
        B ADC_IRQHandler

        PUBWEAK DMAC_IRQHandler
        SECTION .text:CODE:REORDER(1)
DMAC_IRQHandler
        B DMAC_IRQHandler

        PUBWEAK UDPHS_IRQHandler
        SECTION .text:CODE:REORDER(1)
UDPHS_IRQHandler
        B UDPHS_IRQHandler


        END
