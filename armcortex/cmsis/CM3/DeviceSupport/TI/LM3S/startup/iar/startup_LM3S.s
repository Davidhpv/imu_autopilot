;/*****************************************************************************
; * @file:    startup_LM3S.s
; * @purpose: CMSIS Cortex-M3 Core Device Startup File 
; *           for the Luminary LM3S Device Series 
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
        DCD     GPIOPortA_IRQHandler        ;  0: GPIO Port A
        DCD     GPIOPortB_IRQHandler        ;  1: GPIO Port B
        DCD     GPIOPortC_IRQHandler        ;  2: GPIO Port C
        DCD     GPIOPortD_IRQHandler        ;  3: GPIO Port D
        DCD     GPIOPortE_IRQHandler        ;  4: GPIO Port E
        DCD     UART0_IRQHandler            ;  5: UART0
        DCD     UART1_IRQHandler            ;  6: UART1
        DCD     SSI0_IRQHandler             ;  7: SSI0
        DCD     I2C0_IRQHandler             ;  8: I2C0
        DCD     PWMFault_IRQHandler         ;  9: PWM Fault
        DCD     PWMGen0_IRQHandler          ; 10: PWM Generator 0
        DCD     PWMGen1_IRQHandler          ; 11: PWM Generator 1
        DCD     PWMGen2_IRQHandler          ; 12: PWM Generator 2
        DCD     QEI0_IRQHandler             ; 13: Quadrature Encoder
        DCD     ADCSeq0_IRQHandler          ; 14: ADC Sequence 0
        DCD     ADCSeq1_IRQHandler          ; 15: ADC Sequence 1
        DCD     ADCSeq2_IRQHandler          ; 16: ADC Sequence 2
        DCD     ADCSeq3_IRQHandler          ; 17: ADC Sequence 3
        DCD     Watchdog_IRQHandler         ; 18: Watchdog
        DCD     Timer0A_IRQHandler          ; 19: Timer 0A
        DCD     Timer0B_IRQHandler          ; 20: Timer 0B
        DCD     Timer1A_IRQHandler          ; 21: Timer 1A
        DCD     Timer1B_IRQHandler          ; 22: Timer 1B
        DCD     Timer2A_IRQHandler          ; 23: Timer 2A
        DCD     Timer2B_IRQHandler          ; 24: Timer 2B
        DCD     Comp0_IRQHandler            ; 25: Comp 0
        DCD     Comp1_IRQHandler            ; 26: Comp 1
        DCD     Comp2_IRQHandler            ; 27: Comp 2
        DCD     SysCtrl_IRQHandler          ; 28: System Control
        DCD     FlashCtrl_IRQHandler        ; 29: Flash Control
        DCD     GPIOPortF_IRQHandler        ; 30: GPIO Port F
        DCD     GPIOPortG_IRQHandler        ; 31: GPIO Port G
        DCD     GPIOPortH_IRQHandler        ; 32: GPIO Port H
        DCD     USART2_IRQHandler           ; 33: UART2 Rx and Tx
        DCD     SSI1_IRQHandler             ; 34: SSI1 Rx and Tx
        DCD     Timer3A_IRQHandler          ; 35: Timer 3 subtimer A
        DCD     Timer3B_IRQHandler          ; 36: Timer 3 subtimer B
        DCD     I2C1_IRQHandler             ; 37: I2C1 Master and Slave
        DCD     QEI1_IRQHandler             ; 38: Quadrature Encoder 1
        DCD     CAN0_IRQHandler             ; 39: CAN0
        DCD     CAN1_IRQHandler             ; 40: CAN1
        DCD     CAN2_IRQHandler             ; 41: CAN2
        DCD     Ethernet_IRQHandler         ; 42: Ethernet
        DCD     Hibernate_IRQHandler        ; 43:Hibernate
        DCD     USB0_IRQHandler             ; 44: USB0
        DCD     PWMGen3_IRQHandler          ; 45: PWM Generator 3
        DCD     uDMA_IRQHandler             ; 46: uDMA Software Transfer
        DCD     uDMAErr_IRQHandler          ; 47: uDMA Error
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

        PUBWEAK GPIOPortA_IRQHandler
        SECTION .text:CODE:REORDER(1)
GPIOPortA_IRQHandler
        B GPIOPortA_IRQHandler

        PUBWEAK GPIOPortB_IRQHandler
        SECTION .text:CODE:REORDER(1)
GPIOPortB_IRQHandler
        B GPIOPortB_IRQHandler

        PUBWEAK GPIOPortC_IRQHandler
        SECTION .text:CODE:REORDER(1)
GPIOPortC_IRQHandler
        B GPIOPortC_IRQHandler

        PUBWEAK GPIOPortD_IRQHandler
        SECTION .text:CODE:REORDER(1)
GPIOPortD_IRQHandler
        B GPIOPortD_IRQHandler

        PUBWEAK GPIOPortE_IRQHandler
        SECTION .text:CODE:REORDER(1)
GPIOPortE_IRQHandler
        B GPIOPortE_IRQHandler

        PUBWEAK UART0_IRQHandler
        SECTION .text:CODE:REORDER(1)
UART0_IRQHandler
        B UART0_IRQHandler

        PUBWEAK UART1_IRQHandler
        SECTION .text:CODE:REORDER(1)
UART1_IRQHandler
        B UART1_IRQHandler

        PUBWEAK SSI0_IRQHandler
        SECTION .text:CODE:REORDER(1)
SSI0_IRQHandler
        B SSI0_IRQHandler

        PUBWEAK I2C0_IRQHandler
        SECTION .text:CODE:REORDER(1)
I2C0_IRQHandler
        B I2C0_IRQHandler

        PUBWEAK PWMFault_IRQHandler
        SECTION .text:CODE:REORDER(1)
PWMFault_IRQHandler
        B PWMFault_IRQHandler

        PUBWEAK PWMGen0_IRQHandler
        SECTION .text:CODE:REORDER(1)
PWMGen0_IRQHandler
        B PWMGen0_IRQHandler

        PUBWEAK PWMGen1_IRQHandler
        SECTION .text:CODE:REORDER(1)
PWMGen1_IRQHandler
        B PWMGen1_IRQHandler

        PUBWEAK PWMGen2_IRQHandler
        SECTION .text:CODE:REORDER(1)
PWMGen2_IRQHandler
        B PWMGen2_IRQHandler

        PUBWEAK QEI0_IRQHandler
        SECTION .text:CODE:REORDER(1)
QEI0_IRQHandler
        B QEI0_IRQHandler

        PUBWEAK ADCSeq0_IRQHandler
        SECTION .text:CODE:REORDER(1)
ADCSeq0_IRQHandler
        B ADCSeq0_IRQHandler

        PUBWEAK ADCSeq1_IRQHandler
        SECTION .text:CODE:REORDER(1)
ADCSeq1_IRQHandler
        B ADCSeq1_IRQHandler

        PUBWEAK ADCSeq2_IRQHandler
        SECTION .text:CODE:REORDER(1)
ADCSeq2_IRQHandler
        B ADCSeq2_IRQHandler

        PUBWEAK ADCSeq3_IRQHandler
        SECTION .text:CODE:REORDER(1)
ADCSeq3_IRQHandler
        B ADCSeq3_IRQHandler

        PUBWEAK Watchdog_IRQHandler
        SECTION .text:CODE:REORDER(1)
Watchdog_IRQHandler
        B Watchdog_IRQHandler

        PUBWEAK Timer0A_IRQHandler
        SECTION .text:CODE:REORDER(1)
Timer0A_IRQHandler
        B Timer0A_IRQHandler

        PUBWEAK Timer0B_IRQHandler
        SECTION .text:CODE:REORDER(1)
Timer0B_IRQHandler
        B Timer0B_IRQHandler

        PUBWEAK Timer1A_IRQHandler
        SECTION .text:CODE:REORDER(1)
Timer1A_IRQHandler
        B Timer1A_IRQHandler

        PUBWEAK Timer1B_IRQHandler
        SECTION .text:CODE:REORDER(1)
Timer1B_IRQHandler
        B Timer1B_IRQHandler

        PUBWEAK Timer2A_IRQHandler
        SECTION .text:CODE:REORDER(1)
Timer2A_IRQHandler
        B Timer2A_IRQHandler

        PUBWEAK Timer2B_IRQHandler
        SECTION .text:CODE:REORDER(1)
Timer2B_IRQHandler
        B Timer2B_IRQHandler

        PUBWEAK Comp0_IRQHandler
        SECTION .text:CODE:REORDER(1)
Comp0_IRQHandler
        B Comp0_IRQHandler

        PUBWEAK Comp1_IRQHandler
        SECTION .text:CODE:REORDER(1)
Comp1_IRQHandler
        B Comp1_IRQHandler

        PUBWEAK Comp2_IRQHandler
        SECTION .text:CODE:REORDER(1)
Comp2_IRQHandler
        B Comp2_IRQHandler

        PUBWEAK SysCtrl_IRQHandler
        SECTION .text:CODE:REORDER(1)
SysCtrl_IRQHandler
        B SysCtrl_IRQHandler

        PUBWEAK FlashCtrl_IRQHandler
        SECTION .text:CODE:REORDER(1)
FlashCtrl_IRQHandler
        B FlashCtrl_IRQHandler

        PUBWEAK GPIOPortF_IRQHandler
        SECTION .text:CODE:REORDER(1)
GPIOPortF_IRQHandler
        B GPIOPortF_IRQHandler

        PUBWEAK GPIOPortG_IRQHandler
        SECTION .text:CODE:REORDER(1)
GPIOPortG_IRQHandler
        B GPIOPortG_IRQHandler

        PUBWEAK GPIOPortH_IRQHandler
        SECTION .text:CODE:REORDER(1)
GPIOPortH_IRQHandler
        B GPIOPortH_IRQHandler

        PUBWEAK USART2_IRQHandler
        SECTION .text:CODE:REORDER(1)
USART2_IRQHandler
        B USART2_IRQHandler

        PUBWEAK SSI1_IRQHandler
        SECTION .text:CODE:REORDER(1)
SSI1_IRQHandler
        B SSI1_IRQHandler

        PUBWEAK Timer3A_IRQHandler
        SECTION .text:CODE:REORDER(1)
Timer3A_IRQHandler
        B Timer3A_IRQHandler

        PUBWEAK Timer3B_IRQHandler
        SECTION .text:CODE:REORDER(1)
Timer3B_IRQHandler
        B Timer3B_IRQHandler

        PUBWEAK I2C1_IRQHandler
        SECTION .text:CODE:REORDER(1)
I2C1_IRQHandler
        B I2C1_IRQHandler

        PUBWEAK QEI1_IRQHandler
        SECTION .text:CODE:REORDER(1)
QEI1_IRQHandler
        B QEI1_IRQHandler

        PUBWEAK CAN0_IRQHandler
        SECTION .text:CODE:REORDER(1)
CAN0_IRQHandler
        B CAN0_IRQHandler

        PUBWEAK CAN1_IRQHandler
        SECTION .text:CODE:REORDER(1)
CAN1_IRQHandler
        B CAN1_IRQHandler

        PUBWEAK CAN2_IRQHandler
        SECTION .text:CODE:REORDER(1)
CAN2_IRQHandler
        B CAN2_IRQHandler

        PUBWEAK Ethernet_IRQHandler
        SECTION .text:CODE:REORDER(1)
Ethernet_IRQHandler
        B Ethernet_IRQHandler

        PUBWEAK Hibernate_IRQHandler
        SECTION .text:CODE:REORDER(1)
Hibernate_IRQHandler
        B Hibernate_IRQHandler

        PUBWEAK USB0_IRQHandler
        SECTION .text:CODE:REORDER(1)
USB0_IRQHandler
        B USB0_IRQHandler

        PUBWEAK PWMGen3_IRQHandler
        SECTION .text:CODE:REORDER(1)
PWMGen3_IRQHandler
        B PWMGen3_IRQHandler

        PUBWEAK uDMA_IRQHandler
        SECTION .text:CODE:REORDER(1)
uDMA_IRQHandler
        B uDMA_IRQHandler

        PUBWEAK uDMAErr_IRQHandler
        SECTION .text:CODE:REORDER(1)
uDMAErr_IRQHandler
        B uDMAErr_IRQHandler


        END
