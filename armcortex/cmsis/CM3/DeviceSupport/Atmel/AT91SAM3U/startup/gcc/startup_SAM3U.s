/*****************************************************************************/
/* startup_SAM3U.s: Startup file for AT91SAM3U device series                 */
/*****************************************************************************/
/* Version: CodeSourcery Sourcery G++ Lite (with CS3)                        */
/*****************************************************************************/


/* 
//*** <<< Use Configuration Wizard in Context Menu >>> *** 
*/


/*
// <h> Stack Configuration
//   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
// </h>
*/

    .equ    Stack_Size, 0x00000100
    .section ".stack", "w"
    .align  3
    .globl  __cs3_stack_mem
    .globl  __cs3_stack_size
__cs3_stack_mem:
    .if     Stack_Size
    .space  Stack_Size
    .endif
    .size   __cs3_stack_mem,  . - __cs3_stack_mem
    .set    __cs3_stack_size, . - __cs3_stack_mem


/*
// <h> Heap Configuration
//   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
// </h>
*/

    .equ    Heap_Size,  0x00001000
    
    .section ".heap", "w"
    .align  3
    .globl  __cs3_heap_start
    .globl  __cs3_heap_end
__cs3_heap_start:
    .if     Heap_Size
    .space  Heap_Size
    .endif
__cs3_heap_end:


/* Vector Table */

    .section ".cs3.interrupt_vector"
    .globl  __cs3_interrupt_vector_cortex_m
    .type   __cs3_interrupt_vector_cortex_m, %object

__cs3_interrupt_vector_cortex_m:
    .long   __cs3_stack                 /* Top of Stack                 */
    .long   __cs3_reset                 /* Reset Handler                */
    .long   NMI_Handler                 /* NMI Handler                  */
    .long   HardFault_Handler           /* Hard Fault Handler           */
    .long   MemManage_Handler           /* MPU Fault Handler            */
    .long   BusFault_Handler            /* Bus Fault Handler            */
    .long   UsageFault_Handler          /* Usage Fault Handler          */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   SVC_Handler                 /* SVCall Handler               */
    .long   DebugMon_Handler            /* Debug Monitor Handler        */
    .long   0                           /* Reserved                     */
    .long   PendSV_Handler              /* PendSV Handler               */
    .long   SysTick_Handler             /* SysTick Handler              */

    /* External Interrupts */
    .long   SUPC_IRQHandler             /* 16: Supply Controller                    */
    .long   RSTC_IRQHandler             /* 17: Reset Controller                     */
    .long   RTC_IRQHandler              /* 18: Real Time Clock                      */
    .long   RTT_IRQHandler              /* 19: Real Time Timer                      */
    .long   WDT_IRQHandler              /* 20: Watchdog Timer                       */
    .long   PMC_IRQHandler              /* 21: Power Management Controller          */
    .long   EEFC0_IRQHandler            /* 22: Enhanced Embedded Flash Controller 0 */
    .long   EEFC1_IRQHandler            /* 23: Enhanced Embedded Flash Controller 1 */
    .long   UART_IRQHandler             /* 24: UART                                 */
    .long   SMC_IRQHandler              /* 25: Static Memory Controller             */
    .long   PIOA_IRQHandler             /* 26: Parallel I/O Controller A            */
    .long   PIOB_IRQHandler             /* 27: Parallel I/O Controller B            */
    .long   PIOC_IRQHandler             /* 28: Parallel I/O Controller C            */
    .long   USART0_IRQHandler           /* 29: USART 0                              */
    .long   USART1_IRQHandler           /* 30: USART 1                              */
    .long   USART2_IRQHandler           /* 31: USART 2                              */
    .long   USART3_IRQHandler           /* 32: USART 3                              */
    .long   HSMCI_IRQHandler            /* 33: High Speed Multimedia Card Interface */
    .long   TWI0_IRQHandler             /* 34: Two-wire Interface 0                 */
    .long   TWI1_IRQHandler             /* 35: Two-wire Interface 1                 */
    .long   SPI_IRQHandler              /* 36: Synchronous Peripheral Interface     */
    .long   SSC_IRQHandler              /* 37: Synchronous Serial Controller        */
    .long   TC0_IRQHandler              /* 38: Timer Counter 0                      */
    .long   TC1_IRQHandler              /* 39: Timer Counter 1                      */
    .long   TC2_IRQHandler              /* 40: Timer Counter 2                      */
    .long   PWM_IRQHandler              /* 41: Pulse Width Modulation Controller    */
    .long   ADC12B_IRQHandler           /* 42: 12-bit ADC Controller                */
    .long   ADC_IRQHandler              /* 43: 10-bit ADC Controller                */
    .long   DMAC_IRQHandler             /* 44: DMA Controller                       */
    .long   UDPHS_IRQHandler            /* 45: USB Device High Speed                */

    .size   __cs3_interrupt_vector_cortex_m, . - __cs3_interrupt_vector_cortex_m


    .thumb


/* Reset Handler */

    .section .cs3.reset,"x",%progbits
    .thumb_func
    .globl  __cs3_reset_cortex_m
    .type   __cs3_reset_cortex_m, %function
__cs3_reset_cortex_m:
    .fnstart
    LDR     R0, =SystemInit
    BLX     R0
    LDR     R0,=_start
    BX      R0
    .pool
    .cantunwind
    .fnend
    .size   __cs3_reset_cortex_m,.-__cs3_reset_cortex_m

    .section ".text"

/* Exception Handlers */

    .weak   NMI_Handler
    .type   NMI_Handler, %function
NMI_Handler:
    B       .
    .size   NMI_Handler, . - NMI_Handler

    .weak   HardFault_Handler
    .type   HardFault_Handler, %function
HardFault_Handler:
    B       .
    .size   HardFault_Handler, . - HardFault_Handler

    .weak   MemManage_Handler
    .type   MemManage_Handler, %function
MemManage_Handler:
    B       .
    .size   MemManage_Handler, . - MemManage_Handler

    .weak   BusFault_Handler
    .type   BusFault_Handler, %function
BusFault_Handler:
    B       .
    .size   BusFault_Handler, . - BusFault_Handler

    .weak   UsageFault_Handler
    .type   UsageFault_Handler, %function
UsageFault_Handler:
    B       .
    .size   UsageFault_Handler, . - UsageFault_Handler

    .weak   SVC_Handler
    .type   SVC_Handler, %function
SVC_Handler:
    B       .
    .size   SVC_Handler, . - SVC_Handler

    .weak   DebugMon_Handler
    .type   DebugMon_Handler, %function
DebugMon_Handler:
    B       .
    .size   DebugMon_Handler, . - DebugMon_Handler

    .weak   PendSV_Handler
    .type   PendSV_Handler, %function
PendSV_Handler:
    B       .
    .size   PendSV_Handler, . - PendSV_Handler

    .weak   SysTick_Handler
    .type   SysTick_Handler, %function
SysTick_Handler:
    B       .
    .size   SysTick_Handler, . - SysTick_Handler


/* IRQ Handlers */

    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    B       .
    .size   Default_Handler, . - Default_Handler

    .macro  IRQ handler
    .weak   \handler
    .set    \handler, Default_Handler
    .endm

    IRQ     SUPC_IRQHandler
    IRQ     RSTC_IRQHandler
    IRQ     RTC_IRQHandler
    IRQ     RTT_IRQHandler
    IRQ     WDT_IRQHandler
    IRQ     PMC_IRQHandler
    IRQ     EEFC0_IRQHandler
    IRQ     EEFC1_IRQHandler
    IRQ     UART_IRQHandler
    IRQ     SMC_IRQHandler
    IRQ     PIOA_IRQHandler
    IRQ     PIOB_IRQHandler
    IRQ     PIOC_IRQHandler
    IRQ     USART0_IRQHandler
    IRQ     USART1_IRQHandler
    IRQ     USART2_IRQHandler
    IRQ     USART3_IRQHandler
    IRQ     HSMCI_IRQHandler
    IRQ     TWI0_IRQHandler
    IRQ     TWI1_IRQHandler
    IRQ     SPI_IRQHandler
    IRQ     SSC_IRQHandler
    IRQ     TC0_IRQHandler
    IRQ     TC1_IRQHandler
    IRQ     TC2_IRQHandler
    IRQ     PWM_IRQHandler
    IRQ     ADC12B_IRQHandler
    IRQ     ADC_IRQHandler
    IRQ     DMAC_IRQHandler
    IRQ     UDPHS_IRQHandler

    .end
