/*****************************************************************************/
/* startup_LPC11xx.s: Startup file for LPC11xx device series                 */
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
    .long   __cs3_stack                 /* Top of Stack                  */
    .long   __cs3_reset                 /* Reset Handler                 */
    .long   NMI_Handler                 /* NMI Handler                   */
    .long   HardFault_Handler           /* Hard Fault Handler            */
    .long   0                           /* Reserved                      */
    .long   0                           /* Reserved                      */
    .long   0                           /* Reserved                      */
    .long   0                           /* Reserved                      */
    .long   0                           /* Reserved                      */
    .long   0                           /* Reserved                      */
    .long   0                           /* Reserved                      */
    .long   SVC_Handler                 /* SVCall Handler                */
    .long   0                           /* Reserved                      */
    .long   0                           /* Reserved                      */
    .long   PendSV_Handler              /* PendSV Handler                */
    .long   SysTick_Handler             /* SysTick Handler               */

    /* External Interrupts */
    .long   WAKEUP_IRQHandler           /* 16+ 0: Wakeup PIO0.0          */
    .long   WAKEUP_IRQHandler           /* 16+ 1: Wakeup PIO0.1          */
    .long   WAKEUP_IRQHandler           /* 16+ 2: Wakeup PIO0.2          */
    .long   WAKEUP_IRQHandler           /* 16+ 3: Wakeup PIO0.3          */
    .long   WAKEUP_IRQHandler           /* 16+ 4: Wakeup PIO0.4          */
    .long   WAKEUP_IRQHandler           /* 16+ 5: Wakeup PIO0.5          */
    .long   WAKEUP_IRQHandler           /* 16+ 6: Wakeup PIO0.6          */
    .long   WAKEUP_IRQHandler           /* 16+ 7: Wakeup PIO0.7          */
    .long   WAKEUP_IRQHandler           /* 16+ 8: Wakeup PIO0.8          */
    .long   WAKEUP_IRQHandler           /* 16+ 9: Wakeup PIO0.9          */
    .long   WAKEUP_IRQHandler           /* 16+10: Wakeup PIO0.10         */
    .long   WAKEUP_IRQHandler           /* 16+11: Wakeup PIO0.11         */
    .long   WAKEUP_IRQHandler           /* 16+12: Wakeup PIO1.0          */
    .long   0                           /* 16+13: Reserved               */
    .long   SSP1_IRQHandler             /* 16+14: SSP1                   */
    .long   I2C_IRQHandler              /* 16+15: I2C                    */
    .long   TIMER16_0_IRQHandler        /* 16+16: 16-bit Counter-Timer 0 */
    .long   TIMER16_1_IRQHandler        /* 16+17: 16-bit Counter-Timer 1 */
    .long   TIMER32_0_IRQHandler        /* 16+18: 32-bit Counter-Timer 0 */
    .long   TIMER32_1_IRQHandler        /* 16+19: 32-bit Counter-Timer 1 */
    .long   SSP0_IRQHandler             /* 16+20: SSP0                   */
    .long   UART_IRQHandler             /* 16+21: UART                   */
    .long   0                           /* 16+22: Reserved               */
    .long   0                           /* 16+24: Reserved               */
    .long   ADC_IRQHandler              /* 16+24: A/D Converter          */
    .long   WDT_IRQHandler              /* 16+25: Watchdog Timer         */
    .long   BOD_IRQHandler              /* 16+26: Brown Out Detect       */
    .long   0                           /* 16+27: Reserved               */
    .long   PIOINT3_IRQHandler          /* 16+28: PIO INT3               */
    .long   PIOINT2_IRQHandler          /* 16+29: PIO INT2               */
    .long   PIOINT1_IRQHandler          /* 16+30: PIO INT1               */
    .long   PIOINT0_IRQHandler          /* 16+31: PIO INT0               */

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

    .weak   SVC_Handler
    .type   SVC_Handler, %function
SVC_Handler:
    B       .
    .size   SVC_Handler, . - SVC_Handler

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

    IRQ     WAKEUP_IRQHandler
    IRQ     SSP1_IRQHandler
    IRQ     I2C_IRQHandler
    IRQ     TIMER16_0_IRQHandler
    IRQ     TIMER16_1_IRQHandler
    IRQ     TIMER32_0_IRQHandler
    IRQ     TIMER32_1_IRQHandler
    IRQ     SSP0_IRQHandler
    IRQ     UART_IRQHandler
    IRQ     ADC_IRQHandler
    IRQ     WDT_IRQHandler
    IRQ     BOD_IRQHandler
    IRQ     PIOINT3_IRQHandler
    IRQ     PIOINT2_IRQHandler
    IRQ     PIOINT1_IRQHandler
    IRQ     PIOINT0_IRQHandler

    .end
