/*****************************************************************************/
/* startup_LM3S.s: Startup file for LM3S device series                       */
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
    .long   GPIOPortA_IRQHandler        /*  0: GPIO Port A              */
    .long   GPIOPortB_IRQHandler        /*  1: GPIO Port B              */
    .long   GPIOPortC_IRQHandler        /*  2: GPIO Port C              */
    .long   GPIOPortD_IRQHandler        /*  3: GPIO Port D              */
    .long   GPIOPortE_IRQHandler        /*  4: GPIO Port E              */
    .long   UART0_IRQHandler            /*  5: UART0                    */
    .long   UART1_IRQHandler            /*  6: UART1                    */
    .long   SSI0_IRQHandler             /*  7: SSI0                     */
    .long   I2C0_IRQHandler             /*  8: I2C0                     */
    .long   PWMFault_IRQHandler         /*  9: PWM Fault                */
    .long   PWMGen0_IRQHandler          /* 10: PWM Generator 0          */
    .long   PWMGen1_IRQHandler          /* 11: PWM Generator 1          */
    .long   PWMGen2_IRQHandler          /* 12: PWM Generator 2          */
    .long   QEI0_IRQHandler             /* 13: Quadrature Encoder       */
    .long   ADCSeq0_IRQHandler          /* 14: ADC Sequence 0           */
    .long   ADCSeq1_IRQHandler          /* 15: ADC Sequence 1           */
    .long   ADCSeq2_IRQHandler          /* 16: ADC Sequence 2           */
    .long   ADCSeq3_IRQHandler          /* 17: ADC Sequence 3           */
    .long   Watchdog_IRQHandler         /* 18: Watchdog                 */
    .long   Timer0A_IRQHandler          /* 19: Timer 0A                 */
    .long   Timer0B_IRQHandler          /* 20: Timer 0B                 */
    .long   Timer1A_IRQHandler          /* 21: Timer 1A                 */
    .long   Timer1B_IRQHandler          /* 22: Timer 1B                 */
    .long   Timer2A_IRQHandler          /* 23: Timer 2A                 */
    .long   Timer2B_IRQHandler          /* 24: Timer 2B                 */
    .long   Comp0_IRQHandler            /* 25: Comp 0                   */
    .long   Comp1_IRQHandler            /* 26: Comp 1                   */
    .long   Comp2_IRQHandler            /* 27: Comp 2                   */
    .long   SysCtrl_IRQHandler          /* 28: System Control           */
    .long   FlashCtrl_IRQHandler        /* 29: Flash Control            */
    .long   GPIOPortF_IRQHandler        /* 30: GPIO Port F              */
    .long   GPIOPortG_IRQHandler        /* 31: GPIO Port G              */
    .long   GPIOPortH_IRQHandler        /* 32: GPIO Port H              */
    .long   USART2_IRQHandler           /* 33: UART2 Rx and Tx          */
    .long   SSI1_IRQHandler             /* 34: SSI1 Rx and Tx           */
    .long   Timer3A_IRQHandler          /* 35: Timer 3 subtimer A       */
    .long   Timer3B_IRQHandler          /* 36: Timer 3 subtimer B       */
    .long   I2C1_IRQHandler             /* 37: I2C1 Master and Slave    */
    .long   QEI1_IRQHandler             /* 38: Quadrature Encoder 1     */
    .long   CAN0_IRQHandler             /* 39: CAN0                     */
    .long   CAN1_IRQHandler             /* 40: CAN1                     */
    .long   CAN2_IRQHandler             /* 41: CAN2                     */
    .long   Ethernet_IRQHandler         /* 42: Ethernet                 */
    .long   Hibernate_IRQHandler        /* 43:Hibernate                 */
    .long   USB0_IRQHandler             /* 44: USB0                     */
    .long   PWMGen3_IRQHandler          /* 45: PWM Generator 3          */
    .long   uDMA_IRQHandler             /* 46: uDMA Software Transfer   */
    .long   uDMAErr_IRQHandler          /* 47: uDMA Error               */

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

    IRQ     WDT_IRQHandler
    IRQ     GPIOPortA_IRQHandler
    IRQ     GPIOPortB_IRQHandler
    IRQ     GPIOPortC_IRQHandler
    IRQ     GPIOPortD_IRQHandler
    IRQ     GPIOPortE_IRQHandler
    IRQ     UART0_IRQHandler
    IRQ     UART1_IRQHandler
    IRQ     SSI0_IRQHandler
    IRQ     I2C0_IRQHandler
    IRQ     PWMFault_IRQHandler
    IRQ     PWMGen0_IRQHandler
    IRQ     PWMGen1_IRQHandler
    IRQ     PWMGen2_IRQHandler
    IRQ     QEI0_IRQHandler
    IRQ     ADCSeq0_IRQHandler
    IRQ     ADCSeq1_IRQHandler
    IRQ     ADCSeq2_IRQHandler
    IRQ     ADCSeq3_IRQHandler
    IRQ     Watchdog_IRQHandler
    IRQ     Timer0A_IRQHandler
    IRQ     Timer0B_IRQHandler
    IRQ     Timer1A_IRQHandler
    IRQ     Timer1B_IRQHandler
    IRQ     Timer2A_IRQHandler
    IRQ     Timer2B_IRQHandler
    IRQ     Comp0_IRQHandler
    IRQ     Comp1_IRQHandler
    IRQ     Comp2_IRQHandler
    IRQ     SysCtrl_IRQHandler
    IRQ     FlashCtrl_IRQHandler
    IRQ     GPIOPortF_IRQHandler
    IRQ     GPIOPortG_IRQHandler
    IRQ     GPIOPortH_IRQHandler
    IRQ     USART2_IRQHandler
    IRQ     SSI1_IRQHandler
    IRQ     Timer3A_IRQHandler
    IRQ     Timer3B_IRQHandler
    IRQ     I2C1_IRQHandler
    IRQ     QEI1_IRQHandler
    IRQ     CAN0_IRQHandler
    IRQ     CAN1_IRQHandler
    IRQ     CAN2_IRQHandler
    IRQ     Ethernet_IRQHandler
    IRQ     Hibernate_IRQHandler
    IRQ     USB0_IRQHandler
    IRQ     PWMGen3_IRQHandler
    IRQ     uDMA_IRQHandler
    IRQ     uDMAErr_IRQHandler

    .end
