/*****************************************************************************/
/* startup_TMP330.s: Startup file for TMP330 device series                   */
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

    .equ    Stack_Size, 0x00000200
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
    .long   MemManage_Handler           /* MPU Fault Handler             */
    .long   BusFault_Handler            /* Bus Fault Handler             */
    .long   UsageFault_Handler          /* Usage Fault Handler           */
    .long   0                           /* Reserved                      */
    .long   0                           /* Reserved                      */
    .long   0                           /* Reserved                      */
    .long   0                           /* Reserved                      */
    .long   SVC_Handler                 /* SVCall Handler                */
    .long   DebugMon_Handler            /* Debug Monitor Handler         */
    .long   0                           /* Reserved                      */
    .long   PendSV_Handler              /* PendSV Handler                */
    .long   SysTick_Handler             /* SysTick Handler               */

    /* External Interrupts */
    .long   INT0_IRQHandler             /*  0:  Interrupt pin PJ0/70 pin */
    .long   INT1_IRQHandler             /*  1:  Interrupt pin PJ1/49 pin */
    .long   INT2_IRQHandler             /*  2:  Interrupt pin PJ2/86 pin */
    .long   INT3_IRQHandler             /*  3:  Interrupt pin PJ3/87 pin */
    .long   INT4_IRQHandler             /*  4:  Interrupt pin PG3/6  pin */
    .long   INT5_IRQHandler             /*  5:  Interrupt pin PF7/19 pin */
    .long   INTRX0_IRQHandler           /*  6:  Serial receive 0         */
    .long   INTTX0_IRQHandler           /*  7:  Serial transmit 0        */
    .long   INTRX1_IRQHandler           /*  8:  Serial receive 1         */
    .long   INTTX1_IRQHandler           /*  9:  Serial transmit 1        */
    .long   INTSBI0_IRQHandler          /*  10: Serial bus interface 0   */
    .long   INTSBI1_IRQHandler          /*  11: Serial bus interface 1   */
    .long   INTCECRX_IRQHandler         /*  12: CEC receive              */
    .long   INTCECTX_IRQHandler         /*  13: CEC transmit             */
    .long   INTAINTRMCRX0_IRQHandler    /*  14: Remote ctrl reception 0  */
    .long   INTADHP_IRQHandler          /*  15: High priority ADC        */
    .long   INTADM0_IRQHandler          /*  16: ADC monitoring 0         */
    .long   INTADM1_IRQHandler          /*  17: ADC monitoring 1         */
    .long   INTTB0_IRQHandler           /*  18: TMRB match detection 0   */
    .long   INTTB1_IRQHandler           /*  19: TMRB match detection 1   */
    .long   INTTB2_IRQHandler           /*  20: TMRB match detection 2   */
    .long   INTTB3_IRQHandler           /*  21: TMRB match detection 3   */
    .long   INTTB4_IRQHandler           /*  22: TMRB match detection 4   */
    .long   INTTB5_IRQHandler           /*  23: TMRB match detection 5   */
    .long   INTTB6_IRQHandler           /*  24: TMRB match detection 6   */
    .long   INTRTC_IRQHandler           /*  25: RTC                      */
    .long   INTCAP00_IRQHandler         /*  26: TMRB input capture 00    */
    .long   INTCAP01_IRQHandler         /*  27: TMRB input capture 01    */
    .long   INTCAP10_IRQHandler         /*  28: TMRB input capture 10    */
    .long   INTCAP11_IRQHandler         /*  29: TMRB input capture 11    */
    .long   INTCAP50_IRQHandler         /*  30: TMRB input capture 50    */
    .long   INTCAP51_IRQHandler         /*  31: TMRB input capture 51    */
    .long   INTCAP60_IRQHandler         /*  32: TMRB input capture 60    */
    .long   INTCAP61_IRQHandler         /*  33: TMRB input capture 61    */
    .long   INT6_IRQHandler             /*  34: Interrupt pin PJ6/39 pin */
    .long   INT7_IRQHandler             /*  35: Interrupt pin PJ7/58 pin */
    .long   INTRX2_IRQHandler           /*  36: Serial receive 2         */
    .long   INTTX2_IRQHandler           /*  37: Serial transmit 2        */
    .long   INTSBI2_IRQHandler          /*  38: Serial bus interface 2   */
    .long   INTAINTRMCRX1_IRQHandler    /*  39: Remote ctrl reception 1  */
    .long   INTTB7_IRQHandler           /*  40: TMRB match detection 7   */
    .long   INTTB8_IRQHandler           /*  41: TMRB match detection 8   */
    .long   INTTB9_IRQHandler           /*  42: TMRB match detection 9   */
    .long   INTCAP20_IRQHandler         /*  43: TMRB input capture 20    */
    .long   INTCAP21_IRQHandler         /*  44: TMRB input capture 21    */
    .long   INTCAP30_IRQHandler         /*  45: TMRB input capture 30    */
    .long   INTCAP31_IRQHandler         /*  46: TMRB input capture 31    */
    .long   INTCAP40_IRQHandler         /*  47: TMRB input capture 40    */
    .long   INTCAP41_IRQHandler         /*  48: TMRB input capture 41    */
    .long   INTAD_IRQHandler            /*  49: ADC complete             */

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

    IRQ     INT0_IRQHandler
    IRQ     INT1_IRQHandler
    IRQ     INT2_IRQHandler
    IRQ     INT3_IRQHandler
    IRQ     INT4_IRQHandler
    IRQ     INT5_IRQHandler
    IRQ     INTRX0_IRQHandler
    IRQ     INTTX0_IRQHandler
    IRQ     INTRX1_IRQHandler
    IRQ     INTTX1_IRQHandler
    IRQ     INTSBI0_IRQHandler
    IRQ     INTSBI1_IRQHandler
    IRQ     INTCECRX_IRQHandler
    IRQ     INTCECTX_IRQHandler
    IRQ     INTAINTRMCRX0_IRQHandler
    IRQ     INTADHP_IRQHandler
    IRQ     INTADM0_IRQHandler
    IRQ     INTADM1_IRQHandler
    IRQ     INTTB0_IRQHandler
    IRQ     INTTB1_IRQHandler
    IRQ     INTTB2_IRQHandler
    IRQ     INTTB3_IRQHandler
    IRQ     INTTB4_IRQHandler
    IRQ     INTTB5_IRQHandler
    IRQ     INTTB6_IRQHandler
    IRQ     INTRTC_IRQHandler
    IRQ     INTCAP00_IRQHandler
    IRQ     INTCAP01_IRQHandler
    IRQ     INTCAP10_IRQHandler
    IRQ     INTCAP11_IRQHandler
    IRQ     INTCAP50_IRQHandler
    IRQ     INTCAP51_IRQHandler
    IRQ     INTCAP60_IRQHandler
    IRQ     INTCAP61_IRQHandler
    IRQ     INT6_IRQHandler
    IRQ     INT7_IRQHandler
    IRQ     INTRX2_IRQHandler
    IRQ     INTTX2_IRQHandler
    IRQ     INTSBI2_IRQHandler
    IRQ     INTAINTRMCRX1_IRQHandler
    IRQ     INTTB7_IRQHandler
    IRQ     INTTB8_IRQHandler
    IRQ     INTTB9_IRQHandler
    IRQ     INTCAP20_IRQHandler
    IRQ     INTCAP21_IRQHandler
    IRQ     INTCAP30_IRQHandler
    IRQ     INTCAP31_IRQHandler
    IRQ     INTCAP40_IRQHandler
    IRQ     INTCAP41_IRQHandler
    IRQ     INTAD_IRQHandler

    .end
