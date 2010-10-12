/*****************************************************************************/
/* startup_stm32F10x_hd.s: Startup file for STM32F10x HD device series       */
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
    .long   WWDG_IRQHandler             /* Window Watchdog                     */
    .long   PVD_IRQHandler              /* PVD through EXTI Line detect        */
    .long   TAMPER_IRQHandler           /* Tamper                              */
    .long   RTC_IRQHandler              /* RTC                                 */
    .long   FLASH_IRQHandler            /* Flash                               */
    .long   RCC_IRQHandler              /* RCC                                 */
    .long   EXTI0_IRQHandler            /* EXTI Line 0                         */
    .long   EXTI1_IRQHandler            /* EXTI Line 1                         */
    .long   EXTI2_IRQHandler            /* EXTI Line 2                         */
    .long   EXTI3_IRQHandler            /* EXTI Line 3                         */
    .long   EXTI4_IRQHandler            /* EXTI Line 4                         */
    .long   DMA1_Channel1_IRQHandler    /* DMA1 Channel 1                      */
    .long   DMA1_Channel2_IRQHandler    /* DMA1 Channel 2                      */
    .long   DMA1_Channel3_IRQHandler    /* DMA1 Channel 3                      */
    .long   DMA1_Channel4_IRQHandler    /* DMA1 Channel 4                      */
    .long   DMA1_Channel5_IRQHandler    /* DMA1 Channel 5                      */
    .long   DMA1_Channel6_IRQHandler    /* DMA1 Channel 6                      */
    .long   DMA1_Channel7_IRQHandler    /* DMA1 Channel 7                      */                   
    .long   ADC1_2_IRQHandler           /* ADC1 & ADC2                         */
    .long   USB_HP_CAN1_TX_IRQHandler   /* USB High Priority or CAN1 TX        */
    .long   USB_LP_CAN1_RX0_IRQHandler  /* USB Low  Priority or CAN1 RX0       */
    .long   CAN1_RX1_IRQHandler         /* CAN1 RX1                            */
    .long   CAN1_SCE_IRQHandler         /* CAN1 SCE                            */
    .long   EXTI9_5_IRQHandler          /* EXTI Line 9..5                      */
    .long   TIM1_BRK_IRQHandler         /* TIM1 Break                          */
    .long   TIM1_UP_IRQHandler          /* TIM1 Update                         */
    .long   TIM1_TRG_COM_IRQHandler     /* TIM1 Trigger and Commutation        */
    .long   TIM1_CC_IRQHandler          /* TIM1 Capture Compare                */
    .long   TIM2_IRQHandler             /* TIM2                                */
    .long   TIM3_IRQHandler             /* TIM3                                */
    .long   TIM4_IRQHandler             /* TIM4                                */
    .long   I2C1_EV_IRQHandler          /* I2C1 Event                          */
    .long   I2C1_ER_IRQHandler          /* I2C1 Error                          */
    .long   I2C2_EV_IRQHandler          /* I2C2 Event                          */
    .long   I2C2_ER_IRQHandler          /* I2C2 Error                          */
    .long   SPI1_IRQHandler             /* SPI1                                */
    .long   SPI2_IRQHandler             /* SPI2                                */
    .long   USART1_IRQHandler           /* USART1                              */
    .long   USART2_IRQHandler           /* USART2                              */
    .long   USART3_IRQHandler           /* USART3                              */
    .long   EXTI15_10_IRQHandler        /* EXTI Line 15..10                    */
    .long   RTCAlarm_IRQHandler         /* RTC Alarm through EXTI Line         */
    .long   USBWakeUp_IRQHandler        /* USB Wakeup from suspend             */
    .long   TIM8_BRK_IRQHandler         /* TIM8 Break                          */
    .long   TIM8_UP_IRQHandler          /* TIM8 Update                         */
    .long   TIM8_TRG_COM_IRQHandler     /* TIM8 Trigger and Commutation        */
    .long   TIM8_CC_IRQHandler          /* TIM8 Capture Compare                */
    .long   ADC3_IRQHandler             /* ADC3                                */
    .long   FSMC_IRQHandler             /* FSMC                                */
    .long   SDIO_IRQHandler             /* SDIO                                */
    .long   TIM5_IRQHandler             /* TIM5                                */
    .long   SPI3_IRQHandler             /* SPI3                                */
    .long   UART4_IRQHandler            /* UART4                               */
    .long   UART5_IRQHandler            /* UART5                               */
    .long   TIM6_IRQHandler             /* TIM6                                */
    .long   TIM7_IRQHandler             /* TIM7                                */
    .long   DMA2_Channel1_IRQHandler    /* DMA2 Channel1                       */
    .long   DMA2_Channel2_IRQHandler    /* DMA2 Channel2                       */
    .long   DMA2_Channel3_IRQHandler    /* DMA2 Channel3                       */
    .long   DMA2_Channel4_5_IRQHandler  /* DMA2 Channel4 & Channel5            */

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

    IRQ     WWDG_IRQHandler
    IRQ     PVD_IRQHandler             
    IRQ     TAMPER_IRQHandler          
    IRQ     RTC_IRQHandler             
    IRQ     FLASH_IRQHandler           
    IRQ     RCC_IRQHandler             
    IRQ     EXTI0_IRQHandler           
    IRQ     EXTI1_IRQHandler           
    IRQ     EXTI2_IRQHandler           
    IRQ     EXTI3_IRQHandler           
    IRQ     EXTI4_IRQHandler           
    IRQ     DMA1_Channel1_IRQHandler   
    IRQ     DMA1_Channel2_IRQHandler   
    IRQ     DMA1_Channel3_IRQHandler   
    IRQ     DMA1_Channel4_IRQHandler   
    IRQ     DMA1_Channel5_IRQHandler   
    IRQ     DMA1_Channel6_IRQHandler   
    IRQ     DMA1_Channel7_IRQHandler
    IRQ     ADC1_2_IRQHandler          
    IRQ     USB_HP_CAN1_TX_IRQHandler  
    IRQ     USB_LP_CAN1_RX0_IRQHandler 
    IRQ     CAN1_RX1_IRQHandler        
    IRQ     CAN1_SCE_IRQHandler        
    IRQ     EXTI9_5_IRQHandler         
    IRQ     TIM1_BRK_IRQHandler        
    IRQ     TIM1_UP_IRQHandler         
    IRQ     TIM1_TRG_COM_IRQHandler    
    IRQ     TIM1_CC_IRQHandler         
    IRQ     TIM2_IRQHandler            
    IRQ     TIM3_IRQHandler            
    IRQ     TIM4_IRQHandler            
    IRQ     I2C1_EV_IRQHandler         
    IRQ     I2C1_ER_IRQHandler         
    IRQ     I2C2_EV_IRQHandler         
    IRQ     I2C2_ER_IRQHandler         
    IRQ     SPI1_IRQHandler            
    IRQ     SPI2_IRQHandler            
    IRQ     USART1_IRQHandler          
    IRQ     USART2_IRQHandler          
    IRQ     USART3_IRQHandler          
    IRQ     EXTI15_10_IRQHandler       
    IRQ     RTCAlarm_IRQHandler        
    IRQ     USBWakeUp_IRQHandler       
    IRQ     TIM8_BRK_IRQHandler        
    IRQ     TIM8_UP_IRQHandler         
    IRQ     TIM8_TRG_COM_IRQHandler    
    IRQ     TIM8_CC_IRQHandler         
    IRQ     ADC3_IRQHandler            
    IRQ     FSMC_IRQHandler            
    IRQ     SDIO_IRQHandler            
    IRQ     TIM5_IRQHandler            
    IRQ     SPI3_IRQHandler            
    IRQ     UART4_IRQHandler           
    IRQ     UART5_IRQHandler           
    IRQ     TIM6_IRQHandler            
    IRQ     TIM7_IRQHandler            
    IRQ     DMA2_Channel1_IRQHandler
    IRQ     DMA2_Channel2_IRQHandler   
    IRQ     DMA2_Channel3_IRQHandler   
    IRQ     DMA2_Channel4_5_IRQHandler 

    .end
