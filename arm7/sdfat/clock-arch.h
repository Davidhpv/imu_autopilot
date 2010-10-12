#ifndef __TIMER_ARCH_H_
#define __TIMER_ARCH_H_
//-----------------------------------------------------------------------------

#include "typedefs.h"
//-----------------------------------------------------------------------------

extern U32 timer_counter;

void tc0_cmp(void); 
void init_timer(void);
unsigned int SetDelay10ms (int t);
unsigned char CheckDelay10ms ( int t);


#define VICVectCntl0_ENABLE (1<<5)
#define VIC_Channel_Timer0  4

#define TxTCR_COUNTER_ENABLE (1<<0)
#define TxTCR_COUNTER_RESET  (1<<1)
#define TxMCR_INT_ON_MR0     (1<<0)
#define TxMCR_RESET_ON_MR0   (1<<1)
#define TxIR_MR0_FLAG        (1<<0)

#endif


