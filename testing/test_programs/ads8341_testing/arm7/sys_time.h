#ifndef SYS_TIME_H
#define SYS_TIME_H

#include "LPC21xx.h"
#include "armVIC.h"
#include "conf.h"


//int timer_hit=0;

void sys_time_init( void );

void sys_time_periodic_init(void);

int sys_time_periodic(void);

void sys_time_wait(unsigned int usec);

#define SYS_TICS_OF_SEC(s)   (unsigned int)(s * PCLK + 0.5)
#define SYS_TICS_OF_USEC(us) SYS_TICS_OF_SEC((us) * 1e-6)
#define SYS_USEC_OF_TICS(tics) (unsigned int)((tics * 1e6 -0.5)/PCLK)

#endif /* SYS_TIME_H */
