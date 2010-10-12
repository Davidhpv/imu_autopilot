#ifndef _viclowlevel_
#define _viclowlevel_

unsigned enableIRQ(void);
unsigned disableIRQ(void);
unsigned restoreIRQ(unsigned oldCPSR);
inline unsigned asm_get_cpsr(void);
inline void asm_set_cpsr(unsigned val);

#endif //_viclowlevel_


