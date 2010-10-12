#ifndef PPM_H
#define PPM_H

#include "LPC21xx.h"

int ppm_get_channel(unsigned int nr);

void ppm_init(void);

void PPM_ISR(void);

#endif /* PPM_H */
