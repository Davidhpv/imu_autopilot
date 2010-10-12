#ifndef MAX1168_H_
#define MAX1168_H_

#include "LPC21xx.h"

#define MAX1168_NB_CHAN 8

#define MAX1168_ERR_ISR_STATUS   0
#define MAX1168_ERR_READ_OVERUN  1
#define MAX1168_ERR_SPURIOUS_EOC 2

#define MAX1168_SS_PIN 20
#define MAX1168_SS_IODIR IO0DIR
#define MAX1168_SS_IOSET IO0SET
#define MAX1168_SS_IOCLR IO0CLR

#define MAX1168_EOC_PIN 16
#define MAX1168_EOC_PINSEL PINSEL1
#define MAX1168_EOC_PINSEL_BIT 0
#define MAX1168_EOC_PINSEL_VAL 1
#define MAX1168_EOC_EINT 0

void max1168_init( void );
void max1168_read_channel(int channel);
unsigned short max1168_get_value(int channel);

#endif /*MAX1168_H_*/
