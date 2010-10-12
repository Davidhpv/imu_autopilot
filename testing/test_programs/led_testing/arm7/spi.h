#ifndef SPI_H
#define SPI_H

#include "LPC21xx.h"

#define SpiEnable() {		\
    SSPCR1 |= (1 << SSE);	\
  }

#define SpiDisable() {		\
    SSPCR1 &= ~(1 << SSE);	\
  }

#define SpiEnableRti() {	\
    SSPIMSC |= (1 << RTIM);	\
  }

#define SpiDisableRti() {	\
    SSPIMSC &= ~(1 << RTIM);	\
  }

#define SpiClearRti() {         \
    SSPICR |= (1 << RTIC);	\
  }

#define SpiEnableTxi() {	\
    SSPIMSC |= (1 << TXIM);	\
  }

#define SpiDisableTxi() {	\
    SSPIMSC &= ~(1 << TXIM);	\
  }

#define SpiEnableRxi() {	\
    SSPIMSC |= (1 << RXIM);	\
  }

#define SpiDisableRxi() {	\
    SSPIMSC &= ~(1 << RXIM);	\
  }

#endif /* SPI_H */
