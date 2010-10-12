/**************************************************************************//**
 * @file
 * @brief DVK register acess, prototypes and definitions
 * @author Energy Micro AS
 * @version 1.1.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2009 Energy Micro AS, http://www.energymicro.com</b>
 ******************************************************************************
 *
 * This source code is the property of Energy Micro AS. The source and compiled
 * code may only be used on Energy Micro "EFM32" microcontrollers.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/

#ifndef __DVKREG_H
#define __DVKREG_H

#include <stdint.h>
#include "efm32.h"

/**************************************************************************//**
 * Defines FPGA register bank for Energy Micro Development Kit (DVK) board,
 * i.e. DVK control registers
 *****************************************************************************/
typedef struct
{
  uint16_t RESERVED0;         /* 0x00 */
  uint16_t EM;                /* 0x01 */
  uint16_t RESERVED2;         /* 0x02 */

  uint16_t UIF_LEDS;          /* 0x03 */
  uint16_t UIF_PB;            /* 0x04 */
  uint16_t UIF_DIP;           /* 0x05 */
  uint16_t UIF_JOYSTICK;      /* 0x06 */
  uint16_t UIF_AEM;           /* 0x07 */

  uint16_t DISPLAY_CTRL;      /* 0x08 */
  uint16_t EBI_CFG;           /* 0x09 */
  uint16_t BUS_CFG;           /* 0x0a */
  uint16_t INTREG;            /* 0x0b */
  uint16_t PERICON;           /* 0x0c */
  uint16_t AEM_STATE;         /* 0x0d */

  uint16_t SPI_CFG;           /* 0x0e */
  uint16_t RESET;             /* 0x0f */

  uint16_t ADC_START;         /* 0x10 */
  uint16_t ADC_STATUS;        /* 0x11 */
  uint16_t ADC_DATA;          /* 0x12 */

  uint16_t RESERVED3;         /* 0x13 */

  uint16_t HW_VERSION;        /* 0x14 */
  uint16_t FW_BUILDNO;        /* 0x15 */
  uint16_t FW_VERSION;        /* 0x16 */

  uint16_t SCRATCH_COMMON;    /* 0x17 */

  uint16_t SCRATCH_EFM0;      /* 0x18 */
  uint16_t SCRATCH_EFM1;      /* 0x19 */
  uint16_t SCRATCH_EFM2;      /* 0x1A */
  uint16_t SCRATCH_EFM3;      /* 0x1B */

  uint16_t SCRATCH_STM0;      /* 0x1C */
  uint16_t SCRATCH_STM1;      /* 0x1D */
  uint16_t SCRATCH_STM2;      /* 0x1E */
  uint16_t SCRATCH_STM3;      /* 0x1f */

  uint16_t INTFLAG;
  uint16_t INTEN;
} FPGARegister_TypeDef;

#define FPGA_REGISTER_BASE        0x8c000000
#define DVK_REGISTER              ((FPGARegister_TypeDef *) FPGA_REGISTER_BASE)

#define PERICON_ACCEL             (1 << 0)
#define PERICON_LDR               (1 << 1)
#define PERICON_POTMETER          (1 << 2)
#define PERICON_RS232_A           (1 << 3)
#define PERICON_RS232_B           (1 << 4)
#define PERICON_SPI               (1 << 5)
#define PERICON_I2C               (1 << 6)
#define PERICON_IRDA              (1 << 7)
#define PERICON_ANALOG_SE         (1 << 8)
#define PERICON_ANALOG_DIFF       (1 << 9)
#define PERICON_AUDIO_OUT         (1 << 10)
#define PERICON_AUDIO_IN          (1 << 11)
#define PERICON_ACCEL_GSEL        (1 << 12)
#define PERICON_ACCEL_SELFTEST    (1 << 13)
#define PERICON_RS232_SHUTDOWN    (1 << 14)
#define PERICON_IRDA_SHUTDOWN     (1 << 15)

#define INTEN_PB                  (1 << 0)
#define INTEN_DIP                 (1 << 1)
#define INTEN_JOYSTICK            (1 << 2)
#define INTEN_AEM                 (1 << 3)

void DVK_init(void);
void DVK_writeRegister(uint16_t *addr, uint16_t data);
uint16_t DVK_readRegister(uint16_t *addr);

#endif
