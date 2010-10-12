/**************************************************************************//**
 * @file
 * @brief DVK Register Access
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

#include "efm32.h"
#include "dvkreg.h"

#define clear_bit(reg, bit)    (reg &= ~(1 << bit))
#define set_bit(reg, bit)      (reg |= 1 << bit)

static uint16_t *lastAddr = 0;

/**************************************************************************//**
 * @brief  Initializes USART2 SPI interface for access to FPGA registers
 *         for board control
 *****************************************************************************/
static void spiInit(void)
{
  USART_TypeDef *usart = USART2;
  GPIO_TypeDef  *gpio  = GPIO;

  /* Configure SPI pins */
  gpio->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE13_MASK);
  gpio->P[2].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK |
                        _GPIO_P_MODEL_MODE3_MASK |
                        _GPIO_P_MODEL_MODE4_MASK |
                        _GPIO_P_MODEL_MODE5_MASK);

  gpio->P[2].MODEH |= GPIO_P_MODEH_MODE13_PUSHPULL;
  gpio->P[2].MODEL |= (GPIO_P_MODEL_MODE2_PUSHPULL |
                       GPIO_P_MODEL_MODE3_PUSHPULL |
                       GPIO_P_MODEL_MODE4_PUSHPULL |
                       GPIO_P_MODEL_MODE5_PUSHPULL);
  set_bit(gpio->P[2].DOUT, 5);

  /* Configure USART2 as SPI master with manual CS */
  usart->CTRL  = USART_CTRL_SYNC;
  usart->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN;
  usart->CMD   = USART_CMD_MASTEREN | USART_CMD_TXEN | USART_CMD_RXEN;
}

/**************************************************************************//**
 * @brief  Performs USART2 SPI Transfer
 *****************************************************************************/
static uint16_t spiAccess(uint8_t spiadr, uint8_t rw, uint16_t spidata)
{
  USART_TypeDef *usart = USART2;
  GPIO_TypeDef  *gpio  = GPIO;

  uint16_t      tmp;

  clear_bit(gpio->P[2].DOUT, 5);

  /* SPI address */
  usart->TXDATA = (spiadr & 0x3) | rw << 3;
  while (!(usart->STATUS & USART_STATUS_TXC)) ;
  tmp = (usart->RXDATA) << 0;

  /* SPI data LSB */
  usart->TXDATA = spidata & 0xFF;
  while (!(usart->STATUS & USART_STATUS_TXC)) ;
  tmp = (usart->RXDATA);

  /* SPI data MSB */
  usart->TXDATA = spidata >> 8;
  while (!(usart->STATUS & USART_STATUS_TXC)) ;
  tmp |= (usart->RXDATA) << 8;

  set_bit(gpio->P[2].DOUT, 5);

  return tmp;
}

/**************************************************************************//**
 * @brief  Performs USART2 SPI write to FPGA register
 * @param spiadr Address of register
 * @param spidata Data to write
 *****************************************************************************/
static void spiWrite(uint8_t spiadr, uint16_t spidata)
{
  spiAccess(spiadr, 0, spidata);
}

/**************************************************************************//**
 * @brief  Performs USART2 SPI read from FPGA register
 * @param spiadr Address of register
 * @param spidata Dummy data
 *****************************************************************************/
static uint16_t spiRead(uint8_t spiadr, uint16_t spidata)
{
  return spiAccess(spiadr, 1, spidata);
}

/**************************************************************************//**
 * @brief  Initializes DVK register access
 *****************************************************************************/
void DVK_init(void)
{
  spiInit();
}

/**************************************************************************//**
 * @brief  Perform read from DVK board control register
 * @param  addr Address of register to read from
 *****************************************************************************/
uint16_t DVK_readRegister(uint16_t *addr)
{
  uint16_t data;

  if (addr != lastAddr)
  {
    spiWrite(0x00, 0xFFFF & ((uint32_t) addr));             /*LSBs of address*/
    spiWrite(0x01, 0xFF & ((uint32_t) addr >> 16));         /*MSBs of address*/
    spiWrite(0x02, (0x0C000000 & (uint32_t) addr) >> 26);   /*Chip select*/
  }
  /* Read twice */
  data     = spiRead(0x03, 0);
  data     = spiRead(0x03, 0);
  lastAddr = addr;
  return data;
}

/**************************************************************************//**
 * @brief  Perform write to DVK board control register
 * @param addr Address of register to write to
 * @param data 16-bit to  write into register
 *****************************************************************************/
void DVK_writeRegister(uint16_t *addr, uint16_t data)
{
  if (addr != lastAddr)
  {
    spiWrite(0x00, 0xFFFF & ((uint32_t) addr));             /*LSBs of address*/
    spiWrite(0x01, 0xFF & ((uint32_t) addr >> 16));         /*MSBs of address*/
    spiWrite(0x02, (0x0C000000 & (uint32_t) addr) >> 26);   /*Chip select*/
  }
  spiWrite(0x03, data);                                     /*Data*/
  lastAddr = addr;
}
