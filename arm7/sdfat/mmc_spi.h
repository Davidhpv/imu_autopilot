/*! \file "mmc_spi.h" \brief MMC/SD-Definitions */
///	\ingroup MMC
///	\defgroup MMC MMC/SD-Functions (mmc_spi.h)
//#########################################################################
// File: mmc_spi.h
//
// MMC MultiMediaCard and SD SecureDigital definitions for SPI protocol
//
// Benutzung auf eigene Gefahr !
//
// Use at your own risk !
//
// 22.07.2007  Added support for SPI1/SSP module of LPC213x.
//
//#########################################################################
// Last change: 22.07.2007
//#########################################################################
// Compiler: WinARM 4.1.1
//#########################################################################
//@{

#ifndef __MMC_CARD_SPI_H
#define __MMC_CARD_SPI_H

#include "LPC21xx.h"
#include "conf.h"

//If use MMC-Card then set to 1
#define	USE_MMC 			1

#ifdef USE_MMC
 #define MMC_CARD_SPI
#endif


// Use ONE of these definitions only !
// #define USE_SPI0  //SPI0 Module, slow, max. 7.5MHz SPI at 60MHz speed
 #define USE_SPI1  //SPI1 SSP Module, fast, max. 30MHz SPI at 60MHz speed


// Use ONE of these definitions only !
//#define STANDARD_SPI_READ  // No optimizations   
#define FAST_SPI_READ      // Optimizations

// Use ONE of these definitions only !
//#define STANDARD_SPI_WRITE  // No optimizations   
#define FAST_SPI_WRITE      // Optimizations

//#define MMC_DEBUG_IDENTIFY //activate debug output for MMCIdentify() via printf()
//#define MMC_DEBUG_SECTORS //activate debug output via printf()
//#define MMC_DEBUG_COMMANDS //activate debug output via printf()
//#define MMC_DEBUG_CMD0_TIMEOUT //activate debug output via printf()
//#define MMC_DEBUG_CMD1_TIMEOUT //activate debug output via printf()

 #include "typedefs.h" // ARM specific types

#include "conf.h"

 #ifdef USE_SPI0

  #define SPI_PRESCALER		8 // 8..254

  #define SPI_CTRL_REGISTER	S0SPCR
  #define SPI_SPEED_REGISTER	S0SPCCR
  #define SPI_DATA_REGISTER	S0SPDR
  #define SPI_STATUS_REGISTER	S0SPSR

  #define SPIF		7
  #define MSTR		5

  #define SPI_WRITE(a)	 	{ SPI_DATA_REGISTER=(a); }
  #define SPI_WAIT()		{ while(!(SPI_STATUS_REGISTER & ( 1<<SPIF ))); spi_back_byte=SPI_DATA_REGISTER; }
 // SPI0 has no FIFO. So if you are writing, it's not necessary to read SPI_STATUS_REGISTER
  #define SPI_WAIT_NO_BACK()	{ while(!(SPI_STATUS_REGISTER & ( 1<<SPIF ))); }

  #define SPI_PINSEL		PINSEL0
  #define PINSEL_SCK		8
  #define PINSEL_MISO		10
  #define PINSEL_MOSI		12
  #define PINSEL_SS		14
  #define SPI_PINSEL_CLEAN_MASK	(3<<PINSEL_SCK) | (3<<PINSEL_MISO) | (3<<PINSEL_MOSI) | (3<<PINSEL_SS) // clean old MISO,MOSI,SCK,SS definitions
  #define SPI_PINSEL_SET_MASK	(1<<PINSEL_SCK) | (1<<PINSEL_MISO) | (1<<PINSEL_MOSI) // set new MISO,MOSI,SCK,SS definitions

  #define SPI_IODIR		IODIR0
  #define SPI_SCK_PIN		4
  #define SPI_MISO_PIN		5
  #define SPI_MOSI_PIN		6
  #define SPI_SS_PIN		7

 #endif //#ifdef USE_SPI0

 #ifdef USE_SPI1

  #define SPI_PRESCALER		4 // 2..254, 2 is to fast @60MHz !

  #define SPI_CTRL_REGISTER0	SSPCR0
  #define SPI_CTRL_REGISTER1	SSPCR1
  #define SPI_SPEED_REGISTER	SSPCPSR
  #define SPI_DATA_REGISTER	SSPDR
  #define SPI_STATUS_REGISTER	SSPSR

  #define TNF			1
  #define RNE			2
  #define BSY			4

  #define SSE			1
  #define MS			2
  #define SCR			8

//  #define SPI_WRITE(a)	 	{ while( !(SPI_STATUS_REGISTER & (1<<TNF)) ); SPI_DATA_REGISTER=(a); }
  #define SPI_WRITE(a)	 	{ SPI_DATA_REGISTER=(a); }
//  #define SPI_WAIT()		while( SPI_STATUS_REGISTER & (1<<BSY) ){};
  #define SPI_WAIT()		{ while( !(SPI_STATUS_REGISTER & (1<<RNE)) ); spi_back_byte=SPI_DATA_REGISTER; }

  #define SPI_PINSEL		PINSEL1
  #define PINSEL_SCK		2
  #define PINSEL_MISO		4
  #define PINSEL_MOSI		6
  #define PINSEL_SS		8
  #define SPI_PINSEL_CLEAN_MASK	(3<<PINSEL_SCK) | (3<<PINSEL_MISO) | (3<<PINSEL_MOSI) | (3<<PINSEL_SS) // clean old MISO,MOSI,SCK,SS definitions
  #define SPI_PINSEL_SET_MASK	(2<<PINSEL_SCK) | (2<<PINSEL_MISO) | (2<<PINSEL_MOSI) // set new MISO,MOSI,SCK,SS definitions

  #define SPI_IODIR		IO0DIR
  #define SPI_SCK_PIN		17
  #define SPI_MISO_PIN		18
  #define SPI_MOSI_PIN		19
  #define SPI_SS_PIN		SDCARD_SPI_SS_PIN

 #endif //#ifdef USE_SPI1

 #define MMC_CS_OFF()    (IO0CLR = (1<<SPI_SS_PIN)) //CS auf Low
 #define MMC_CS_ON()     (IO0SET = (1<<SPI_SS_PIN))  //CS auf High


// MMC/SD commands
#define MMC_RESET		(unsigned char)(0x40 + 0)
#define MMC_GO_IDLE_STATE	(unsigned char)(0x40 + 0)
#define MMC_INIT		(unsigned char)(0x40 + 1)
#define MMC_CMD0		(U8)(0x40 + 0)
#define MMC_CMD1		(U8)(0x40 + 1)
#define SD_CMD8			(U8)(0x40 + 8)
#define MMC_READ_CSD		(unsigned char)(0x40 + 9)
#define MMC_READ_CID		(unsigned char)(0x40 + 10)
#define MMC_STOP_TRANSMISSION	(unsigned char)(0x40 + 12)
#define MMC_SEND_STATUS		(unsigned char)(0x40 + 13)
#define MMC_SET_BLOCKLEN	(unsigned char)(0x40 + 16)
#define MMC_READ_BLOCK		(unsigned char)(0x40 + 17)
#define MMC_READ_MULTI_BLOCK	(unsigned char)(0x40 + 18)
#define MMC_WRITE_BLOCK		(unsigned char)(0x40 + 24)
#define MMC_WRITE_MULTI_BLOCK	(unsigned char)(0x40 + 25)
#define SD_CMD55		(U8)(0x40 + 55)
#define SD_CMD58		(U8)(0x40 + 58)
#define SD_ACMD41		(U8)(0x40 + 41)

#define DUMMY_WRITE		(unsigned char)(0xFF)
#define START_BLOCK_TOKEN	(unsigned char)(0xFE)

#ifndef BYTE_PER_SEC
 #define BYTE_PER_SEC (U16) 512
#endif 

// SPI Response Flags
#define IN_IDLE_STATE	   (U8)(0x01)
#define ERASE_RESET	   (U8)(0x02)
#define ILLEGAL_COMMAND	   (U8)(0x04)
#define COM_CRC_ERROR	   (U8)(0x08)
#define ERASE_ERROR	   (U8)(0x10)
#define ADRESS_ERROR	   (U8)(0x20)
#define PARAMETER_ERROR	   (U8)(0x40)


#define STANDARD_CAPACITY  (U8)0
#define HIGH_CAPACITY      (U8)1

#define UNKNOWN_CARD	(U8)0
#define MMC_CARD	(U8)1
#define SD_CARD		(U8)2
#define SDHC_CARD	(U8)3

//Returncodes for MMCIdentify()
#define MMC_OK			(U8)0
#define CMD0_TIMEOUT		(U8)1
#define CMD1_TIMEOUT		(U8)2
#define ACMD41_TIMEOUT		(U8)3
#define CSD_ERROR		(U8)4
#define TEST_PATTERN_ERROR	(U8)5
#define TEST_VOLTAGE_ERROR	(U8)6
#define SET_BLOCKLEN_ERROR	(U8)7

//prototypes
extern unsigned char spi_back_byte; // gives back a byte after each SPI_WAIT();
extern U32 maxsect;           // last sector on drive

extern unsigned char MMCCommand(unsigned char command, unsigned long adress);
extern unsigned char MMCReadSector(unsigned long sector, unsigned char *buf);
extern unsigned char MMCWriteSector(unsigned long sector, unsigned char *buf);
extern unsigned char MMCIdentify(void);
extern void MMC_IO_Init(void);
extern void GetResponse(U8 *buf, U8 numbytes);

#define ReadSector(a,b) 	MMCReadSector((a),(b))
#define WriteSector(a,b) 	MMCWriteSector((a),(b))
#define IdentifyMedia()		MMCIdentify()

#define mmc_read_sector(a,b)    MMCReadSector((a),(b))
#define mmc_write_sector(a,b)   MMCWriteSector((a),(b))

// security checks !
#if defined (STANDARD_SPI_WRITE) && defined (FAST_SPI_WRITE)
 #error "Define STANDARD_SPI_WRITE or FAST_SPI_WRITE only in mmc_spi.h, NOT both !"
#endif

#if !defined (STANDARD_SPI_WRITE) && !defined (FAST_SPI_WRITE)
 #error "Define at least STANDARD_SPI_WRITE or FAST_SPI_WRITE in mmc_spi.h !"
#endif

#if defined (STANDARD_SPI_READ) && defined (FAST_SPI_READ)
 #error "Define STANDARD_SPI_READ or FAST_SPI_READ only in mmc_spi.h, NOT both !"
#endif

#if !defined (STANDARD_SPI_READ) && !defined (FAST_SPI_READ)
 #error "Define at least STANDARD_SPI_READ or FAST_SPI_READ in mmc_spi.h !"
#endif

#if defined (USE_SPI0) && defined (USE_SPI1)
 #error "Define USE_SPI0 or USE_SPI1 only in mmc_spi.h, NOT both !"
#endif

#if !defined (USE_SPI0) && !defined (USE_SPI1)
 #error "Define at least USE_SPI0 or USE_SPI1 in mmc_spi.h !"
#endif

#endif
//@}
