/*! \file mmc_spi.c \brief MMC/SD-Functions */
//###########################################################
///	\defgroup MMC MMC/SD-Functions (mmc_spi.c)
///	\ingroup MMC
///	\code #include "mmc_spi.h" \endcode
///	\code #include "dos.h" \endcode
///	\par Uebersicht
//###########################################################
// File: mmc_spi.c
//
// Read-/Writeroutines for MMC MultiMedia cards and
// SD SecureDigital cards in SPI mode.
//
// This will work only for cards with 512 bytes block length !
// SD Cards with 4GB that are not SDHC cards may not work !
// MMC Cards above 1GB may not work !
//
// 22.07.2007 Added support for SPI1/SSP module of LPC213x.
//
// 04.07.2007 2GB SD cards work 
//
// 28.06.2007 Started Support for 2GB and non SDHC 4GB cards
//
// 26.04.2007 Toshiba 4GB SDHC works !
//     
// 01.04.2007 Started SDHC support.
//
// 30.12.2006 Started ARM support.
//
// 04.09.2006 Made new MMC_IO_Init(). Removed IO pin settings from
//            MMCIdentify(). Why ? See comments above MMC_IO_Init(). 
//
// 20.06.2006 Store value for SPI speed and switch to higher speed
//            for Read/WriteSector(). Restore SPI speed at end of
//            routines.
//
// 29.09.2004 split SPI_WRITE() into SPI_WRITE() and SPI_WAIT()
//            speeds up program because CPU can do something else
//            while SPI hardware module is shifting in/out data 
//            see MMCReadSector() and MMCWriteSector()
//
// Benutzung auf eigene Gefahr !
//
// Use at your own risk !
//
//#########################################################################
// Last change: 22.07.2007
//#########################################################################
// hk@holger-klabunde.de
// http://www.holger-klabunde.de/index.html
//#########################################################################
// Compiler: WinARM 4.1.1
//#########################################################################
//@{

#include "mmc_spi.h"
#include "dos.h"
#include "dosdefs.h"

#include "typedefs.h" // ARM specific types
#if defined (MMC_DEBUG_IDENTIFY) || defined (MMC_DEBUG_SECTORS) || defined (MMC_DEBUG_COMMANDS)
#include "rprintf.h"  // For debug only
#endif

#ifdef MMC_CARD_SPI

unsigned char card_capacity; // Standard for MMC/SD or High for SDHC
U32 maxsect; // last sector on drive
unsigned char spi_back_byte; // gives back a byte after each SPI_WAIT();

//######################################################
/*!\brief Send commands to MMC/SD
 * \param		command	Command code
 * \param		adress	Sector adress
 * \return 		Response of MMC/SD
 *
 */
unsigned char MMCCommand(unsigned char command, unsigned long adress)
//######################################################
{
	unsigned char timeout;

#ifdef MMC_DEBUG_COMMANDS
	printf("Cmd %02u:",(U16)command-0x40);
#endif

	// MMC_CS_ON(); // Write the dummy byte with CS high. My cards don't need it,
	// but some datasheets say CS=High for dummy byte

	SPI_WRITE(DUMMY_WRITE); // This dummy write is necessary for most SD cards !
	SPI_WAIT();

	MMC_CS_OFF();

	SPI_WRITE(command);
	SPI_WAIT();
	SPI_WRITE((unsigned char)((adress & 0xFF000000)>>24)); // MSB of adress
	SPI_WAIT();
	SPI_WRITE((unsigned char)((adress & 0x00FF0000)>>16));
	SPI_WAIT();
	SPI_WRITE((unsigned char)((adress & 0x0000FF00)>>8));
	SPI_WAIT();
	SPI_WRITE((unsigned char)(adress & 0x000000FF)); // LSB of adress
	SPI_WAIT();
	SPI_WRITE(0x95); // Checksum for CMD0 GO_IDLE_STATE and dummy checksum for other commands
	SPI_WAIT();

	timeout = 255;

	//wait for response
	do
	{
		SPI_WRITE(DUMMY_WRITE);
		SPI_WAIT();

#ifdef MMC_DEBUG_COMMANDS
		printf(" 0x%02X",(U16)spi_back_byte);
#endif
		timeout--;
		if (timeout == 0)
			break; // no response
	} while (spi_back_byte == DUMMY_WRITE);

#ifdef MMC_DEBUG_COMMANDS
	printf("\n");
#endif

	return spi_back_byte;
}

//######################################################
/*!\brief Read a sector from MMC/SD
 * \param		sector	Actual sector number
 * \param		buf	Buffer for data
 * \return 		0 if successfull
 */
unsigned char MMCReadSector(unsigned long sector, unsigned char *buf)
//######################################################
{
	U32 i;

	unsigned char by;
	unsigned long startadr;
	unsigned char tmpSPSR;

#ifdef MMC_DEBUG_SECTORS
	printf("RS %lu 0x%08lX\n",sector,sector);
#endif

	// IOSET0 = P0_16; // for testing only

	if (sector >= maxsect)
		return 1; //sectornumber too big

	tmpSPSR = SPI_SPEED_REGISTER; // Store last setting
	SPI_SPEED_REGISTER = SPI_PRESCALER; // Switch to high speed

	// MMC_CS_OFF(); // moved to MMCCommand()

	//calculate startadress of the sector
	startadr = sector; // this is the blockadress for SDHC
	if (card_capacity == STANDARD_CAPACITY)
		startadr *= BYTE_PER_SEC; // SD/MMC This will work only up to 4GB

	by = MMCCommand(MMC_READ_BLOCK, startadr);

	while (by != START_BLOCK_TOKEN)
	{
		SPI_WRITE(DUMMY_WRITE);
		SPI_WAIT();
		by = spi_back_byte; // wait for card response

#ifdef MMC_DEBUG_COMMANDS
		// no way to come out of this :( skip this sector !?
		if(by==0x01) // ERROR !

		{
			// One of my SD cards sends this error. My cardreader has also
			// problems to read (NOT write !) the card completely. Trash.
			printf("\nRead error 0x01 at sector %lu !\n",sector);

			MMC_CS_ON();
			SPI_SPEED_REGISTER = tmpSPSR; // Restore old speed setting

			// data buffer is not valid at this point !
			return 1;
		}
#endif      
	}

	//----------------------------------------------------------------
#ifdef STANDARD_SPI_READ

	i=BYTE_PER_SEC; // This routine is a little bit faster as for()
	while(i)
	{
		SPI_WRITE(DUMMY_WRITE); // start shift in next byte
		i--; // dec loop counter while SPI shifts in
		SPI_WAIT(); // wait for next byte
		*buf++ = spi_back_byte; // store byte in buffer
	}

	SPI_WRITE(DUMMY_WRITE); // shift in crc part1
	SPI_WAIT();
	SPI_WRITE(DUMMY_WRITE); // shift in crc part2
	SPI_WAIT();
#endif // STANDARD_SPI_READ
	//----------------------------------------------------------------

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef FAST_SPI_READ
	// The following code looks very strange !
	// The idea is not to stop the cpu while SPI module transfers data.
	// You have some cpu cycles until transmission has finished !
	// You can use this time to do something like storing your last data
	// or get your next data out of memory, doing some loop overhead.....
	// Don't wait for end of transmission until you have done something better ;)

	SPI_WRITE(DUMMY_WRITE); // shift in first byte
	SPI_WAIT(); // we have to wait for the first byte, but ONLY for the first byte
	by = spi_back_byte; // get first byte, but store later !

	SPI_WRITE(DUMMY_WRITE); // start shift in next byte

	i = BYTE_PER_SEC - 1;
	while (i) //execute the loop while transmission is running in background
	{
		*buf++ = by; // store last byte in buffer while SPI module shifts in new data
		i--;
		SPI_WAIT(); // wait for next byte
		by = spi_back_byte; // get next byte, but store later !
		SPI_WRITE(DUMMY_WRITE); // start shift in next byte
		// do the for() loop overhead at this point while SPI module shifts in new data
	}

	// last SPI_WRITE(DUMMY_WRITE); is shifting in crc part1 at this point
	*buf = by; // store last byte in buffer while SPI module shifts in crc part1
	SPI_WAIT();
	SPI_WRITE(DUMMY_WRITE); // shift in crc part2
	SPI_WAIT();

#endif //FAST_SPI_READ
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	MMC_CS_ON();

	SPI_SPEED_REGISTER = tmpSPSR; // Restore old setting

	// IOCLR0 = P0_16;

	return 0;
}

#ifdef DOS_WRITE
//######################################################
/*!\brief Write a sector to MMC/SD
 * \param		sector	Actual sector number
 * \param		buf	Buffer for data
 * \return 		0 if successfull
 */
unsigned char MMCWriteSector(unsigned long sector, unsigned char *buf)
//######################################################
{
	U32 i;

	unsigned char by;
	unsigned long startadr;
	unsigned char tmpSPSR;

#ifdef MMC_DEBUG_SECTORS
	printf("WS %lu 0x%08X\n",sector,sector);
#endif

	if (sector >= maxsect)
		return 1; //sectornumber too big

	tmpSPSR = SPI_SPEED_REGISTER; // Store last speed setting
	SPI_SPEED_REGISTER = SPI_PRESCALER; // Switch to high speed

	// MMC_CS_OFF(); // moved to MMCCommand()

	//calculate startadress
	startadr = sector; // this is the blockadress for SDHC
	if (card_capacity == STANDARD_CAPACITY)
		startadr *= BYTE_PER_SEC; // SD/MMC This will work only up to 4GB

	MMCCommand(MMC_WRITE_BLOCK, startadr);

	//----------------------------------------------------------------
#ifdef STANDARD_SPI_WRITE
	SPI_WRITE(START_BLOCK_TOKEN); // start block token for next sector
	SPI_WAIT();

	i=BYTE_PER_SEC;
	while(i)
	{
		SPI_WRITE(*buf++); // shift out next byte
		i--; // dec loop counter while SPI shifts out

#ifdef USE_SPI0
		SPI_WAIT_NO_BACK(); // wait for end of transmission
#endif

#ifdef USE_SPI1
		SPI_WAIT(); // wait for end of transmission
#endif
	}

#endif //STANDARD_SPI_WRITE
	//----------------------------------------------------------------

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef FAST_SPI_WRITE
	SPI_WRITE(START_BLOCK_TOKEN); // start block token for next sector

	i = BYTE_PER_SEC;
	while (i) // execute the loop while transmission is running in background
	{
		// do the loop overhead at this point while SPI module shifts out new data
		by = *buf++; // get next data from memory while SPI module shifts out new data
		i--;
#ifdef USE_SPI0
		SPI_WAIT_NO_BACK(); // wait for end of transmission
#endif

#ifdef USE_SPI1
		SPI_WAIT(); // wait for end of transmission
#endif
		SPI_WRITE(by); // start shift out next byte
	}

	SPI_WAIT(); // wait til last byte is written to MMC
#endif //FAST_SPI_WRITE
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	SPI_WRITE(DUMMY_WRITE); // 16 bit crc follows data
	SPI_WAIT();
	SPI_WRITE(DUMMY_WRITE);
	SPI_WAIT();

	SPI_WRITE(DUMMY_WRITE); // read response
	SPI_WAIT();
	by = spi_back_byte & 0x1F;
	if (by != 0x05) // data block accepted ?
	{
#ifdef MMC_DEBUG_COMMANDS
		printf("\nWrite error at sector %lu !\n",sector);
#endif      

		MMC_CS_ON();
		SPI_SPEED_REGISTER = tmpSPSR; // Restore old speed setting
		return 1;
	}

	do
	{
		SPI_WRITE(DUMMY_WRITE);
		SPI_WAIT();
	} while (spi_back_byte == 0x00); // wait til busy is gone

	MMC_CS_ON();

	SPI_SPEED_REGISTER = tmpSPSR; // Restore old speed setting

	return 0;
}
#endif //DOS_WRITE
//######################################################
/*!\brief Initialise io pin settings for MMC/SD
 * \return 		Nothing
 *
 * Removed this from MMCIdentify().
 * Maybe some other devices are connected to SPI bus.
 * All chip select pins of these devices should have
 * high level before starting SPI transmissions !
 * So first call MMC_IO_Init(), after that for example
 * VS1001_IO_Init(), SPILCD_IO_Init() and AFTER that MMCIdentify().
 */
void MMC_IO_Init(void)
//######################################################
{
	// make IO pin settings
	SPI_IODIR |= (1 << SPI_SCK_PIN) | (1 << SPI_MOSI_PIN) | (1 << SPI_SS_PIN);
	SPI_IODIR &= ~(1 << SPI_MISO_PIN);

	MMC_CS_ON();

	SPI_PINSEL &= ~(SPI_PINSEL_CLEAN_MASK);
	SPI_PINSEL |= SPI_PINSEL_SET_MASK;

#ifdef USE_SPI0
	SPI_CTRL_REGISTER = 0x00; //Disable SPI
#endif

#ifdef USE_SPI1
	SPI_CTRL_REGISTER0 = 0x0007; //SPI 8 Bit Transfer
	SPI_CTRL_REGISTER1 = 0x00; //Disable SPI, SPI Master
#endif

}

//######################################################
/*!\brief Get most important informations from MMC/SD
 * \return 		Nothing til now
 *
 * This function enables SPI transfers and determines
 * size of MMC/SD Card 
 */
unsigned char MMCIdentify(void)
//######################################################
{
	U8 by, card_type;
	U16 i;
	U16 c_size_mult;
	U32 c_size; // now has 22 bits
	U16 read_bl_len;
	U8 csd_version;
	U8 response[16];

	// Init SPI with a very slow transfer rate first ! <400kHz

	SPI_SPEED_REGISTER = 254;

#ifdef USE_SPI0
	SPI_CTRL_REGISTER = (1<<MSTR);
#endif

#ifdef USE_SPI1
	SPI_CTRL_REGISTER0 = (8 - 1) | (0x20 << SCR); // 8Bit Transfer, SCR=0x20+1
	SPI_CTRL_REGISTER1 = (1 << SSE); // Enable SPI
#endif

	// give min 74 SPI clock pulses before sending commands
	i = 10;
	while (i)
	{
		SPI_WRITE(DUMMY_WRITE);
		SPI_WAIT();
		i--;
	}

	// MMC_CS_OFF(); // moved to MMCCommand()

#ifdef MMC_DEBUG_IDENTIFY
	printf("Send CMD0\n");
#endif

	// Some of my cards give 0x01 response only after a few retrys of CMD0
	for (i = 0; i < 25; i++)
	{
		//send CMD0 for RESET
		by = MMCCommand(MMC_CMD0, 0);
		if (by == IN_IDLE_STATE)
			break;
	}

#ifdef MMC_DEBUG_CMD0_TIMEOUT
	printf("CMD0 retrys %u\n", (U16)i);
#endif

	if (by != IN_IDLE_STATE)
	{
		MMC_CS_ON();
		return CMD0_TIMEOUT;
	} // no response from card

#ifdef MMC_DEBUG_IDENTIFY
	printf("Send CMD8\n");
#endif

	card_capacity = STANDARD_CAPACITY; // assume standard capacity
	card_type = UNKNOWN_CARD;

	// CMD8 has to be send for SDHC cards to expand ACMD41 functions
	// Check pattern = 0x22 Bits 7..0 (calculated for crc=0x95 !)
	// VHS Voltage supplied 2.7-3.6V 0b0001 Bits 11..8
	c_size = (unsigned long) 0x00000122; // reuse c_size here ;)

	// SEND_IF_COND
	if ((MMCCommand(SD_CMD8, c_size) & ILLEGAL_COMMAND) == 0) // SD card version 2.00
	{
		GetResponse(response, 4); //geht four more bytes of response3

		card_type = SD_CARD; // MMC don't know CMD8  !

		if (response[2] != 0x01) // test voltage range failed
		{
#ifdef MMC_DEBUG_IDENTIFY
			printf("Test Voltage range failed 0x%02X\n",(U16)response[2]);
#endif
			MMC_CS_ON();
			return TEST_VOLTAGE_ERROR;
		}

		if (response[3] != 0x22) // test check pattern failed
		{
#ifdef MMC_DEBUG_IDENTIFY
			printf("Test pattern failed 0x%02X\n",(U16)response[3]);
#endif
			MMC_CS_ON();
			return TEST_PATTERN_ERROR;
		}

		// begin: Think we don't need this !
#ifdef MMC_DEBUG_IDENTIFY
		printf("Send CMD58\n");
#endif

		MMCCommand(SD_CMD58, 0); // READ_OCR
		GetResponse(response, 4); //geht four more bytes of response3
		// end: Think we don't need this !

		i = 0;
		while (1) // repeat ACMD41 until card is ready
		{
#ifdef MMC_DEBUG_IDENTIFY
			printf("Send CMD55\n");
#endif
			MMCCommand(SD_CMD55, 0); // SEND_APP_CMD

#ifdef MMC_DEBUG_IDENTIFY
			printf("Send ACMD41\n");
#endif
			if (MMCCommand(SD_ACMD41, 0x40000000) == 0)
				break; // send with HCS bit set
			if (i++ > 1024)
			{
				MMC_CS_ON();
				return ACMD41_TIMEOUT;
			} // no response from card
		}

#ifdef MMC_DEBUG_CMD1_TIMEOUT
		printf("ACMD41 retrys %u\n", (U16)i);
#endif

#ifdef MMC_DEBUG_IDENTIFY
		printf("Send CMD58\n");
#endif
		// check for high capacity card now
		by = MMCCommand(SD_CMD58, 0); // READ OCR
		GetResponse(response, 4); //geht four more bytes of response3

		if (response[0] & 0x40)
		{
			card_capacity = HIGH_CAPACITY; // high capacity card if bit 30 of OCR is set
			card_type = SDHC_CARD;
#ifdef MMC_DEBUG_IDENTIFY
			printf("High capacity card\n");
#endif
		}
	}
	else // SD card V1.xx or MMC
	{
#ifdef MMC_DEBUG_IDENTIFY
		printf("CMD8 illegal command\n");
#endif

		// Note: CMD1 is not supported by all SD cards ?
		// Thin 1,4mm cards don't accept CMD1 before sending ACMD41
		// Try ACMD41 first here !
#ifdef MMC_DEBUG_IDENTIFY
		printf("Send CMD55\n");
#endif

		// SEND_APP_CMD
		if ((MMCCommand(SD_CMD55, 0) & ILLEGAL_COMMAND) == 0) // SD card V1.xx
		{
			card_type = SD_CARD; // MMC don't know CMD55  !

			i = 0;
			while (1) // repeat ACMD41 until card is ready
			{
#ifdef MMC_DEBUG_IDENTIFY
				printf("Send ACMD41\n");
#endif
				if (MMCCommand(SD_ACMD41, 0) == 0)
					break;
				if (i++ > 1024)
				{
					MMC_CS_ON();
					return ACMD41_TIMEOUT;
				} // no response from card

#ifdef MMC_DEBUG_IDENTIFY
				printf("Send CMD55\n");
#endif
				MMCCommand(SD_CMD55, 0); // Repeat SEND_APP_CMD
			}

#ifdef MMC_DEBUG_CMD1_TIMEOUT
			printf("ACMD41 retrys %u\n", (U16)i);
#endif

		}
		else // MMC card
		{
#ifdef MMC_DEBUG_IDENTIFY
			printf("CMD55 illegal command\n");
#endif

			card_type = MMC_CARD;
			// Repeat CMD1 til result=0

			i = 0;
			while (1)
			{
#ifdef MMC_DEBUG_IDENTIFY
				printf("Send CMD1\n");
#endif
				if (MMCCommand(MMC_CMD1, 0) == 0)
					break;
				if (i++ > 1024)
				{
					MMC_CS_ON();
					return CMD1_TIMEOUT;
				} // no response from card
			}

#ifdef MMC_DEBUG_CMD1_TIMEOUT
			printf("CMD1 retrys %u\n", (U16)i);
#endif

		}// if((MMCCommand(SD_CMD55,0) & ILLEGAL_COMMAND) == 0 ) // SD card version 1.00

	}// if((MMCCommand(SD_CMD8,c_size) & ILLEGAL_COMMAND) == 0 ) // SD card version 2.00

	// Read CID
	// MMCCommand(MMC_READ_CID,0); // nothing really interesting here

#ifdef MMC_DEBUG_IDENTIFY
	printf("Read CSD\n");
#endif

	// Read CSD Card Specific Data
	by = MMCCommand(MMC_READ_CSD, 0);

	while (by != START_BLOCK_TOKEN)
	{
		SPI_WRITE(DUMMY_WRITE);
		SPI_WAIT();
		by = spi_back_byte; // Wait for card response
	}

	GetResponse(response, 16); //CSD has 128 bits -> 16 bytes

	SPI_WRITE(DUMMY_WRITE); // 16 bit crc follows data
	SPI_WAIT();
	SPI_WRITE(DUMMY_WRITE);
	SPI_WAIT();

	MMC_CS_ON();

#ifdef MMC_DEBUG_IDENTIFY
	printf("CSD done\n");
#endif
	// Here comes the hard stuff !
	// Calculate disk size and number of last sector
	// that can be used on your mmc/sd card
	// Be aware ! Bit 127 of CSD is shifted in FIRST.

	by = response[5] & 0x0F;
	read_bl_len = 1;
	read_bl_len <<= by;

#ifdef MMC_DEBUG_IDENTIFY
	printf("read_bl_len %u\n",read_bl_len);
	if(read_bl_len > 512)
	{

		// MMC_SET_BLOCKLEN always fails on my 2GB memorex card.
		// Response is 0xFF. But it does work without this.
		// All my other cards work too WITHOUT it. So: FORGET IT !!
		//
		// Auf deutsch: MMC_SET_BLOCKLEN braucht niemand !
		// Geht komplett ohne bei allen meinen Karten.
		//
		/*
		 // Set blocklen to 512 bytes if >512 bytes
		 by=MMCCommand(MMC_SET_BLOCKLEN,512);
		 if(by != 0)
		 {
		 #ifdef MMC_DEBUG_IDENTIFY
		 printf("SetBlocklen failed ! Resp=0x%02X\n",(int)by);
		 #endif
		 return SET_BLOCKLEN_ERROR; // Set blocklen failed
		 }
		 */
	}
#endif

	// CSDVERSION = 0, MMC and Version 1.0/2.0 SD Standard Capacity Cards
	// CSDVERSION = 1, SDHC and Version 2.0 SD Cards

	if (card_type == MMC_CARD)
	{
		csd_version = 0;
	}
	else // SD, SDHC
	{
		csd_version = response[0] >> 6;
	}

#ifdef MMC_DEBUG_IDENTIFY
	by = response[0] >> 6;
	printf("CSD_STRUCT %u\n",(U16)by);
	by = (response[0] >> 2) & 0x0F;
	printf("SPEC_VERSION %u\n",(U16)by);

	if(card_type == MMC_CARD)
	{	printf("MMC card\n");}
	else // SD / SDHC card

	{
		if(csd_version==0) printf("SD card\n");
		if(csd_version==1) printf("SDHC card\n");
	}
#endif

	if (csd_version == 2 || csd_version == 3)
	{
#ifdef MMC_DEBUG_IDENTIFY
		printf("Unknown card\n");
#endif
		return CSD_ERROR; // error
	}

	if (csd_version == 0)
	{
		//c_size has 12 bits
		c_size = response[6] & 0x03; //bits 1..0
		c_size <<= 10;
		c_size += (U16) response[7] << 2;
		c_size += response[8] >> 6;

		by = response[9] & 0x03;
		by <<= 1;
		by += response[10] >> 7;

		c_size_mult = 1;
		c_size_mult <<= (2 + by);

		//   drive_size=(unsigned long)(c_size+1) * (unsigned long)c_size_mult * (unsigned long)read_bl_len;
		//   maxsect= drive_size / BYTE_PER_SEC;

		maxsect = read_bl_len / BYTE_PER_SEC;
		maxsect *= (unsigned long) (c_size + 1) * (unsigned long) c_size_mult;
	}
	else
	// if(csd_version==1)
	{
		//c_size has 22 bits
		maxsect = 1024; // c_size is a multiple of 512kB -> 1024 sectors
		c_size = response[7] & 0x3F; // 6 lower bits from here
		c_size <<= 16;
		c_size += (U16) response[8] << 8;
		c_size += response[9];
		maxsect *= (c_size + 1);
		c_size_mult = 0; // to remove compiler warnings only
	}

#ifdef MMC_DEBUG_IDENTIFY
	printf("c_size %lu , c_size_mult %u\n",c_size, c_size_mult);
	printf("DriveSize %lu kB , maxsect %lu\n",(maxsect>>1), maxsect);
#endif

	// Switch to high speed SPI
#ifdef USE_SPI1
	SPI_CTRL_REGISTER1 &= ~(1 << SSE); // Disable SPI
	SPI_CTRL_REGISTER0 = (8 - 1); // 8Bit Transfer, SCR=0x0+1
	SPI_CTRL_REGISTER1 |= (1 << SSE); // Enable SPI
#endif

	SPI_SPEED_REGISTER = SPI_PRESCALER;

#ifdef MMC_DEBUG_IDENTIFY
	printf("mmc_init() ok\n");
#endif

	return MMC_OK;
}

//######################################################
void GetResponse(U8 *buf, U8 numbytes)
//######################################################
{
	unsigned char i;

	i = numbytes;

	while (i)
	{
		SPI_WRITE(DUMMY_WRITE);
		SPI_WAIT();
		*buf++ = spi_back_byte;

#ifdef MMC_DEBUG_IDENTIFY
		printf(" 0x%02X",(U16)spi_back_byte);
#endif
		i--;
	}

#ifdef MMC_DEBUG_IDENTIFY
	printf("\n");
#endif
}

#endif //MMC_CARD_SPI
//@}
