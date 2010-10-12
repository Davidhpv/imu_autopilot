//#######################################################################################
//
// 2MB große Dateien auf den CF schreiben und wieder lesen.
// Jeweils mit unterschiedlichen Puffergrößen. Zeit messen und anzeigen.
//
// Dies ist meine am häufigsten benutzte Testroutine.
//
// hk@holger-klabunde.de
// www.holger-klabunde.de
//#######################################################################################

#include <lpc213x.h>
#include "main.h"

#include "enc28j60.h" // You don't need this ! It's for my board only.

#define TEST_FILE_SIZE	(unsigned long)2000 * (unsigned long)1024  // test filesize 2MB

#define BUFFER_SIZE	8192

void ReadTestFile(char *name, unsigned char *buf, U16 bufsize);
void WriteTestFile(char *name, unsigned char *buf, U16 bufsize);
unsigned char VerifyTestFile(char *name, unsigned char *buf, U16 bufsize);

void SystemInit(void);
void ShowDOSConfig(void);
void HaltCPU(void);
void StartTimer(void);
void StopTimer(void);

U32 timer_value;
U32 start_timer,stop_timer;

/* *** MAIN ************************************************************* */

int main(void)
{
 static unsigned char buffer[BUFFER_SIZE];
 unsigned short i;

 unsigned char timeout = 3;

 RTCTime local_time;

 SystemInit();

// IODIR0 |= (P0_15|P0_16|P0_17); // some outputs for debugging and measures
//P0_16 ReadSector() in mmc_spi.c
//P0_17 enc28j60 write packet
//P0_15 general purpose, BE AWARE !

// uart1Init(B38400, UART_8N1, UART_FIFO_8); // setup the UART
 uart1Init(B115200, UART_8N1, UART_FIFO_8); // setup the UART
 rprintf_devopen( uart1_sendchar ); /* init rprintf */

 VICIntEnClear = 0xFFFFFFFF;
 VICIntSelect = 0x00000000;

 RTCInit();
/* current date 05.08.2007 20:45:00*/
 local_time.RTC_Sec = 0;
 local_time.RTC_Min = 45;
 local_time.RTC_Hour = 20;
 local_time.RTC_Mday = 5;
 local_time.RTC_Wday = 3; // My monday has 0
 local_time.RTC_Yday = 1;
 local_time.RTC_Mon = 8;
 local_time.RTC_Year = 2007;
 RTCSetTime( local_time );		/* Set local time */
// RTCStart();

 init_timer();

 enableIRQ();

 printf("Holgi's ARM7 LPC213x MultiFAT Speed Test\n\r");

 MMC_IO_Init();
 enc28j60_io_init(); // You don't need this ! It's for my board only.

 while (GetDriveInformation() != F_OK && timeout--)
  {
   printf ("MMC/SD-Card not found !\n\r");
   HaltCPU();
  }

 ShowDOSConfig(); // show selected DOS details from dosdefs.h

/*
// test time measurement
 StartTimer();
 timer_value = SetDelay10ms(1000); // delay 10 seconds
 while(CheckDelay10ms(timer_value)==0);
 StopTimer(); // stop measure and show results
// result is 2MB in 10s -> 200kB/s
// end test time measurement
*/

 printf("\nTest directory functions\n");
 Chdir("dir1");
 Remove("dir4");
 Chdir("/"); // Back to root
 Remove("dir1");

 Mkdir("dir1");
 Chdir("dir1");
 Mkdir("dir2");
 Mkdir("dir3");
 Rename("dir3","dir4");
 Remove("dir2");

 // You should have now:
 // dir1/dir4

 Chdir("/"); // Back to root

 printf("\nDeleting files\n");
 Remove("01.bin"); //Remove last data
 Remove("02.bin");
 Remove("04.bin");
 Remove("08.bin");
 Remove("16.bin");
 Remove("32.bin");
 Remove("64.bin");
 Remove("77.bin");
 Remove("128.bin");
 Remove("MAX.bin");

 //Fill the test buffer
 for(i=0; i<BUFFER_SIZE; i++) buffer[i]=(unsigned char)(i);

 printf("\nStart writing files\n");
 WriteTestFile("01.bin",buffer,1);
 WriteTestFile("02.bin",buffer,2);
 WriteTestFile("04.bin",buffer,4);
 WriteTestFile("08.bin",buffer,8);
 WriteTestFile("16.bin",buffer,16);
 WriteTestFile("32.bin",buffer,32);
 WriteTestFile("64.bin",buffer,64);
 WriteTestFile("77.bin",buffer,77);
 WriteTestFile("128.bin",buffer,128);
 WriteTestFile("MAX.bin",buffer,BUFFER_SIZE);

 printf("\nStart reading files\n");
 ReadTestFile("01.bin",buffer,1);
 ReadTestFile("02.bin",buffer,2);
 ReadTestFile("04.bin",buffer,4);
 ReadTestFile("08.bin",buffer,8);
 ReadTestFile("16.bin",buffer,16);
 ReadTestFile("32.bin",buffer,32);
 ReadTestFile("64.bin",buffer,64);
 ReadTestFile("77.bin",buffer,77);
 ReadTestFile("128.bin",buffer,128);
 ReadTestFile("MAX.bin",buffer,BUFFER_SIZE);

 printf("\nVerifying files\n");
 VerifyTestFile("32.bin",buffer,32);
 VerifyTestFile("64.bin",buffer,64);
 VerifyTestFile("77.bin",buffer,77);
 VerifyTestFile("128.bin",buffer,128);
 VerifyTestFile("MAX.bin",buffer,BUFFER_SIZE);

 printf("Test done.\n");

// test if RTC is running
     local_time=RTCGetTime();
     printf("%02u:%02u:%02u %02u.%02u.%04u\n\r",local_time.RTC_Hour,local_time.RTC_Min,local_time.RTC_Sec
                                          ,local_time.RTC_Mday,local_time.RTC_Mon,local_time.RTC_Year );
 while(1)
  {
/*
   if(CheckDelay10ms(timer_value))
    {
     timer_value = SetDelay10ms(1000); // delay 10 seconds

// test if RTC is running
     local_time=RTCGetTime();
     printf("%02u:%02u:%02u %02u.%02u.%04u\n\r",local_time.RTC_Hour,local_time.RTC_Min,local_time.RTC_Sec
                                          ,local_time.RTC_Mday,local_time.RTC_Mon,local_time.RTC_Year );
    }
*/
  }

 return(0);
}

//###################################################################################
void WriteTestFile(char *name, unsigned char *buf, U16 bufsize)
//###################################################################################
{
 unsigned long i;
 S16 fileid;

 fileid=Fopen(name,F_WRITE);
 if(fileid>=0 && fileid<MAX_OPEN_FILE)
  {
   StartTimer();
   for(i=0; i<TEST_FILE_SIZE; i+=bufsize)
    {
     if(Fwrite(buf,bufsize,fileid)!=bufsize) break;
    }
   Fclose(fileid);
   printf("% 3lu",(U32)bufsize);
   StopTimer();
  }
}

//###################################################################################
void ReadTestFile(char *name, unsigned char *buf, U16 bufsize)
//###################################################################################
{
 unsigned long i;
 S16 fileid;

 fileid=Fopen("01.TXT",F_READ);
 if(fileid>=0 && fileid<MAX_OPEN_FILE)
  {
   StartTimer();
   for(i=0; i<TEST_FILE_SIZE; i+=bufsize)
    {
     if(Fread(buf,bufsize,fileid)!=bufsize) break;
    }
   Fclose(fileid);
   printf("% 3lu",(U32)bufsize);
   StopTimer();
  }
}

//###################################################################################
unsigned char VerifyTestFile(char *name, unsigned char *buffer, U16 bufsize)
//###################################################################################
{
 S16 fileid;
 unsigned long i, k;
 unsigned char result;

 result=F_OK;

 fileid=Fopen(name,F_READ);
 if(fileid>=0 && fileid<MAX_OPEN_FILE)
  {
   for(i=0; i<TEST_FILE_SIZE; i+=bufsize)
    {
     if(Fread(buffer,bufsize,fileid)!=bufsize) result=F_ERROR;
     if(result==F_ERROR) break;

     //Verify buffer
     for(k=0; k<bufsize; k++)
      {
       if(buffer[k] != (unsigned char)(k))
        {
         printf("Verify Error at 0x%08lX\n", i+k);
         result=F_ERROR;
         break;
        }
      }

     if(result==F_ERROR) break;
    }
   Fclose(fileid);

   if(result!=F_ERROR) printf("%s verify ok !\n", name);
   else printf("%s verify NOT ok !\n", name);
  }

 return result;
}

//#############################################################
// start time measure
void StartTimer(void)
//#############################################################
{
 start_timer=timer_counter;
}

//#############################################################
// stop time measure an show results
void StopTimer(void)
//#############################################################
{
 unsigned long tmp1,tmp2,countclock;
 float kBps;

 stop_timer = timer_counter - 1;
 countclock = stop_timer - start_timer;

/*
 printf("%lX ",FileCurrentCluster);
 printf("%lu ",FileCurrentSector);
 printf("%lu\n",FileSize);
*/

 tmp1 = countclock / 100; // timer ticks per count is 10ms, this is the seconds part
 tmp2 = countclock % 100; // timer tick counts below 1s
 printf(" % 3lu.%02lus ",tmp1,tmp2);

 // We have to use float here to avoid rounding
 kBps = (float)TEST_FILE_SIZE / (float)countclock; //Bytes per 0.01 seconds
 kBps *= 1000.0; //Bytes per 10 seconds to get xxx.x values
 kBps /= 1024.0; // kiloBytes per 10 seconds
 tmp1 = (unsigned long)(kBps / 10.0);
 tmp2 = (unsigned long)kBps % 10;
 printf("% 3lu.%lukB/s\n",tmp1,tmp2);
}

//#############################################################
void ShowDOSConfig(void)
//#############################################################
{
#ifdef USE_FAT12
 printf("Using FAT12\n");
#endif

#ifdef USE_FAT16
 printf("Using FAT16\n");
#endif

#ifdef USE_FAT32
 printf("Using FAT32\n");
#endif

#ifdef USE_FATBUFFER
 printf("\nUsing FAT Buffer\n");
#else
 printf("\nNo FAT Buffer\n");
#endif

#ifdef COMPACTFLASH_CARD
 printf("CF Card\n");
#endif
#ifdef MMC_CARD_SPI //SD_CARD_SPI too !
 printf("MMC/SD Card\n");
#endif

#ifdef STANDARD_SPI_WRITE  // No optimizations
 printf("STANDARD_SPI_WRITE\n");
#endif
#ifdef FAST_SPI_WRITE      // Optimizations
 printf("FAST_SPI_WRITE\n");
#endif
#ifdef STANDARD_SPI_READ  // No optimizations
 printf("STANDARD_SPI_READ\n");
#endif
#ifdef FAST_SPI_READ      // Optimizations
 printf("FAST_SPI_READ\n");
#endif

#ifdef USE_SPI0
 printf("SPI0\n");
#endif
#ifdef USE_SPI1
 printf("SPI1\n");
#endif
 printf("SPI_PRESCALER %lu\n",SPI_PRESCALER);


 printf("\nCPU configuration\n");

#if defined (__arm__)
 printf("ARM mode\n");
#endif

#if defined (__thumb__)
 printf("THUMB mode\n");
#endif

 printf("FOSC %lu\n",FOSC);
 printf("PLL_M %lu\n",PLL_M);
 printf("VPBDIV %lu\n\n",VPBDIV_VAL);
}

//#############################################################
void HaltCPU(void)
//#############################################################
{
 printf ("Error executing program !\n\r");
 printf ("Program halted !\n\r");
 while(1);
}

//#############################################################
void SystemInit(void)
//#############################################################
{
	// --- enable and connect the PLL (Phase Locked Loop) ---
	// a. set multiplier and divider
	PLLCFG = MSEL | (1<<PSEL1) | (0<<PSEL0);
	// b. enable PLL
	PLLCON = (1<<PLLE);
	// c. feed sequence
	PLLFEED = PLL_FEED1;
	PLLFEED = PLL_FEED2;
	// d. wait for PLL lock (PLOCK bit is set if locked)
	while (!(PLLSTAT & (1<<PLOCK)));
	// e. connect (and enable) PLL
	PLLCON = (1<<PLLE) | (1<<PLLC);
	// f. feed sequence
	PLLFEED = PLL_FEED1;
	PLLFEED = PLL_FEED2;

	// --- setup and enable the MAM (Memory Accelerator Module) ---
	// a. start change by turning of the MAM (redundant)
	MAMCR = 0;
	// b. set MAM-Fetch cycle to 3 cclk as recommended for >40MHz
	MAMTIM = MAM_FETCH;
	// c. enable MAM
	MAMCR = MAM_MODE;

	// --- set VPB speed ---
	VPBDIV = VPBDIV_VAL;

	// --- map INT-vector ---
	#if defined(RAM_RUN)
	  MEMMAP = MEMMAP_USER_RAM_MODE;
	#elif defined(ROM_RUN)
	  MEMMAP = MEMMAP_USER_FLASH_MODE;
	#else
	#error RUN_MODE not defined!
	#endif
}

