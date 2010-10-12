/*! \file "fat.h" \brief FAT-Definitions */
///	\ingroup multifat
///	\defgroup FAT FAT-Functions (fat.h)
//#########################################################################
//
// Benutzt nur die erste Partition
// Nur für Laufwerke mit 512 Bytes pro Sektor
//
// Nach einem White Paper von MS
// FAT: General Overview of On-Disk Format
// Version 1.03, December 6, 2000
//
// Benutzung auf eigene Gefahr !
//
// Use at your own risk !
//
//#########################################################################
// Last change: 24.10.2007
//#########################################################################
// Compiler: WinARM 4.1.1
//#########################################################################
//@{

#ifndef __FAT_H
#define __FAT_H

#include "typedefs.h"

// These defines are for FAT debugging only. You don't need them.
// Activating all three options takes about 1.5kB of flash.
// So be careful on devices with small flash memory !
//#define FAT_DEBUG_SHOW_FAT_INFO //activate FAT information output via printf() or puts() in GetDriveInformation()
//#define FAT_DEBUG_CLUSTERS // show cluster numbers read/write access via printf()

//#define USE_64k_CLUSTERS  // This will use more code !
                          // You should not use 64kB Clusters if your card is smaller than 4GB

//file operations
#define END_DIR		0
#define NO_MATCH	1
#define MATCH_NAME	2
#define MATCH_EXT	3
#define FULL_MATCH	MATCH_NAME + MATCH_EXT

#define PART1_TABLE_OFFSET (U16) 0x01BE //offset to first partitiontable in sector 0

//Using structures needs less memory than indexing in arrays like inbuff[]

//partitiontable structure
//most of it is not used in this program
//bootsector offset is the only thing we need
//because C/H/S values are not used. LBA !
struct PartInfo {
                 U8 status;      //Partition status, 0x80 = Active, 0x00 = inactive
                 U8 firsthead;   //First head used by partition
                 U16  firstseccyl; //First sector and cylinder used by partition
                 U8 type;        //Partition type
                 U8 lasthead;    //Last head used by partition
                 U16  lastseccyl;  //Last sector and cylinder used by partition
                 U32 bootoffset;  //Location of boot sector. !!!!!!!!!!!
                 U32 secofpart;   //Number of sectors for partition
} __attribute__((packed));

//first sector of disc is the master boot record
//it contains four partitiontables
//only the first partition is used in this program
struct MBR {
            U8 dummy[PART1_TABLE_OFFSET]; //we don't need all these bytes
            struct PartInfo part1;
            struct PartInfo part2;
            struct PartInfo part3;
            struct PartInfo part4;
//all bytes below are not necessary
} __attribute__((packed));

//part of FAT12/16 bootsector different to FAT32
struct RemBoot //FAT12/16 defs beginning at offset 36
 {
	U8  BS_DrvNum;
	U8  BS_Reserved1;
	U8  BS_BootSig;
	U8  BS_VolID[4];
	char           BS_VolLab[11];
	char           BS_FilSysType[8];
	U8  remaining_part[450];
 } __attribute__((packed));

//part of FAT32 bootsector different to FAT12/16
struct RemBoot32 //FAT32 defs beginning at offset 36
  {
	U32  BPB_FATSz32; //4 bytes
	U16   BPB_ExtFlags; //2 bytes
	U16   BPB_FSVer; //2 bytes
	U32  BPB_RootClus; //4 bytes
	U16   BPB_FSInfo; //2 bytes
	U16   BPB_BkBootSec; //2 bytes
	U8  BPB_Reserved[12];
	U8  BS_DrvNum;
	U8  BS_Reserved1;
	U8  BS_BootSig;
	U32  BS_VolID; //4 bytes
	char           BS_VolLab[11];
	char           BS_FilSysType[8];
	U8  remaining_part[422];
} __attribute__((packed)); 

union endboot 
{
       struct RemBoot   rm;
       struct RemBoot32 rm32;
} __attribute__((packed));

struct BootSec 
{
	U8  BS_jmpBoot[3];
	char           BS_OEMName[8];
	U16   BPB_BytesPerSec; //2 bytes
	U8  BPB_SecPerClus;
	U16   BPB_RsvdSecCnt; //2 bytes
	U8  BPB_NumFATs;
	U16   BPB_RootEntCnt; //2 bytes
	U16   BPB_TotSec16; //2 bytes
	U8  BPB_Media;
	U16   BPB_FATSz16; //2 bytes
	U16   BPB_SecPerTrk; //2 bytes
	U16   BPB_NumHeads; //2 bytes
	U32  BPB_HiddSec; //4 bytes
	U32  BPB_TotSec32; //4 bytes
        union endboot  eb; //remaining part of bootsector
} __attribute__((packed));


#ifndef BYTE_PER_SEC
 #define BYTE_PER_SEC (U16) 512
#endif 

#define FAT12	(U8) 12
#define FAT16	(U8) 16
#define FAT32	(U8) 32

//defines for special cluster values
//free cluster has value 0
//for fat32 don't use upper four bits ! ignore them
//cluster value of 0x10000000 is a FREE cluster in FAT32
 
//values for end of cluster chain
//ranges for example for FAT12 from 0xFF8 to 0xFFF
#define EOC12	(U16) 0xFF8
#define EOC16	(U16) 0xFFF8
#define EOC32	(U32) 0x0FFFFFF8

//values for bad marked clusters
#define BADC12	(U16) 0xFF7
#define BADC16	(U16) 0xFFF7
#define BADC32	(U32) 0x0FFFFFF7

//values for reserved clusters
//ranges for example for FAT12 from 0xFF0 to 0xFF6
#define RESC12	(U16) 0xFF0
#define RESC16	(U16) 0xFFF0
#define RESC32	(U32) 0x0FFFFFF0

#ifdef USE_FAT32
 #define DISK_FULL (U32) 0xFFFFFFFF
#else                              
 #define DISK_FULL (U16) 0xFFFF
#endif

//File/Dir Attributes
#define ATTR_FILE	(U8) 0x00 //not defined by MS ! I did it 
#define ATTR_READ_ONLY	(U8) 0x01
#define ATTR_HIDDEN	(U8) 0x02
#define ATTR_SYSTEM	(U8) 0x04
#define ATTR_VOLUME_ID	(U8) 0x08
#define ATTR_DIRECTORY	(U8) 0x10
#define ATTR_ARCHIVE	(U8) 0x20
#define ATTR_LONG_NAME  (U8) 0x0F
#define ATTR_NO_ATTR  	(U8) 0xFF //not defined by MS ! I did it 

//Char codes not allowed in a filename
//NOT checked yet
//0x22, 0x2A, 0x2B, 0x2C, 0x2E, 0x2F, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x5B, 0x5C, 0x5D, and 0x7C. 

struct DirEntry {
                 char DIR_Name[11];      //8 chars filename 3 chars extension
                 U8 DIR_Attr;         //file attributes RSHA
                 U8 DIR_NTres;        //set to zero
                 U8 DIR_CrtTimeTenth; //creation time part in milliseconds
                 U16  DIR_CrtTime;      //creation time
                 U16  DIR_CrtDate;      //creation date
                 U16  DIR_LastAccDate;  //last access date (no time for this !)
                 U16  DIR_FstClusHI;  //first cluster high word                 
                 U16  DIR_WrtTime;      //last write time
                 U16  DIR_WrtDate;      //last write date
                 U16  DIR_FstClusLO;  //first cluster low word                 
                 U32 DIR_FileSize;     
                } __attribute__((packed));

//do a little trick for getting long name characters from a DirEntry
//DirEntryBuffer later gets the same adress as DirEntry
struct DirEntryBuffer {
                 U8 longchars[sizeof(struct DirEntry)];
                } __attribute__((packed));

//Prototypes
extern U8 GetDriveInformation(void);
extern void UpdateFATBuffer(U32 newsector);

#ifdef USE_FAT32
 extern U32 GetFirstSectorOfCluster(U32 n);
 extern U32 GetNextClusterNumber(U32 cluster);
 extern U8 WriteClusterNumber(U32 cluster, U32 number);
 extern U32 AllocCluster(U32 currentcluster);
 extern U32 FindFreeCluster(U32 currentcluster);
#else
 extern U32 GetFirstSectorOfCluster(U16 n);
 extern U16 GetNextClusterNumber(U16 cluster);
 extern U8 WriteClusterNumber(U16 cluster, U16 number);
 extern U16 AllocCluster(U16 currentcluster);
 extern U16 FindFreeCluster(U16 currentcluster);
#endif

#ifdef USE_FAT32
 extern U32 endofclusterchain;
 extern U32 maxcluster;        // last usable cluster+1
 extern U32 FAT32RootCluster;
#else
 extern U16 endofclusterchain;
 extern U16 maxcluster;        // last usable cluster+1
#endif

extern U8 secPerCluster;

#ifdef USE_64k_CLUSTERS
 extern U32 BytesPerCluster;
#else
 extern U16 BytesPerCluster;
#endif

extern U8 fatbuf[];   //buffer for FAT sectors

//extern U32 FATHits;	// count FAT write cycles. you don't really need this ;)
extern U32 FATFirstSector;
extern U32 FATCurrentSector;
extern U8 FATtype;
extern U8 FATStatus; // only for FAT write buffering

extern U32 FirstRootSector;
extern U32 FirstDataSector; 
//extern U32 RootDirSectors;
extern U8 RootDirSectors;

#endif //FAT_H
//@}
