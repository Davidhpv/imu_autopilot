/*! \file "dos.h" \brief DOS-Definitions */
///	\ingroup multifat
///	\defgroup DOS DOS-Functions (dos.h)
//#########################################################################
// Benutzung auf eigene Gefahr !
//
// Use at your own risk !
//
// 18.02.2007 Changed F_READ to 'r', F_WRITE to 'w'
//            Now you can do Fopen("mydata.txt",'w');    ;)
//
//#########################################################################
// Last change: 12.08.2007
//#########################################################################
// Compiler: WinARM 4.1.1
//#########################################################################
//@{
#ifndef __DOS_H
#define __DOS_H

#include "typedefs.h"

#define MAX_OPEN_FILE    5
// every 1 declared open file takes 558 Bytes for FAT12/16/32
// every 1 declared open file takes 554 Bytes for FAT12/16 only

 typedef S16 FileID;

// Some more defines for special filesystem handling

#define STRICT_FILESYSTEM_CHECKING // If you define this, some very rare special cases will be
				    // noticed. But this needs more code.
				    // Special cases when files where made on a Win system and
				    // copied to the flash card:
				    //
                                    // Bug found by Michele Ribaudo
				    // Files with zero filesize have no clusters allocated !
				    // Calling Remove() and Fopen() may hang up the system.
				    // If you define STRICT_FILESYSTEM_CHECKING opening a
				    // zero length file for reading or writing gives an error.
				    // You can remove a zero length file !
				    //
				    // You don't need this define if all files where made with
				    // my FAT system. Even if filesize is zero.
                                    // If unsure keep it defined !

//fopen flags
#define F_CLOSED	0
#define F_READ		'r' // read only
#define F_WRITE		'w' // read/write, no automatic seek to end of file
#define F_APPEND	'a' // read/write, automatic seek to end of file

#define F_ERROR		0 // dir/file operation failed
#define F_OK		1 // dir/file operation successfull

#ifndef SEEK_SET
 #define	SEEK_SET	0	/* set file offset to offset */
#endif
#ifndef SEEK_CUR
 #define	SEEK_CUR	1	/* set file offset to current plus offset */
#endif
#ifndef SEEK_END
 #define	SEEK_END	2	/* set file offset to EOF plus offset */
#endif

// #undef defines below in "dosdefs.h" if you don't need them
// spare program memory by deciding if we want to read, write or both
#define DOS_READ	//define this if you want to read files
#define DOS_WRITE	//define this if you want to write files
#define DOS_DELETE	//define this if you want to delete files
#define DOS_READ_RAW	//define this if you want to read files with ReadFileRaw()

#define DOS_CHDIR	//define this if you want to go into subdirectories
#define DOS_MKDIR	//define this if you want to make subdirectories
#define DOS_RENAME	//define this if you want to rename files
#define DOS_FSEEK       //define this if you want to seek in files (reading only)

// spare program memory by deciding if we want to use FAT12, FAT16, FAT32.
// don't touch if you don't know the FAT type of your drive !
#define USE_FAT12	//define this if you want to use FAT12
#define USE_FAT16	//define this if you want to use FAT16
#define USE_FAT32	//define this if you want to use FAT32

#define USE_FATBUFFER	//define this if you want to use a FAT buffer
                        //needs 517 Bytes of RAM !

#define USE_FINDFILE	//define this if you want to use Findfirst(); Findnext();
#define USE_FINDLONG    //define this if you want to get long filenames
			//from Findfirst(); Findnext();

#define USE_FOPEN_LONG     // Fopen() with LFN support (not for new file creation with LFN !)
#define USE_CHDIR_LONG     // Chdir() with LFN support
#define USE_REMOVE_LONG    // Remove() with LFN support
#define USE_FINDNAME_LONG  // FindName() with LFN support
//#define USE_LISTDIR_LONG   // ListDirectory() with LFN support

#define USE_DRIVEFREE   //define this if you want to get free and used space of your drive

#include "dosdefs.h"  // keep the line at this place ! don't move down or delete

extern FileID Fopen(char *name, U8 flag);
extern FileID Fopen83(char *name, U8 flag);
extern void Fclose(FileID fileid);

extern U16 Fread(U8 *buf, U16 count, FileID fileid );
extern U16 Fwrite(U8 *buf, U16 count, FileID fileid);

extern void Fflush(FileID fileid);
extern void fflush_all(void);
extern U8 Remove(char *name);
extern U8 Remove83(char *name);

extern U8 ReadFileRaw(char *name);
extern U8 FindEntry(FileID fileid);
extern U8 FindName(char *name);
extern U8 FindName83(char *name);
extern U8 Rename(char *OldName, char *NewName);

extern U8 Fseek(S32 offset, U8 mode, FileID fileid);
extern void FlushWriteBuffer(FileID fileid);

extern U32 Filelength(FileID fileid);
extern FileID findfreefiledsc(void); // return -1 if too many open files

#ifndef BYTE_PER_SEC
 #define BYTE_PER_SEC (U16) 512
#endif

struct FileDesc { //FileDescriptor structure
                 U8 iob[BYTE_PER_SEC];      //file i/o buffer
//		 U32 FileCurrentSector;      //number of sector with last data read/written
		 U32 FileDirSector;          //dir sector holding this fileentry
		 U32 FileSize;
		 U32 FilePosition;           //file byte position
                 U32 FileFirstClusterSector;    // first sector of cluster with last data read/written
		 U8 FileClusterSectorOffset;   //sector of current cluster used
		 U8 FileFlag;               //open or closed
		 U8 FileAttr;               //file attribute. also used for directory functions
		 U32 FileClusterCount;       //this is NOT uint !
#ifdef USE_FAT32
                 U32 FileFirstCluster;       //needed for UpdateFileEntry() !
                 U32 FileCurrentCluster;     //number of cluster in use
#else //#ifdef USE_FAT32
                 U16 FileFirstCluster;
                 U16 FileCurrentCluster;
#endif //#ifdef USE_FAT32
		 char FileName[11];            //file name
		 U8 FileDirOffset;          //dir entry offset in FileDirSector/32
#ifdef DOS_WRITE
                 U8 FileWriteBufferDirty;
#endif
                };

extern struct FileDesc FileDescriptors[];

//this is for easier and faster converting from byte arrays to UINT, ULONG
//ui and ul share the same memory space

union Convert {
 U16  ui;
 U32  ul;
};


#ifdef COMPACTFLASH_CARD
 #include "compact.h"
#endif

#ifdef MMC_CARD_SPI
 #include "mmc_spi.h"
#endif

#include "fat.h"

//dir.c
extern U8 Mkdir(char *name);
extern U8 Chdir(char *name);
extern U8 Chdir83(char *name);
extern U8 MakeNewFileEntry(FileID fileid);
extern U8 UpdateFileEntry(FileID fileid);
extern U16 DOSTime(void);
extern U16 DOSDate(void);

extern U8 ScanOneDirectorySector(FileID fileid, U32 sector);

#ifdef USE_FAT32
 extern U8 SearchFreeDirentry(FileID fileid, U32 cluster);
 extern U8 ScanDirectory(FileID fileid, U32 startcluster);
#else
 extern U8 SearchFreeDirentry(FileID fileid, U16 cluster);
 extern U8 ScanDirectory(FileID fileid, U16 startcluster);
#endif

extern U8 SearchDirSector(FileID fileid, U32 sector);

extern void MakeDirEntryName(char *inname, FileID fileid);
extern void ZeroCluster(U32 startsector);

extern U8 dirbuf[];   //buffer for directory sectors

#ifdef USE_FAT32
 extern U32 FirstDirCluster;
#else
 extern U16 FirstDirCluster;
#endif
//dir.c

#ifdef USE_FINDFILE
 #include "find_x.h"
#endif

//drivefree.c
extern U32 drivefree(void);
extern U32 driveused(void);
extern U32 drivesize(void);

//lfn_util.c
extern void ListDirectory(void);
extern FileID FopenLFN(char* filename, U8 fileflag);
extern U8 ChdirLFN(char *dirname);
extern U8 RemoveLFN(char *name);
extern U8 FindNameLFN(char *name);

#endif //__DOS_H
//@}
