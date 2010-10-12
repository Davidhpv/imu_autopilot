/*! \file dir.c
    \brief Directory-Functions
*/
//###########################################################
///	\ingroup multifat
///	\defgroup DIR DIR-Functions (dir.c)
///	\code #include "dir.h" \endcode
///	\code #include "dos.h" \endcode
///	\par Uebersicht
//###########################################################
//
// For FAT12, FAT16 and FAT32
// Only for first Partition
// Only for drives with 512 bytes per sector (the most)
//
// Benutzung auf eigene Gefahr !
//
// Use at your own risk !
//
// 24.08.2007 Changed MakeFileName() to MakeDirEntryName().
//            A lot of optimizations to make smaller code.
//
// 10.08.2007 A lot of changes in ScanOneDirectorySector().
//
// 08.08.2007 Merged ScanRootDir() and ScanSubDir() in one function ScanDirectory()
//
// 20.11.2006 Bugfix. If WIN makes a directory for FAT32 and directory
//            is in Root directory, upperdir cluster is ZERO !
//
//#########################################################################
// Last change: 06.11.2008
//#########################################################################
// hk@holger-klabunde.de
// http://www.holger-klabunde.de/index.html
//#########################################################################
// Compiler: WinARM 4.1.1
//#########################################################################
//@{
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "sys_time.h"

#include "dos.h"
#include "rtc.h"

U8 dirbuf[BYTE_PER_SEC];   //buffer for directory sectors

#ifdef USE_FAT32
 U32 FirstDirCluster=0;
#else
 U16 FirstDirCluster=0;
#endif

#ifdef DOS_WRITE
//###########################################################
/*!\brief Make DOS time from system time
 * \return 		time in DOS format
 */
U16 DOSTime(void)
//###########################################################
{
// RTCTime local_time;
// U16 time;
//
// local_time=RTCGetTime();
//// while(seconds==59); // maybe we should do this !?
//
//// fixed time 16:19:33 if no clock exists
///*
// time  = (U16)16 << 11; // hours
// time |= (U16)19 << 5;  // minutes
// time |= 33 / 2;        // seconds
//*/
//
// time  = (U16)local_time.RTC_Hour << 11;
// time |= (U16)local_time.RTC_Min << 5;
// time |= (U16)local_time.RTC_Sec / 2;

	// FIXME SDCARD
 return sys_time_clock_get_unix_time();
}
#endif //DOS_WRITE

#ifdef DOS_WRITE
//###########################################################
/*!\brief Make DOS date from system date
 * \return 		date in DOS format
 */
U16 DOSDate(void)
//###########################################################
{
// RTCTime local_time;
// U16 date;
//
// local_time=RTCGetTime();
//
//// fixed date 18.02.2007 if no clock exists
///*
// date = 2007 - 1980;  // years since 1980
// date <<= 9;
//
// date |= (U16)2 << 5;  // month
// date |= (U16)18;      // day
//*/
//
// date = (U16)local_time.RTC_Year - 1980;  // years since 1980
// date <<= 9;
//
// date |= (U16)local_time.RTC_Mon << 5;
// date |= (U16)local_time.RTC_Mday;

	// FIXME SDCARD
	uint16_t date = sys_time_clock_get_unix_time();

 return date;
}
#endif //DOS_WRITE

#ifdef DOS_WRITE
//###########################################################
/*!\brief Search sub dir for free dir entry
 * \param		fileid	A fileid you got from Fopen() or other functions
 * \param		startcluster	first cluster number of the sub dir
 * \return 		Above 0 if successfull, 0 if not
 */
#ifdef USE_FAT32
 U8 SearchFreeDirentry(FileID fileid, U32 startcluster)
#else
 U8 SearchFreeDirentry(FileID fileid, U16 startcluster)
#endif
//###########################################################
{
 U32 tmpsector;

#ifdef USE_FAT32
 U32 tmpcluster;
#else
 U16 tmpcluster;
#endif

 U8 i,result;
 result=0;

 if(startcluster<2) // We are in root dir for FAT12/16
  {
   tmpsector = FirstRootSector;

   i = RootDirSectors;
   do
    {
     result=SearchDirSector(fileid,tmpsector++);
     if(result!=0) break; //break sector loop
     i--;
    }while(i);
  }
 else // We are in a subdir
  {
   tmpcluster=startcluster;

   while(tmpcluster < endofclusterchain)
    {
     tmpsector=GetFirstSectorOfCluster(tmpcluster);

     i = secPerCluster;
     do
      {
       result=SearchDirSector(fileid, tmpsector++);
       if(result!=0) break; //break sector loop
       i--;
      }while(i);

     if(result!=0) break; //break cluster loop
     tmpcluster=GetNextClusterNumber(tmpcluster);
    }
  }

 return result;
}
#endif //DOS_WRITE

#ifdef DOS_WRITE
//###########################################################
/*!\brief Search dir sector for free dir entry
 * \param		fileid	A fileid you got from Fopen() or other functions
 * \param		sector	Sector number of the directory
 * \return 		Above 0 if successfull, 0 if not
 */
U8 SearchDirSector(FileID fileid, U32 sector)
//###########################################################
{
 U16 count;
 struct FileDesc *fdesc;

 fdesc=&FileDescriptors[(U16)fileid];

 ReadSector(sector,dirbuf); //read one directory sector.
 count=0;
 do
  {
   if(dirbuf[count]==0 || dirbuf[count]==0xE5)
    {
     fdesc->FileDirSector=sector;     //keep some values in mind
     fdesc->FileDirOffset=count/32;
     return 1;
    }
   count+=32;
  }while(count<BYTE_PER_SEC);

 return 0;
}
#endif //DOS_WRITE

#ifdef DOS_WRITE
//###########################################################
/*!\brief Fill a cluster with zeros
 * \param		startsector
 * \return 		nothing
 */
void ZeroCluster(U32 startsector)
//###########################################################
{
 U32 sector;
 U16 i;

 sector = startsector;

 //clean all sectors of a cluster (fill with 0x00)
 for(i=0; i<BYTE_PER_SEC; i++) dirbuf[i]=0; //Fill write buffer

 unsigned char j = secPerCluster;

 do
  {
   WriteSector(sector++,dirbuf);
   j--;
  }while(j);

}
#endif

#ifdef DOS_WRITE
#ifdef DOS_MKDIR
//###########################################################
/*!\brief Make a new directory
 * \param		name	8.3 DOS name
 * \return 		F_OK if successfull, otherwise F_ERROR
 */
U8 Mkdir(char *name)
//###########################################################
{
 struct FileDesc *fdesc;

 FileID fileid=findfreefiledsc();
 if(fileid==-1) return F_ERROR; // no free filedescriptor found

 fdesc=&FileDescriptors[(U16)fileid];

 //Test if directoryname exists in current directory
 MakeDirEntryName(name,fileid); //split name into name and extension parts

 if(FindEntry(fileid)==FULL_MATCH)
 {
  if(fdesc->FileAttr == ATTR_DIRECTORY)
   {
    fdesc->FileFlag=0; // Close this filedescriptor
    return F_OK;       // directory exists
   }

  fdesc->FileFlag=0; // Close this filedescriptor
  return F_ERROR;    // There is a FILE named "name" !
 }

 fdesc->FileAttr=ATTR_DIRECTORY;  //want to make a directory !

 if(MakeNewFileEntry(fileid))
  {
   //MakeNewFileEntry() allocates first cluster for new directory !
   //first cluster of new directory is returned in FileFirstCluster

   fdesc->FileDirSector=GetFirstSectorOfCluster(fdesc->FileFirstCluster);

   ZeroCluster(fdesc->FileDirSector);

   //insert "." my new dir entry with newdir firstcluster
   fdesc->FileDirOffset=0; //first direntry "."
   MakeDirEntryName(".",fileid);
   UpdateFileEntry(fileid);

   //insert ".." upper dir entry with upperdir firstcluster
   fdesc->FileDirOffset=1;  //2nd direntry ".."
   MakeDirEntryName("..",fileid);
   fdesc->FileFirstCluster=FirstDirCluster;
   UpdateFileEntry(fileid);

#ifdef USE_FATBUFFER
     WriteSector(FATCurrentSector,fatbuf); // write the FAT buffer
     FATStatus=0;
#endif
  }
 else
  {
   fdesc->FileFlag=0; // Close this filedescriptor
//   fdesc->FileAttr=ATTR_FILE; //default
   return F_ERROR; //new dir could not be made
  }

 fdesc->FileFlag=0; // Close this filedescriptor
// fdesc->FileAttr=ATTR_FILE; //default
 return F_OK;
}
#endif //DOS_MKDIR
#endif //DOS_WRITE

#ifdef DOS_CHDIR
//###########################################################
/*!\brief Change to a directory
 * \param		name	8.3 DOS name
 * \return 		F_OK if successfull, otherwise F_ERROR
 */
//###########################################################
U8 Chdir(char *name)
//###########################################################
{
 #if defined (USE_CHDIR_LONG)
  return ChdirLFN(name);
 #else
  return Chdir83(name);
 #endif
}

//###########################################################
U8 Chdir83(char *name)
//###########################################################
{
 struct FileDesc *fdesc;

 FileID fileid=findfreefiledsc();

 if(fileid==-1) return F_ERROR;

 fdesc=&FileDescriptors[(U16)fileid];

 if(name[0]=='/')
  {
#ifdef USE_FAT12
   if(FATtype==FAT12) FirstDirCluster=0;
#endif
#ifdef USE_FAT16
   if(FATtype==FAT16) FirstDirCluster=0;
#endif
#ifdef USE_FAT32
   if(FATtype==FAT32) FirstDirCluster=FAT32RootCluster;
#endif
   fdesc->FileFlag=0;  // Close this filedescriptor
   return F_OK;
  }

 MakeDirEntryName(name,fileid);

 if(FindEntry(fileid)==FULL_MATCH)
 {
  if(fdesc->FileAttr == ATTR_DIRECTORY) // Is this a directory ?
   {
     //First cluster of directory is returned in FileFirstCluster
    FirstDirCluster = fdesc->FileFirstCluster;
    fdesc->FileFlag=0;  // Close this filedescriptor
    return F_OK;
   }
 }

 fdesc->FileFlag=0;  // Close this filedescriptor
 return F_ERROR; // Maybe there is a FILE named "name" !
}
#endif //DOS_CHDIR

#ifdef DOS_WRITE
//###########################################################
/*!\brief Make a new file/directory entry in current directory
 * \param		fileid Number of a filedescriptor
 * \return 		F_OK if successfull, otherwise F_ERROR
 */
U8 MakeNewFileEntry(FileID fileid)
//###########################################################
{
#ifdef USE_FAT32
 U32 tmpcluster,lastdircluster;
#else
 U16 tmpcluster,lastdircluster;
#endif

 U8 result;
 struct FileDesc *fdesc;

 fdesc=&FileDescriptors[(U16)fileid];

 result=SearchFreeDirentry(fileid,FirstDirCluster);
 if(result==0) // no free direntry found. lets try to alloc and add a new dircluster
  {
   //if dir is rootdir (FAT12/16 only) you have a problem ;)
#if defined (USE_FAT12) || defined (USE_FAT16)
   if(FirstDirCluster<2)
    {
#ifdef USE_FAT12
     if(FATtype==FAT12) return F_ERROR;
#endif
#ifdef USE_FAT16
     if(FATtype==FAT16) return F_ERROR;
#endif
    }
#endif

   //search the last cluster of directory
   lastdircluster=FirstDirCluster;
   do
    {
     tmpcluster=GetNextClusterNumber(lastdircluster);
     if(tmpcluster < endofclusterchain) lastdircluster=tmpcluster;
    }while(tmpcluster < endofclusterchain);

   tmpcluster=AllocCluster(lastdircluster); //if current directory is full alloc new cluster for dir
   if(tmpcluster==DISK_FULL) //no free clusters ?
    {
     return F_ERROR;
    }

   fdesc->FileDirSector=GetFirstSectorOfCluster(tmpcluster);

   ZeroCluster(fdesc->FileDirSector);

   fdesc->FileDirOffset=0;              //set offset for new direntry in dirsector
  }

 tmpcluster=AllocCluster(0); //alloc first cluster for file
 if(tmpcluster==DISK_FULL) //no free clusters ?
  {
   return F_ERROR;
  }

 fdesc->FileFirstCluster=tmpcluster;
 fdesc->FileSize=0;
 UpdateFileEntry(fileid); //write new file entry

 return F_OK; //all ok
}
#endif //DOS_WRITE

#ifdef DOS_WRITE
//###########################################################
/*!\brief Update directory entry of a file/dir
 * \param		fileid	A fileid you got from Fopen()
 * \return 		F_OK if successfull, F_ERROR if not
 */
U8 UpdateFileEntry(FileID fileid)
//###########################################################
{
 struct DirEntry *di;
 struct FileDesc *fdesc;

 fdesc=&FileDescriptors[(U16)fileid];

 ReadSector(fdesc->FileDirSector,dirbuf);
 di=(struct DirEntry *)&dirbuf[fdesc->FileDirOffset * 32];

 strncpy(di->DIR_Name,fdesc->FileName,11);

 di->DIR_Attr=fdesc->FileAttr;
 di->DIR_NTres=0;
 di->DIR_CrtTimeTenth=0;

 di->DIR_CrtTime= DOSTime();           //creation time
 di->DIR_WrtTime= di->DIR_CrtTime;     //last write time
 di->DIR_CrtDate= DOSDate();           //creation date
 di->DIR_LastAccDate= di->DIR_CrtDate; //last access date
 di->DIR_WrtDate= di->DIR_CrtDate;     //last write date

#ifdef USE_FAT32
 di->DIR_FstClusHI=(U16)(fdesc->FileFirstCluster>>16);  //first cluster high word
#else
 di->DIR_FstClusHI=0;  //first cluster high word
#endif
 di->DIR_FstClusLO=(U16)(fdesc->FileFirstCluster);  //first cluster low word
 di->DIR_FileSize=fdesc->FileSize;

 WriteSector(fdesc->FileDirSector,dirbuf);
 return F_OK;
}
#endif //DOS_WRITE

//#####################################################################
/*!\brief Heart of the directory functions
 * \param		fileid	A fileid you got from Fopen() or other functions
 * \param		sector	Sector number of the directory
 * \return 		FULL_MATCH if successfull, NO_MATCH or END_DIR if not
 *

 Following global variables will be set if dir or filename was found:

 For directories:

 FirstDirCluster   First cluster of new current directory

 For files:

 FileName[fileid]          8 chars for filename

 FileExt[fileid]           3 chars for extension

 FileDirSector[fileid]     Sector which keeps direntry of the file

 FileDirOffset[fileid]     Offset to direntry of the file

 FileFirstCluster[fileid]  First cluster of the dir/file in FAT

 FileSize[fileid]

 FileAttr[fileid]
*/
//
U8 ScanOneDirectorySector(FileID fileid,U32 sector)
//#####################################################################
{
 U8 by;
 U16 count;
 struct DirEntry *di;
 struct DirEntryBuffer *dib;
 U8 match;
 struct FileDesc *fdesc;

 if(fileid>=MAX_OPEN_FILE) return NO_MATCH;
 if(fileid<(-1)) return NO_MATCH;

 if(fileid>=0) fdesc=&FileDescriptors[(U16)fileid];
 else fdesc=NULL;

 by=ReadSector(sector,dirbuf); //read one directory sector.
 count=0;
 do
  {
   match=NO_MATCH;
   di=(struct DirEntry *)(&dirbuf[count]);

   //make a second pointer to dirbuf for easier access to long filename entrys
   dib=(struct DirEntryBuffer *)di;

   if(di->DIR_Name[0]==0) return END_DIR; //end of directory

   if((unsigned char)di->DIR_Name[0]!=0xE5) // Deleted entry ?
    {
     di->DIR_Attr&=0x3F;            //smash upper two bits

     if(di->DIR_Attr==ATTR_LONG_NAME) //is this a long name entry ?
      {
#ifdef USE_FINDFILE
#ifdef USE_FINDLONG
       if(fileid==-1) //FIND_OPERATION
        {
         // Build the long name from the 13 bytes long name direntrys.
         // The long name direntrys have to be in a block of direntrys.
         // Otherwise this will not work and you get strange results.


         if(ffblk.newposition >= ffblk.lastposition) //found a new file ?
          {
           U8 offset; //offset of the direntry in long name
           U8 ffcount;  //loop counter
           U8 i;

           offset=dib->longchars[0];
           offset&=0x1F; //force upper bits D7..5 to zero.
                         //max. value of 20 is allowed here, or even less
                         //if _MAX_NAME is defined smaller than 255.
                         //long filenames will then be cut at _MAX_NAME - 1.

           if(offset>20) offset=20; //force maximum value if too big

           ffcount=(offset-1) * 13; // calc start adress in long name array

           //We can not use strncpy() because a CHAR in the long name
           //direntry has two bytes ! 2nd byte is ignored here.

           for(i=1; i<=9; i+=2)
            {
             by=dib->longchars[i];
             if(ffcount<_MAX_NAME) ffblk.ff_longname[ffcount]=by;
             ffcount++;
            }

           for(i=14; i<=24; i+=2)
            {
             by=dib->longchars[i];
             if(ffcount<_MAX_NAME) ffblk.ff_longname[ffcount]=by;
             ffcount++;
            }

           for(i=28; i<=30; i+=2)
            {
             by=dib->longchars[i];
             if(ffcount<_MAX_NAME) ffblk.ff_longname[ffcount]=by;
             ffcount++;
            }

           ffblk.ff_longname[_MAX_NAME-1]=0; //End of string to avoid buffer overruns

          }//if(ffblk.newposition >= ffblk.lastposition)
        }//if(fileid==-1)
#endif //#ifdef USE_FINDLONG
#endif //#ifdef USE_FINDFILE
      }
     else //no long name entry
      {
       if(fileid==-1) //List only
        { //place a listing function here
        }
       else
        {
         if(strncmp(fdesc->FileName,di->DIR_Name,11)==0) match=FULL_MATCH; //does name match ?
         //match==FULL_MATCH if name found
        }

       if(di->DIR_Attr & ATTR_VOLUME_ID) //is this a volume label ?
        {           //nothing to do here. volume id not supported
        }
       else //FILE/DIR/FIND operation
        {
         if(fileid==-1)  //FIND_OPERATION
          {
#ifdef USE_FINDFILE
           ffblk.newposition++; //one more entry found

           if(ffblk.newposition > ffblk.lastposition) //found a new file ?
            {
             ffblk.lastposition=ffblk.newposition;    //save new file position

             // di->DIR_Name may look like this "0123     ext"
             // But we need this "0123.ext". If there is no extension
             // we also don't want a '.'.

             unsigned char pos,pos1;

             pos = 0;
             do
              {
               if(di->DIR_Name[pos] == ' ') break;
               else ffblk.ff_name[pos] = di->DIR_Name[pos];
               pos++;
              }while(pos < 8);

             if(di->DIR_Name[8] != ' ') // we have an extension
              {
               ffblk.ff_name[pos++] = '.'; // insert '.'

               pos1 = 8; // position of extension
               do
                {
                 if(di->DIR_Name[pos1] == ' ') break;
                 else ffblk.ff_name[pos++] = di->DIR_Name[pos1];
                 pos1++;
                }while(pos1 < 11);

              }

//             strncpy(ffblk.ff_name,di->DIR_Name,8);   //copy filename
//             strncpy(&ffblk.ff_name[9],&di->DIR_Name[8],3);//copy fileextension

             if(di->DIR_Attr & ATTR_DIRECTORY)
              {
               ffblk.ff_attr=ATTR_DIRECTORY;      //file attribute
               ffblk.ff_fsize=0; 		  //not a file, clear filesize
              }
             else
              {
               ffblk.ff_attr=ATTR_FILE;
               ffblk.ff_fsize=di->DIR_FileSize;
              }

             return FULL_MATCH; //found next entry, stop searching

            }//if(ffblk.newposition > ffblk.lastposition)
#endif//#ifdef USE_FINDFILE
          }//if(fileid==-1)
         else  //FILE/DIR operation
          {
#ifdef USE_FAT32
           fdesc->FileFirstCluster = di->DIR_FstClusHI; //Highword of first cluster number
           fdesc->FileFirstCluster <<= 16;
           fdesc->FileFirstCluster += di->DIR_FstClusLO; //Lowword of first cluster number
#else
           fdesc->FileFirstCluster = di->DIR_FstClusLO; //Lowword of first cluster number
#endif

           fdesc->FileDirSector=sector; //keep some values in mind
           fdesc->FileDirOffset=count/32;

           if(match==FULL_MATCH)
            {

             if(di->DIR_Attr & ATTR_DIRECTORY) //this is a directory
              {

#ifdef USE_FAT32
// Special case for FAT32 and directories in ROOT directory MADE BY WIN.
// Upper directory ".." first cluster for a subdirectory is ZERO here, and NOT FAT32RootCluster !
// Bug found by Andreas ???
               if(FATtype==FAT32)
                {
                 if(fdesc->FileFirstCluster < 2) fdesc->FileFirstCluster = FAT32RootCluster; // force to correct cluster
                }
#endif //#ifdef USE_FAT32

               fdesc->FileSize = 0; // Directorys have no size
               fdesc->FileAttr = ATTR_DIRECTORY;

              }//if(di->DIR_Attr & ATTR_DIRECTORY)
             else //is not a directory. this is a file
              {
               fdesc->FileSize = di->DIR_FileSize;
               fdesc->FileAttr = ATTR_FILE;
              }//if(di->DIR_Attr & ATTR_DIRECTORY)

             return FULL_MATCH;
            }//if(match==FULL_MATCH)

         }//if(fileid==-1)
        }
      }

    }//if(di->DIR_Name[0]!=0xE5)

   count+=32;
  }while(count<BYTE_PER_SEC);

 return NO_MATCH;
}


//###########################################################
/*!\brief Scan directory for something
 * \param		fileid	A fileid you got from Fopen() or other functions
 * \param		startcluster	first cluster number of the sub dir
 * \return 		FULL_MATCH if successfull, NO_MATCH if not
 */
#ifdef USE_FAT32
 U8 ScanDirectory(FileID fileid,U32 startcluster)
#else
 U8 ScanDirectory(FileID fileid,U16 startcluster)
#endif
//###########################################################
{
 U32 tmpsector;

#ifdef USE_FAT32
 U32 tmpcluster;
#else
 U16 tmpcluster;
#endif

 U8 i,result;

 result=NO_MATCH;

 if(startcluster < 2) // Is this a FAT12/FAT16 rootdirectory ?
  {
   tmpsector = FirstRootSector;

   i = RootDirSectors;
   do
    {
     result=ScanOneDirectorySector(fileid,tmpsector++);
     if(result!=NO_MATCH) break; //break sector loop
     i--;
    }while(i);
  }
 else // We are in a subdirectory
  {
   tmpcluster=startcluster;

   while(tmpcluster < endofclusterchain)
    {
     tmpsector=GetFirstSectorOfCluster(tmpcluster);

     i = secPerCluster;
     do
      {
       result=ScanOneDirectorySector(fileid,tmpsector++);
       if(result!=NO_MATCH) break; //break sector loop
       i--;
      }while(i);

     if(result!=NO_MATCH) break; //break cluster loop
     tmpcluster=GetNextClusterNumber(tmpcluster);
    }
   }

 return(result);
}

//###########################################################
/*!\brief Change a 8.3 DOS name to a direntry formatted name
 * \param		inname	8.3 DOS name as a string ("test.txt")
 * \param		fileid	A fileid you got from Fopen() or other functions
 * \return 		Nothing
 */
void MakeDirEntryName(char *inname, FileID fileid)
//###########################################################
{
 U8 by,i,j;
 char *po;
 struct FileDesc *fdesc;

 fdesc=&FileDescriptors[(U16)fileid];


 po=fdesc->FileName;
 i=11;
 while(i--) *po++ = ' '; //fill filename buffer with spaces

 po=fdesc->FileName;

 if(inname[0]=='.')
  {
   *po++ ='.';
   if(inname[1]=='.') *po  ='.'; //change to upper dir

   return;
  }

 i = 0;
 j = 0;
 do
  {
   by=inname[i++];
   if(by == 0) return;             // end of filename reached
   if(by == '.') po = &fdesc->FileName[8]; // extension reached
   else
    {
     *po++ = toupper(by);
     j++;
    }
  }while(j<11);

// for(i=0; i<11; i++) putchar(outname[i]);
}

//@}
