/*! \file dos.c \brief DOS-Functions */
//###########################################################
///	\ingroup multifat
///	\defgroup DOS DOS-Functions (dos.c)
///	\code #include "dos.h" \endcode
///	\code #include "mmc_spi.h" \endcode
///	\par Uebersicht
//###########################################################
// Benutzung auf eigene Gefahr !
//
// Use at your own risk !
//
// If you like it fast compile with -O2
// If you like it small compile with -Os
//
// 24.03.2010 Removed bug in Fseek()
//
// 07.11.2007 Started LFN support for some routines.
//
// 29.09.2007 Started Fseek() for files open for writing.
//            Flag F_WRITE 'w' does not spool to end of file anymore !
//            You have to use flag F_APPEND 'a' for this now.
//
// 01.10.2007 Changed FileCurrentSector to FileFirstClusterSector.
//            Add FileClusterSectorOffset every time we need to.
//
// 10.08.2007 Remove() can delete directorys now.
//            Take care that you only delete EMPTY directorys.
//            Remove() does not clean up for you !
//
// 28.06.2007 Fseek() from Alex ???.
//            Seeking is only available for files open for reading !
//
//#########################################################################
// Last change: 24.03.2010
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

#include "dos.h"

struct FileDesc FileDescriptors[MAX_OPEN_FILE];

//###################################################################
/*!\brief Give back actual size of an open file
 * \param		fileid	A fileid you got from Fopen()
 * \return 		Filesize of the file
 */
U32 Filelength(FileID fileid)
//###################################################################
{
 struct FileDesc *fdesc;

 if(fileid<0 || fileid>=MAX_OPEN_FILE) return 0; //invalid filehandle

 fdesc=&FileDescriptors[(U16)fileid];

// if(fdesc->FileFlag==0) return 0; //file is closed

 return fdesc->FileSize;
}

#ifdef DOS_WRITE
#ifdef DOS_DELETE
//###########################################################
/*!\brief Delete a file/dir
 * \param		name	DOS 8.3 name of the file/dir
 * \return 		F_OK if file/dir could be deleted, F_ERROR if not
 */
U8 Remove(char *name)
//###########################################################
{
 #ifdef USE_REMOVE_LONG
  return RemoveLFN(name);
 #else
  return Remove83(name);
 #endif
}

//###########################################################
U8 Remove83(char *name)
//###########################################################
{
#ifdef USE_FAT32
 U32 tmp,tmp1;
#else
 U16 tmp,tmp1;
#endif
 struct FileDesc *fdesc;

  FileID fileid=findfreefiledsc();

  if (fileid==-1) return F_ERROR ; //too many open files

  fdesc=&FileDescriptors[(U16)fileid];

 //Test if directoryname exists in current directory

  MakeDirEntryName(name,fileid);

  if(FindEntry(fileid)!=FULL_MATCH)
   {
    fdesc->FileFlag=0; // Close this filedescriptor
    return F_OK;  //file not found, so nothing to remove
   }

  if(fdesc->FileAttr != ATTR_FILE && fdesc->FileAttr != ATTR_DIRECTORY)
   {
    fdesc->FileFlag=0; // Close this filedescriptor
    return F_ERROR; // this was not a file/dir !
   }

#ifdef STRICT_FILESYSTEM_CHECKING
// Bug found by Michele Ribaudo
   if(fdesc->FileFirstCluster>=2) // Zero length files made by Win have no cluster chain !
    {
#endif

     tmp=fdesc->FileFirstCluster;

     // free clusters in FAT cluster chain (make zero)
     do
      {
       tmp1=GetNextClusterNumber(tmp); // save next cluster number
       WriteClusterNumber(tmp,0);      // free cluster
       tmp=tmp1;                       // restore next cluster number
      }while(tmp<endofclusterchain);

#ifdef STRICT_FILESYSTEM_CHECKING
    }
#endif

#ifdef USE_FATBUFFER
   if(FATStatus>0)
    {
     WriteSector(FATCurrentSector,fatbuf); // write the FAT buffer
     FATStatus=0;
    }
#endif

   fdesc->FileName[0]=0xE5;   //mark file as deleted. does not affect long filename entrys !
   fdesc->FileSize=0;         //make filesize 0
   fdesc->FileFirstCluster=0; //delete first cluster
   fdesc->FileAttr=0;         //is this necessary ?

   UpdateFileEntry(fileid);

  fdesc->FileFlag=0; // Close this filedescriptor

 return F_OK;
}
#endif //DOS_DELETE
#endif //DOS_WRITE

#ifdef DOS_WRITE
//###########################################################
/*!\brief Write to a file
 * \param		buf	Buffer containing your data
 * \param		count	Number of bytes to be written
 * \param		fileid	A fileid you got from Fopen()
 * \return 		Number of bytes written to the file
 *
 * If number of bytes written is 0, your disk is full.

 Fwrite() does not write to CF until a sector is completely
 filled. You can force writing with Fflush() when file should
 keep open, or close the file with Fclose().

*/
//###########################################################
U16 Fwrite(U8 *buf, U16 count, FileID fileid)
//###########################################################
{
 struct FileDesc *fdesc;
 U8 secoffset;
 U32 tmp;
 U16 buffoffset;

 U16 tmp2;
 U16 remaining;
 U16 bytecount;

 if(fileid<0 || fileid>=MAX_OPEN_FILE) return 0; //invalid filehandle

 fdesc=&FileDescriptors[(U16)fileid];

 if(fdesc->FileFlag!=F_WRITE) return 0; //don't write if file is closed
                                 //or open for reading only !

 bytecount=0;
 remaining = count;

 while(remaining)
  {
//   tmp=fdesc->FileSize;         //next write position
   tmp = fdesc->FilePosition;         //next write position
   tmp -= fdesc->FileClusterCount; //calc byte position in cluster

   if(tmp >= BytesPerCluster)//is position in new cluster ?
    {
     FlushWriteBuffer(fileid); // Write old sector if it is not saved til now !

     if(fdesc->FilePosition == fdesc->FileSize) // we are at end of file, get a new cluster
      {
       fdesc->FileCurrentCluster = AllocCluster(fdesc->FileCurrentCluster); //alloc new cluster
       if(fdesc->FileCurrentCluster == DISK_FULL)
        {
         return bytecount; //return number of bytes written before disk full
        }
      }
     else // we are not at end of file (maybe fseek), so use next cluster
      {
       fdesc->FileCurrentCluster = GetNextClusterNumber(fdesc->FileCurrentCluster);
      }


     fdesc->FileFirstClusterSector = GetFirstSectorOfCluster(fdesc->FileCurrentCluster);//set new 1st sector of cluster
     fdesc->FileClusterSectorOffset = 0;

     if(fdesc->FilePosition < fdesc->FileSize) // fseek !?
      {
       ReadSector(fdesc->FileFirstClusterSector,fdesc->iob); //read new sector used
      }

     fdesc->FileClusterCount += BytesPerCluster;   //update cluster count
     tmp -= BytesPerCluster;               //calc new byte position in cluster
    }

   buffoffset=(U16)tmp; //we loose upper bits here, but it does not matter !

//   secoffset=(U8)(buffoffset / BYTE_PER_SEC);  //calc offset from first sector in cluster
   secoffset=(U8)(buffoffset >> 9 );  //calc offset from first sector in cluster

   if(fdesc->FileClusterSectorOffset != secoffset)
    {
     FlushWriteBuffer(fileid); // Write old sector if it is not saved til now !

     fdesc->FileClusterSectorOffset = secoffset;

     if(fdesc->FilePosition < fdesc->FileSize) // fseek !?
      {
       ReadSector(fdesc->FileFirstClusterSector + fdesc->FileClusterSectorOffset,fdesc->iob); //read new sector used
      }
    }

//   buffoffset=(U16)(tmp % BYTE_PER_SEC); //calc offset in sector
   buffoffset = buffoffset & (BYTE_PER_SEC - 1); // tmp % 512 => tmp & (512-1)

   if((buffoffset + remaining) <= BYTE_PER_SEC) // all bytes can be copied to current sector buffer
    {
     tmp2 = remaining;
     remaining = 0; // All bytes copied
    }
  else // All bytes can NOT be copied to current sector buffer. Copy as much as we can
   {   // to fill the buffer to the end. Then do the loop again with remaining bytes.
     tmp2 = BYTE_PER_SEC-buffoffset;
     remaining -= tmp2;
    }

  bytecount += tmp2;
  fdesc->FilePosition += tmp2; //update Filesize
  if(fdesc->FilePosition > fdesc->FileSize) fdesc->FileSize = fdesc->FilePosition;

   U8 *p;
   p = &fdesc->iob[buffoffset];

   while(tmp2--) *p++ = *buf++;

/*
  do // a little bit faster,smaller than while(tmp2--)
   {
    *p++ = *buf++;
    tmp2--;
   }while(tmp2);
*/

   fdesc->FileWriteBufferDirty=1; // Write Buffer contains data to be written !

  }// while(remaining)

 return bytecount;
}
#endif //DOS_WRITE

#ifdef DOS_READ
//###########################################################
/*!\brief Read from a file
 * \param		buf	Buffer where your data should be stored
 * \param		count	Number of bytes to be read
 * \param		fileid	A fileid you got from Fopen()
 * \return 		Number of bytes read from the file
 *
 * If number of bytes read is smaller as count or equal 0, end of file is reached.
 */
U16 Fread(U8 *buf, U16 count, FileID fileid)
//###########################################################
{
 struct FileDesc *fdesc;
 U8 secoffset;
 U32 tmp;
 U16 buffoffset;

 U16 tmp2;
 U16 remaining;
 U16 bytecount;

 if(fileid<0 || fileid>=MAX_OPEN_FILE) return 0; //invalid filehandle

 fdesc=&FileDescriptors[(U16)fileid];

 if(fdesc->FileFlag == 0) return 0; //don't read if file is closed

 bytecount=0;
 remaining=count;

 tmp = fdesc->FileSize - fdesc->FilePosition; // get bytes til end of file
 if(tmp < remaining) remaining = tmp; // smaller as we want to read, then read til end of file

 while(remaining)
  {
   tmp=fdesc->FilePosition;

   tmp-= fdesc->FileClusterCount; //calc byte position in cluster

   if(tmp >= BytesPerCluster)//is position in current cluster ?
    {
#ifdef DOS_WRITE
     if(fdesc->FileFlag==F_WRITE) FlushWriteBuffer(fileid); // Write old sector if it is not saved til now !
#endif

     fdesc->FileCurrentCluster = GetNextClusterNumber(fdesc->FileCurrentCluster); //if not get next cluster
     fdesc->FileFirstClusterSector = GetFirstSectorOfCluster(fdesc->FileCurrentCluster);//set new 1st sector of cluster
     ReadSector(fdesc->FileFirstClusterSector,fdesc->iob); //read new sector
     fdesc->FileClusterSectorOffset = 0;

     fdesc->FileClusterCount += BytesPerCluster;   //update cluster count
     tmp -= BytesPerCluster;               //calc new byte position in cluster
    }

     buffoffset=(U16)tmp; //we loose upper bits here, but it does not matter !

//     secoffset=(U8)(buffoffset / BYTE_PER_SEC);  //calc sector offset from first sector in cluster
     secoffset=(U8)(buffoffset >> 9);  //calc sector offset from first sector in cluster

     if(fdesc->FileClusterSectorOffset != secoffset) //new sector ?
      {
#ifdef DOS_WRITE
     if(fdesc->FileFlag==F_WRITE) FlushWriteBuffer(fileid); // Write old sector if it is not saved til now !
#endif

     fdesc->FileClusterSectorOffset = secoffset;
     ReadSector(fdesc->FileFirstClusterSector + fdesc->FileClusterSectorOffset,fdesc->iob); //read new sector
      }

//   buffoffset=(U16)(tmp % BYTE_PER_SEC); //calc offset in sector
     buffoffset = buffoffset & (BYTE_PER_SEC - 1); // tmp % 512 => tmp & (512-1)

    if((buffoffset + remaining) <= BYTE_PER_SEC) // All bytes can be copied from current sector buffer
     {
      tmp2 = remaining;
      remaining = 0; // All bytes copied
     }
    else // all bytes can NOT be copied from current sector buffer
     {
       tmp2 = BYTE_PER_SEC - buffoffset;
       remaining -= tmp2;
     }

    bytecount += tmp2;
    fdesc->FilePosition += tmp2;

    U8 *p;
    p = &fdesc->iob[buffoffset];

    while(tmp2--) *buf++ = *p++;

/*
   do // a little bit faster,smaller than while(tmp2--)
    {
     *buf++ = *p++;
     tmp2--;
    }while(tmp2);
*/

  } // while(remaining)

 return bytecount;
}
#endif //DOS_READ

//###########################################################
/*!\brief Find a free file descriptor
 * \return 		0 to MAX_OPENFILE-1 if free descriptor is found, -1 if not.
 *
 * Don't call this function directly !
 */
//look for a free file descriptor
//returns -1 if not found
//returns 0..(MAX_OPEN_FILE-1) if found
FileID findfreefiledsc(void)
//###########################################################
{
   FileID i;
   struct FileDesc *fdesc;

   for (i=0;i<MAX_OPEN_FILE;i++)
   {
      if (FileDescriptors[i].FileFlag==0)
       {
         fdesc=&FileDescriptors[i];

         fdesc->FileFlag=0xFF;   // make this filedescriptor ;-) open
	 fdesc->FileDirOffset=0;
//	 fdesc->FileCurrentSector=0;
         fdesc->FileFirstClusterSector=0;
         fdesc->FileClusterSectorOffset = 0;
	 fdesc->FileDirSector=0;
	 fdesc->FileSize=0;
	 fdesc->FilePosition=0;
	 fdesc->FileAttr=0;
	 fdesc->FileFirstCluster=0;
	 fdesc->FileCurrentCluster=0;
	 fdesc->FileClusterCount=0;
#ifdef DOS_WRITE
	 fdesc->FileWriteBufferDirty=0;
#endif
         return i;
       }
   }
   return (-1);
}

//###########################################################
/*!\brief Open a file
 * \param		name	8.3 DOS name
 * \param		flag	F_READ or F_WRITE
 * \return 		A fileid (0 to MAX_OPEN_FILE-1) if successfull, otherwise -1
 *
 */
//###########################################################
FileID Fopen(char *name, U8 flag)
//###########################################################
{
 #ifdef USE_FOPEN_LONG
  return FopenLFN(name,flag);
 #else
  return Fopen83(name,flag);
 #endif
}

//###########################################################
FileID Fopen83(char *name, U8 flag)
//###########################################################
{

 FileID fileid;
 struct FileDesc *fdesc;

 fileid=findfreefiledsc();
 if (fileid==(-1)) return -1;

 fdesc=&FileDescriptors[(U16)fileid];

// fdesc->FileClusterCount=0;

 MakeDirEntryName(name,fileid); //Split into name and extension

#ifdef DOS_READ
 if(flag==F_READ)
  {
   if((FindEntry(fileid))==FULL_MATCH) //file MUST exist for reading
    {
     if(fdesc->FileAttr != ATTR_FILE)
      {
       fdesc->FileFlag=0; // Close this filedescriptor
       return -1; // this was not a file !
      }

#ifdef STRICT_FILESYSTEM_CHECKING
// Bug found by Michele Ribaudo
     if(fdesc->FileFirstCluster<2) // no clusters allocated for the file !
      {
       fdesc->FileFlag=0; // Close this filedescriptor
       return -1; // nothing to read from this file
      }

#endif

     if(fdesc->FileSize==0) //nothing to read
      {
       fdesc->FileFlag=0; // Close this filedescriptor
       return -1; // nothing to read from this file
      }

     fdesc->FileFlag = F_READ; //needed for fclose

#ifdef DOS_FSEEK
     Fseek(0,SEEK_SET,fileid);
#else
     fdesc->FilePosition = 0; //actual read position
     fdesc->FileCurrentCluster = fdesc->FileFirstCluster;

     fdesc->FileFirstClusterSector = GetFirstSectorOfCluster(fdesc->FileFirstCluster);
     fdesc->FileClusterSectorOffset = 0;

     ReadSector(fdesc->FileFirstClusterSector,fdesc->iob); //read first sector of file
#endif

     return fileid;        //give back filehandle number if successfull
    }
  }
#endif //DOS_READ

#ifdef DOS_WRITE
 if(flag==F_WRITE || flag==F_APPEND)
  {
   fdesc->FileWriteBufferDirty=0;

   //if file exists, open it
   if(FindEntry(fileid)==FULL_MATCH)
    {
     if(fdesc->FileAttr != ATTR_FILE)
      {
        fdesc->FileFlag=0; // Close this filedescriptor
        return -1; // this was not a file !
      }

#ifdef STRICT_FILESYSTEM_CHECKING
// Bug found by Michele Ribaudo
     if(fdesc->FileFirstCluster<2) // no clusters allocated for the file !
      {
        // todo: allocate a cluster
        // You can open this file if you do the following:
        // Remove() the file ! Make a new one with Fopen()
        fdesc->FileFlag=0; // Close this filedescriptor
        return -1;
      }
#endif

     fdesc->FileFlag=F_WRITE; //needed for fclose
                       //FileFlag becomes F_WRITE, even when opened with F_APPEND
#ifdef DOS_FSEEK

     if(flag==F_APPEND) Fseek(0,SEEK_END,fileid); // go to end of file if flag is append
     else               Fseek(0,SEEK_SET,fileid);

#else //#ifdef DOS_FSEEK

     U32 tmp;

     fdesc->FileCurrentCluster=fdesc->FileFirstCluster; //we need this if file is smaller as ONE cluster
     fdesc->FileClusterSectorOffset = 0;
     fdesc->FilePosition=0; //actual write position

     if(flag==F_APPEND) // go to end of file if flag is append
      {
       tmp=fdesc->FileFirstCluster;
       while(tmp<endofclusterchain) //go to end of cluster chain
        {
         tmp=GetNextClusterNumber(tmp);
         if(tmp<endofclusterchain)
          {
           fdesc->FileCurrentCluster=tmp;
           fdesc->FileClusterCount+=BytesPerCluster;
          }
        }

       tmp= fdesc->FileSize - fdesc->FileClusterCount; //get number of bytes in current cluster
       //fdesc->FileClusterSectorOffset = (U8)(tmp / BYTE_PER_SEC);
       fdesc->FileClusterSectorOffset = (U8)(tmp >> 9);
       fdesc->FilePosition = fdesc->FileSize; //actual write position
      }//if(flag==F_APPEND)

     fdesc->FileFirstClusterSector = GetFirstSectorOfCluster(fdesc->FileCurrentCluster);

     ReadSector(fdesc->FileFirstClusterSector + fdesc->FileClusterSectorOffset,fdesc->iob); //read first sector of file

#endif //#ifdef DOS_FSEEK
    }
   else //make a new file
    {
     fdesc->FileAttr=ATTR_FILE; //needed for MakeNewFileEntry() ! 18.10.2006
     if(MakeNewFileEntry(fileid)) //file does not exist, try to make new file in current directory
      {
       fdesc->FileFlag=F_WRITE; //needed for fclose
                       //FileFlag becomes F_WRITE, even when opened with F_APPEND
#ifdef DOS_FSEEK
       Fseek(0,SEEK_SET,fileid);
#else
       fdesc->FileCurrentCluster = fdesc->FileFirstCluster;
       fdesc->FileFirstClusterSector = GetFirstSectorOfCluster(fdesc->FileFirstCluster);
       fdesc->FileClusterSectorOffset = 0;
       fdesc->FilePosition=0; //actual write position

       ReadSector(fdesc->FileFirstClusterSector,fdesc->iob); //read first sector of file
#endif //#ifdef DOS_FSEEK
      }
     else
      {
       fdesc->FileFlag=0; // Close this filedescriptor
       return -1; //new file could not be made
      }
    }

   return fileid;        //give back filehandle number if successfull
  }//if(flag==F_WRITE)
#endif //DOS_WRITE

 fdesc->FileFlag=0; // Close this filedescriptor
 return -1; //something went wrong
}

//###########################################################
/*!\brief Close a file
 * \param		fileid	A fileid you got from Fopen()
 * \return 		Nothing
 */
void Fclose(FileID fileid)
//###########################################################
{
 struct FileDesc *fdesc;

 if (fileid<0 || fileid>=MAX_OPEN_FILE) return;

 fdesc=&FileDescriptors[(U16)fileid];

#ifdef DOS_READ
// if(fdesc->FileFlag==F_READ)
//  {
//  }
#endif //DOS_READ

#ifdef DOS_WRITE
 if(fdesc->FileFlag==F_WRITE)
  {
   Fflush(fileid);  //write last sector used to CF and update filesize, filetime
  }
#endif //DOS_WRITE

 fdesc->FileFlag=0; //a second fclose should do nothing
             //reading and writing disabled
}

#ifdef DOS_WRITE
//###########################################################
/*!\brief Flush a file buffer
 * \param		fileid	A fileid you got from Fopen()
 * \return 		Nothing
 *
 Force writing last data written into sectorbuffer to be
 stored into CF without Fclose(). Direntry will also be updated.
 */
void Fflush(FileID fileid)
//###########################################################
{
 struct FileDesc *fdesc;

 if (fileid<0 || fileid>=MAX_OPEN_FILE) return;

 fdesc=&FileDescriptors[fileid];

// if(fdesc->FileFlag==0) return; //don't write if file is closed
// if(fdesc->FileFlag==F_READ) return; //don't write if file is open for reading

 if(fdesc->FileFlag==F_WRITE)
  {

#ifdef USE_FATBUFFER
   if(FATStatus>0)
    {
     WriteSector(FATCurrentSector,fatbuf); // write the FAT buffer
     FATStatus=0;
    }
#endif

   FlushWriteBuffer(fileid);
   //update file entry filesize and date/time
   UpdateFileEntry(fileid);
  }
}
#endif //DOS_WRITE

#ifdef DOS_WRITE
//###########################################################
/*!\brief Flush all file buffers
 * \return 		Nothing
 */
// force writing last data written into sectorbuffers to be
// stored into CF without fclose(). direntrys for all open
// files will also be updated.
void fflush_all(void)
//###########################################################
{
   FileID i;
   for (i=0;i<MAX_OPEN_FILE;i++) Fflush(i);
}
#endif //DOS_WRITE


//############################################################
/*!\brief Search for a fileentry in current directory
 * \param		fileid	A fileid you got from findfreefiledsc()
 * \return 		FULL_MATCH if file exists, NO_MATCH if not
 *
 */
U8 FindEntry(FileID fileid)
//############################################################
{
 U8 result;

 if (fileid<=(-1))          return NO_MATCH; //don't search if too many files open
 if(fileid>=MAX_OPEN_FILE) return NO_MATCH; //no valid filehandle number

 result=ScanDirectory(fileid,FirstDirCluster);

 return result;
}

//############################################################
/*!\brief Search for a filename in current directory
 * \param		name
 * \return 		FULL_MATCH if file exists, NO_MATCH if not
 *
 */
U8 FindName(char *name)
//############################################################
{
 #ifdef USE_FINDNAME_LONG
  return FindNameLFN(name);
 #else
  return FindName83(name);
 #endif
}

//############################################################
U8 FindName83(char *name)
//############################################################
{
 U8 result;

 struct FileDesc *fdesc;

 FileID fileid=findfreefiledsc();

 if (fileid==-1) return NO_MATCH ; //too many open files

 fdesc=&FileDescriptors[(U16)fileid];

 //Test if name exists in current directory
 MakeDirEntryName(name,fileid);
 result=FindEntry(fileid);

 fdesc->FileFlag=0; // Close this filedescriptor
 return result;
}

#ifdef DOS_FSEEK
/*****************************************************************************/
// First version by Alex ??? 28.06.2007
/*!\brief Seek in a file
 * \param		offset	position to seek to (forward,backward)
 * \param		mode	start position from where to seek
 * \param		fileid	you got from Fopen()
 * \return 		0 if seeking successfull, >0 if not
 *
  You can not seek above FileSize.

  Parameters for mode:

  SEEK_SET	Seek from beginning of file. offset has to be positive.

  SEEK_CUR	Seek from current file pointer position. offset may be positive or negative.

  SEEK_END	Seek from end of file. offset has to be negative.
 */
/*****************************************************************************/
U8 Fseek(S32 offset, U8 mode, FileID fileid)
{
#ifdef USE_FAT32
 U32 numClusters, curCluster;
#else
 U16 numClusters, curCluster;
#endif

 struct FileDesc *fdesc;

 if(fileid<0 || fileid>=MAX_OPEN_FILE) return 0; //invalid filehandle

 fdesc=&FileDescriptors[(U16)fileid];

#ifdef DOS_WRITE
// Check if file write buffer is dirty and save it here before seeking
 if(fdesc->FileFlag == F_WRITE) FlushWriteBuffer(fileid);
#endif

 // calculate the new File Position
 switch (mode)
  {
   case SEEK_SET:
        if(offset<0) return 3; 		// don't seek below 0
        if(offset > fdesc->FileSize) return 4;   // don't seek above FileSize

        fdesc->FilePosition = offset;
    break;

   case SEEK_END: // offset has to be 0 or negative	!
        if(offset>0) return 5; 		// don't seek above FileSize
        if((fdesc->FileSize + offset) < 0) return 6; 		// don't seek below 0

        fdesc->FilePosition = fdesc->FileSize + offset; // offset is negativ !
    break;

   case SEEK_CUR:
        if((fdesc->FilePosition + offset) < 0) return 7;	// don't seek below 0
        if((fdesc->FilePosition + offset) > fdesc->FileSize) return 8;	// don't seek above FileSize

        fdesc->FilePosition = fdesc->FilePosition + offset;
    break;

   default:    return 9;
  }

 // test if new FilePosition exists
// if(fdesc->FilePosition > fdesc->FileSize) return F_ERROR; // Notbremse

// numSector = (fdesc->FilePosition / BYTE_PER_SEC) % secPerCluster;
// numSector = (fdesc->FilePosition >> 9) % secPerCluster;

// secPerCluster is always power of 2 and <=128 !
 fdesc->FileClusterSectorOffset = ((U8)(fdesc->FilePosition >> 9) & (secPerCluster-1) );

// numClusters = fdesc->FilePosition / (BYTE_PER_SEC*secPerCluster);
 numClusters = (fdesc->FilePosition >> 9) / secPerCluster;

 // calculate the current file cluster
 curCluster=fdesc->FileFirstCluster;
 fdesc->FileCurrentCluster=curCluster;

 // calculate the cluster address of the new Position (verkettete pointerliste)
 fdesc->FileClusterCount=0;
 while (numClusters > 0)
  {
   curCluster = GetNextClusterNumber(curCluster);
   if(curCluster < endofclusterchain)
    {
     fdesc->FileCurrentCluster = curCluster;
     fdesc->FileClusterCount += BytesPerCluster; //update cluster count
    }
   numClusters--;
  }

 // calculate the Sector address of the new Position
 fdesc->FileFirstClusterSector = GetFirstSectorOfCluster(fdesc->FileCurrentCluster);

 ReadSector(fdesc->FileFirstClusterSector + fdesc->FileClusterSectorOffset,fdesc->iob); //read current sector of file

 return 0;
}
#endif //DOS_FSEEK

#ifdef DOS_WRITE
 #ifdef DOS_RENAME
//###########################################################
/*!\brief Rename a file/dir
 * \param		OldName	DOS 8.3 name of the file/dir
 * \param		NewName	DOS 8.3 name of the file/dir
 * \return 		F_OK if file/dir could be renamed, F_ERROR if not
 */
U8 Rename(char *OldName, char *NewName)
//###########################################################
{
 FileID fileid;
 struct FileDesc *fdesc;

 fileid=findfreefiledsc();

 if (fileid==(-1)) return F_ERROR;

 fdesc = &FileDescriptors[(U16)fileid];

 // FAT does NOT like DOUBLE filenames !
 MakeDirEntryName(NewName,fileid); //Split into name and extension
 if(FindEntry(fileid)==FULL_MATCH)
  {
   fdesc->FileFlag=0; // Close filedescriptor
   return F_ERROR; // File already exists !
  }

 fdesc->FileAttr=0xFF; // test if we really find a file or a directory

 MakeDirEntryName(OldName,fileid); //Split into name and extension
 if(FindEntry(fileid)==FULL_MATCH)  //To fill some usefull variables...
  {
   if(fdesc->FileAttr != ATTR_FILE && fdesc->FileAttr != ATTR_DIRECTORY) return F_ERROR; // this was not a file/dir !

   MakeDirEntryName(NewName,fileid); //Split into name and extension
   UpdateFileEntry(fileid);

   fdesc->FileFlag=0; // Close filedescriptor
   return F_OK;
  }

 fdesc->FileFlag=0; // Close filedescriptor
 return F_ERROR; // OldName not found
}
 #endif
#endif

#ifdef DOS_WRITE
/*****************************************************************************/
/*!\brief Write the file buffer if necessary
 * \param		fileid yove got from Fopen()
 * \return 		nothing
 *
*/
void FlushWriteBuffer(FileID fileid)
/*****************************************************************************/
{
 struct FileDesc *fdesc;

 fdesc=&FileDescriptors[(U16)fileid];

   if(fdesc->FileWriteBufferDirty == 1)
    {
     WriteSector(fdesc->FileFirstClusterSector + fdesc->FileClusterSectorOffset,fdesc->iob); //write current sector of file
     fdesc->FileWriteBufferDirty=0;
    }
}
#endif

//@}

