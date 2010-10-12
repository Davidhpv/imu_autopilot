/*! \file lfn_util.c
    \brief Long-Filename-Functions
*/
//###########################################################
///	\ingroup multifat
///	\defgroup lfn Long-Filename-Functions (lfn_util.c)
///	\code #include "lfn_util.h" \endcode
///	\code #include "dir.h" \endcode
///	\code #include "dos.h" \endcode
///	\par Uebersicht
//###########################################################
//
// Some LFN (Long File Name) support.
// This does not give full LFN support. It is not possible
// to make a new file/dir with LFN or to rename a file with LFN.
// This routines can FIND LFN entrys, they can not CREATE LFNs.
//
// Only a few tests have been done. This is very buggy !
// For example: You want to write to "LongFilename.bin", but
// there is none. Then you will get a "LONGFILE.NAM" file.
// If you try a second time you will get a SECOND "LONGFILE.NAM" file.
// Win does not like two files with the same name.
//
// What you can do:
// Change to directories with LFN.
// Open files with LFN for reading.
// Open files with LFN for writing/appending data. If there is a file !
// Remove files/dirs with LFN (not tested).
//
// Benutzung auf eigene Gefahr !
//
// Use at your own risk !
//
//#########################################################################
// Last change: 23.05.2008
//#########################################################################
// hk@holger-klabunde.de
// http://www.holger-klabunde.de/index.html
//#########################################################################
//#########################################################################
//@{
#include <string.h>

#include "dos.h"
//#include "rprintf.h"

char dosname[13];

#if defined (USE_LISTDIR_LONG) && defined (USE_FINDFILE) && defined (USE_FINDLONG)
//###########################################################
void ListDirectory(void)
//###########################################################
{
 U8 result;

   printf("Directory Listing\n");
   printf("=================\n");

   if(Findfirst()!=0) //find FIRST direntry in directory
    {
     do
      {
       if(ffblk.ff_attr==ATTR_FILE || ffblk.ff_attr==ATTR_DIRECTORY) //did we find a file or dir ?
        {
           if(ffblk.ff_attr==ATTR_FILE) printf("FIL ");
           if(ffblk.ff_attr==ATTR_DIRECTORY) printf("DIR ");

           //do we have a long filename ?
           if(ffblk.ff_longname[0]!=0)
            {
             printf("%s | %s",ffblk.ff_longname, ffblk.ff_name);
             printf("\t\t");
             }
           else
            {
             printf("%s",ffblk.ff_name);    //print 8.3 DOS name
             printf("\t\t\t");
            }

         printf("Size: % 9lu",ffblk.ff_fsize);
//         printf("\tCluster: % 9lu",(unsigned long)FileFirstCluster);
         printf("\n");
        }//if(ffblk.ff_attr==ATTR_FILE....

       result=Findnext(); //find next directory entry

      }while(result!=0);  //test all files in directory

    } //if(Findfirst()!=0) //find FIRST file in directory

}
#endif

#if defined (USE_FINDNAME_LONG) && defined (USE_FINDFILE) && defined (USE_FINDLONG)
//###########################################################
///!\brief Find a file/dir with LFN in current directory by name.
/// * \return FULL_MATCH if file/dir exists, NO_MATCH if not
/// *
/// * Always starts to search from the beginning of a directory !
/// *
///
U8 FindNameLFN(char *filename)
//###########################################################
{
 U8 result;

//   printf("FindNameLFN %s\n",filename);

   if(Findfirst()!=0) //find FIRST direntry in directory
    {
     do
      {
       if(ffblk.ff_attr==ATTR_FILE || ffblk.ff_attr==ATTR_DIRECTORY) //did we find a file or dir ?
        {
           //do we have a long filename ?
           if(ffblk.ff_longname[0]!=0)
            {
//             printf("Findname %s | %s\n",ffblk.ff_longname,filename); //print long name

             if(strcasecmp(ffblk.ff_longname,filename)==0)
              {
//               printf("Found by LFN name\n"); // Never saw this message !
               return FULL_MATCH; // Filename found
              }
            }
// We never come here if there is a long filename !
//           else
//            {
//             printf("Findname %s | %s\n",ffblk.ff_name,filename);    //print 8.3 DOS name

//             if(strcasecmp(ffblk.ff_name,filename)==0)
//              {
//               printf("Found by 8.3 name\n");
//               return FULL_MATCH; // Filename found
//              }
//            }

        }//if(ffblk.ff_attr==ATTR_FILE....

       result=Findnext(); //find next directory entry

      }while(result!=0);  //test all files in directory

    } //if(Findfirst()!=0) //find FIRST file in directory

 // This has to stay here holgi ! It's for looking for the short name of a LFN
 if(FindName83(filename)==FULL_MATCH)
  {
//   printf("Found by 8.3 name\n");
   return FULL_MATCH; // Filename found
  }

// printf("Not found !\n");
 return NO_MATCH; // File not found
}
#endif //#if defined (USE_FINDNAME_LONG) && defined (USE_FINDFILE)


#if defined (USE_CHDIR_LONG) && defined (USE_FINDFILE) && defined (USE_FINDLONG)
//###########################################################
/*!\brief Change to a directory with LFN in current directory
 * \return F_OK if changed to directory, F_ERROR if not
 *
 * Always starts to search from the beginning of a directory !
 *
 */
U8 ChdirLFN(char *dirname)
//###########################################################
{
 U8 result;

//   printf("ChdirLFN %s\n",dirname);

   if(Findfirst()!=0) //find FIRST direntry in directory
    {
     do
      {
       if(ffblk.ff_attr==ATTR_DIRECTORY) //did we find a file ?
        {
           //do we have a long filename ?
           if(ffblk.ff_longname[0]!=0)
            {
//             printf("ChdirLFN %s | %s\n",ffblk.ff_longname,dirname); //print long name

             if(strcasecmp(ffblk.ff_longname,dirname)==0)
              {
               // ffblk.ff_name is changed by most FAT routines !
               // You have to use a copy of it.
               strcpy(dosname,ffblk.ff_name);
//               printf("ChdirLFN %s | %s\n",ffblk.ff_longname,dosname);
               result = Chdir83(dosname); // directory found
//               if(result==F_ERROR) printf("Dir not found\n");
               return result;
              }
            }

        }//if(ffblk.ff_attr==ATTR_DIRECTORY

       result=Findnext(); //find next directory entry

      }while(result!=0);  //test all files in directory

    } //if(Findfirst()!=0) //find FIRST file in directory

 // If we come here, no long name found.
 // So try a short name
 result = Chdir83(dirname);

// if(result==F_ERROR) printf("Dir not found\n");

 return result;
// return F_ERROR; // directory not found
}
#endif //#if defined (USE_CHDIR_LONG) && defined (USE_FINDFILE)

#if defined (USE_FOPEN_LONG) && defined (USE_FINDFILE) && defined (USE_FINDLONG)
//###########################################################
/*!\brief Open a file with LFN in current directory
 * \return F_OK if file is open,F_ERROR if not
 *
 * Always starts to search from the beginning of a directory !
 *
 * It is not possible to create new files with FopenLFN().
 *
 */
FileID FopenLFN(char *filename, U8 fileflag)
//###########################################################
{
 U8 result;

//   printf("FopenNameLFN %s\n",filename);

   if(Findfirst()!=0) //find FIRST direntry in directory
    {
     do
      {
       if(ffblk.ff_attr==ATTR_FILE) //did we find a file ?
        {
           //do we have a long filename ?
           if(ffblk.ff_longname[0]!=0)
            {
//             printf("FopenLFN %s | %s\n",ffblk.ff_longname,filename); //print long name

             if(strcasecmp(ffblk.ff_longname,filename)==0)
              {
               // ffblk.ff_name is changed by most FAT routines !
               // You have to use a copy of it.
               strcpy(dosname,ffblk.ff_name);
//               printf("FopenLFN %s | %s\n",ffblk.ff_longname,ffblk.ff_name);
               return Fopen83(dosname,fileflag); // Filename found
              }
            }

        }//if(ffblk.ff_attr==ATTR_FILE

       result=Findnext(); //find next directory entry

      }while(result!=0);  //test all files in directory

    } //if(Findfirst()!=0) //find FIRST file in directory

// If we come here, file does not exist
//  if(fileflag=='r') return -1; // file not found

// Todo:
// If fileflag == 'w' or 'a' create LFN entrys
// and 8.3 entry. Then Fopen() the 8.3 entry.
// To make things easier, always make LFN entrys, even
// if filename would fit in a 8.3 name.

//Quick hack: If file with LFN does not exist jump out of here
//This does not solve all problems !
 if(strlen(filename)>12) return -1; // File not found

 return Fopen83(filename,fileflag); // This will only work for 8.3 filenames.
                                  // If it was a LFN, it may give strange results !

// return -1; // File not found
}
#endif //#if defined (USE_FOPEN_LONG) && defined (USE_FINDFILE)


#if defined (USE_REMOVE_LONG) && defined (USE_FINDFILE) && defined (USE_FINDLONG)
//###########################################################
/*!\brief Remove a file/dir with LFN in current directory
 * \return F_OK if file/dir is removed,F_ERROR if not
 *
 * Always starts to search from the beginning of a directory !
 *
 * Does not remove LFN entrys in directory ! Only removes
 * 8.3 entry. Have to test if this is ok.
 *
 */
U8 RemoveLFN(char *filename)
//###########################################################
{
 U8 result;

//   printf("RemoveLFN %s\n",filename);

   if(Findfirst()!=0) //find FIRST direntry in directory
    {
     do
      {
       if(ffblk.ff_attr==ATTR_FILE || ffblk.ff_attr==ATTR_DIRECTORY) //did we find a file or dir ?
        {
           //do we have a long filename ?
           if(ffblk.ff_longname[0]!=0)
            {
//             printf("RemoveLFN %s | %s\n",ffblk.ff_longname,filename); //print long name

             if(strcasecmp(ffblk.ff_longname,filename)==0)
              {
               // ffblk.ff_name is changed by most FAT routines !
               // You have to use a copy of it.
               strcpy(dosname,ffblk.ff_name);
//               printf("RemoveLFN %s | %s\n",ffblk.ff_longname,ffblk.ff_name);
               return Remove83(dosname); // Filename found
              }
            }
/*
           else
            {
             if(strcasecmp(ffblk.ff_name,filename)==0)
              {
               strcpy(dosname,ffblk.ff_name);
               return Remove83(dosname); // Filename found
              }
            }
*/

        }//if(ffblk.ff_attr==ATTR_FILE....

       result=Findnext(); //find next directory entry

      }while(result!=0);  //test all files in directory

    } //if(Findfirst()!=0) //find FIRST file in directory

 return Remove83(filename);

// return F_ERROR; // File not found
}
#endif //#if defined (USE_REMOVE_LONG) && defined (USE_FINDFILE)


//@}


