/*! \file find_x.c \brief Directory Find-Functions */
//###########################################################
///	\ingroup multifat
///	\defgroup find Directory Find-Functions (find_x.c)
///	\code #include "dos.h" \endcode
///	\par Uebersicht
//###########################################################
// Find first file in a directory and give back filename as
// a NULL terminated string. Also fileattr and filesize.
//
// ffblk.ff_name[]   	8.3 DOS name with '.' in it and \0 at the end
// ffblk.ff_longname[]  Long filename with \0 at the end
// ffblk.ff_attr     	ATTR_FILE or ATTR_DIRECTORY
// ffblk.ff_fsize    	Filesize, 0 if directory
//
// Use this data to find next file, next file, ... in a directory.
//
// Necessary to make a "dir" or "ls" command. Or opening files
// with Fopen() without knowing the filename of the first,next,next... file.
//
// This doesn't work without initialized global variable FirstDirCluster
// which points to the current directory.
//
// 11.08.2007 Most code simply merged into dir.c
//
// 26.12.2005 Found a bug in constructing the long name. If it had exactly
//            13 Bytes, parts of the last Findnext() are in ffblk.ff_longname. 
//
// Benutzung auf eigene Gefahr !
//
// Use at your own risk !
//
//#########################################################################
// Last change: 11.08.2007
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

#ifdef USE_FINDFILE

struct FindFile ffblk; 	//make this global to avoid parameter passing


//###########################################################
// finds the first file or dir entry in current directory
// returns 0 if no file is found
// returns 1 if a file is found
//
/*!\brief Find first directory entry (file or directory)
 * \return 		0 if nothing found, 1 if something found
 *
 * check ffblk.ff_attr if you have a file or a dir entry
 */
unsigned char Findfirst(void)
//###########################################################
{
 ffblk.lastposition=0;                	//no position
 return Findnext();
}

//###########################################################
// finds the next file or dir entry in current directory
//
/*!\brief Find next directory entry (file or directory)
 * \return 		0 if nothing found, 1 if something found
 * 
 * Check ffblk.f_attr if you have a file or a dir entry.
 * Always starts to search from beginning of a directory.
 * NEVER call this before calling Findfirst() !
 * Your program crashes and your hardware will be destroyed.
 * 
 * Findfirst(), Findnext(), Findnext()....
 *
 * If you change to another directory you have to call Findfirst() first again !
 */
unsigned char Findnext(void)
//###########################################################
{
 U8 i;

 //delete last data
 for(i=0; i<13; i++) ffblk.ff_name[i]=0; //clean last filename

#ifdef USE_FINDLONG
 for(i=0; i<_MAX_NAME; i++) ffblk.ff_longname[i]=0;	// clean the long filename.
#endif

 ffblk.ff_fsize=0; 	      		//no filesize
 ffblk.ff_attr=ATTR_NO_ATTR; 	      	//no file attr
 ffblk.newposition=0;  	//no position for next search

 if(ScanDirectory(-1, FirstDirCluster) == FULL_MATCH) return 1;

 return 0;
}

#endif //USE_FINDFILE
//@}
