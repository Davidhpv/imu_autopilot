//#########################################################################
// File: find_x.h
//
// Benutzung auf eigene Gefahr !
//
// Use at your own risk !
//
//#########################################################################
// Last change: 03.08.2007
//#########################################################################
//#########################################################################
//@{

#ifndef __FINDX_H
#define __FINDX_H

#include "typedefs.h"

#define _MAX_NAME	255		// Max. length of long filenames + 1.
					// This should be 256, but i dont want
					// to use an unsigned int.
					// Maybe 128 or 64 Bytes are also enough
        				// for a microcontroller DOS.
        				// Change it here if you have problems
        				// with free RAM.

struct FindFile
{
 	U8 ff_attr;		// file attributes like file, dir
 					// long name ,hidden,system and readonly flags are ignored
 	U32 ff_fsize;		// filesize of the file ( not directory ! )
	char ff_name[13];		// 8.3 DOS filename with '.' in it and \0 at the end for fopen()
#ifdef USE_FINDLONG
        char ff_longname[_MAX_NAME];	// The very long filename.
#endif
#ifdef USE_FAT32
 	U32 newposition;	// position of this file entry in current directory (1 means first file)
 	U32 lastposition;	// position of last file entry found in current directory (1 means first file)
#else
 	U16 newposition;	// position of this file entry in current directory (1 means first file)
 	U16 lastposition;	// position of last file entry found in current directory (1 means first file)
 					// does also count ".", ".." entrys !
 					// does not count long filename entrys and volume id
#endif
};

extern struct FindFile ffblk;

extern U8 Findfirst(void);
extern U8 Findnext(void);

#endif //__FINDX_H
//@}
