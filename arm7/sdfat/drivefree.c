//###########################################################
// File: drivefree.c
//
// Get used and free memory of the drive.
//
// Counts only free and used CLUSTERS. If a file does not
// use all bytes in a cluster, this free space is NOT
// given back by this routines. This space can't be used
// by another file because the cluster is reserved.
//
// Only the free space of the unused clusters will be given back.
// So you get the MINIMUM free memory. This is the same way WIN
// gives you free or used space.
//
// Don't use this functions without a FAT buffer !
// Or you will have to wait a LOOOOOONG time.
//
// With FAT buffer on a 256MB FAT16 CF and 62650 clusters:
// 2 seconds for drivefree() or driveused()
// ATMega128 at 16MHz
//
// On a 32GB harddrive with FAT32 and 2 Mio clusters: ??????????????????
//
// Benutzung auf eigene Gefahr !
//
// Use at your own risk !
//
//#########################################################################
// Last change: 18.12.2005
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

#ifdef USE_DRIVEFREE

//################################################
// Give back free memory of unused clusters in kB
U32 drivefree(void)
//################################################
{
#ifdef USE_FAT32
 U32 tmpcluster, count;
#else
 U16 tmpcluster, count;
#endif

 count=0;
 tmpcluster=2;

// search til end of FAT
 while(tmpcluster<maxcluster)
  {
   if(GetNextClusterNumber(tmpcluster)==0) count++;
   tmpcluster++;
  }
 
 return ((U32)count * secPerCluster) / 2;
}

//################################################
// Give back memory size of used clusters in kB
U32 driveused(void)
//################################################
{
 return drivesize() - drivefree();
}

//################################################
// Give back memory size of the drive in kB
U32 drivesize(void)
//################################################
{
 return ((U32)(maxcluster-2) * secPerCluster) / 2;
}

#endif //#ifdef USE_DRIVEFREE
//@}

