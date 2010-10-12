#ifndef rprintf_h_
#define rprintf_h_

extern void rprintf_devopen( int(*put)(int) );
extern void rprintf ( char const *format, ... );

#define printf(format, args...)   rprintf(format , ## args)

#endif
