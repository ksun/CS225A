#ifndef XPRINTF_H
#define XPRINTF_H

// Override global printf with an application-specific function.

#include <cstdarg>
#include <cstdio>
int XPrintf( const char* fmt, ... );
#define printf XPrintf

#endif // XPRINTF_H
