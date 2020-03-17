#ifndef DEBUG_HEADER
#define DEBUG_HEADER

#include <assert.h>

#define DMSG(a) _PRNT(String() + a )
#define DMSGLN(a) _PRNT(String() + a + '\n')

#endif //DEBUG_HEADER

#ifdef DEBUG
#define _PRNT(a) Serial.print(a); Serial.flush();
#else
#define _PRNT(...) //discard arguments, generates no statements
#undef assert
#define assert(...)
#endif
