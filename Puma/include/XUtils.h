#include "XPrintf.h"

#ifdef _DEBUG
#undef NDEBUG
#include <cassert>
#define SAIAssert(fCondition) assert(fCondition)
#define SAIAssertSz(fCondition, sz) assert(fCondition)
#else
#define SAIAssert(fCondition)
#define SAIAssertSz(fCondition, sz)
#endif
