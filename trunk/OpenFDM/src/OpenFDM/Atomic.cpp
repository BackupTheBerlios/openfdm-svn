/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Atomic.h"

#if defined(OpenFDM_USE_GCC4_BUILTINS) && defined(__i386__)

// Usually the apropriate functions are inlined by gcc.
// But if gcc is called with something aequivalent to -march=i386,
// it will not assume that there is a lock instruction and instead
// calls this pair of functions. We will provide them here in this case.
// Note that this assembler code will not work on a i386 chip anymore.
// But I hardly believe that we can assume to run at least on a i486 ...

extern "C" {

unsigned __sync_sub_and_fetch_4(volatile void *ptr, unsigned value)
{
  register volatile unsigned* mem = reinterpret_cast<volatile unsigned*>(ptr);
  register unsigned result;
  __asm__ __volatile__("lock; xadd{l} {%0,%1|%1,%0}"
                       : "=r" (result), "=m" (*mem)
                       : "0" (-value), "m" (*mem)
                       : "memory");
  return result - value;
}

unsigned __sync_add_and_fetch_4(volatile void *ptr, unsigned value)
{
  register volatile unsigned* mem = reinterpret_cast<volatile unsigned*>(ptr);
  register unsigned result;
  __asm__ __volatile__("lock; xadd{l} {%0,%1|%1,%0}"
                       : "=r" (result), "=m" (*mem)
                       : "0" (value), "m" (*mem)
                       : "memory");
  return result + value;
}

void __sync_synchronize()
{
  __asm__ __volatile__("": : : "memory");
}

} // extern "C"

#endif
