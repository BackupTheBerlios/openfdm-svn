/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Atomic_H
#define OpenFDM_Atomic_H

#if defined(__GNUC__) && (4 <= __GNUC__) && (1 <= __GNUC_MINOR__)
// No need to include something. Is a Compiler API ...
# define OpenFDM_USE_GCC4_BUILTINS
#elif defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
// Neat old Win32 functions
# include <windows.h>
# define OpenFDM_USE_WIN32_INTERLOCKED
#else
// The sledge hammer ...
# include "Mutex.h"
# include "ScopedLock.h"
#endif

namespace OpenFDM {

class Atomic {
public:
  Atomic(unsigned value = 0) : mValue(value)
  { }
  unsigned operator++()
  {
#if defined(OpenFDM_USE_GCC4_BUILTINS)
    return __sync_add_and_fetch(&mValue, 1);
#elif defined(OpenFDM_USE_WIN32_INTERLOCKED)
    return InterlockedIncrement(reinterpret_cast<long volatile*>(&mValue));
#else
    ScopedLock lock(mMutex);
    return ++mValue;
#endif
  }
  unsigned operator--()
  {
#if defined(OpenFDM_USE_GCC4_BUILTINS)
    return __sync_sub_and_fetch(&mValue, 1);
#elif defined(OpenFDM_USE_WIN32_INTERLOCKED)
    return InterlockedDecrement(reinterpret_cast<long volatile*>(&mValue));
#else
    ScopedLock lock(mMutex);
    return --mValue;
#endif
  }
  operator unsigned() const
  {
#if defined(OpenFDM_USE_GCC4_BUILTINS)
    __sync_synchronize();
    return mValue;
#elif defined(OpenFDM_USE_WIN32_INTERLOCKED)
    return static_cast<unsigned const volatile &>(mValue);
#else
    ScopedLock lock(mMutex);
    return mValue;
#endif
 }

private:
  Atomic(const Atomic&);
  Atomic& operator=(const Atomic&);

#if !defined(OpenFDM_USE_GCC4_BUILTINS) && !defined(OpenFDM_USE_WIN32_INTERLOCKED)
  mutable Mutex mMutex;
#endif
  unsigned mValue;
};

}  // namespace OpenFDM

#endif
