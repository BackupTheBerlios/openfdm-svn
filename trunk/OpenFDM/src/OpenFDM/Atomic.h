/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Atomic_H
#define OpenFDM_Atomic_H

#include "OpenFDMConfig.h"

#if defined(OpenFDM_USE_WIN32_INTERLOCKED)
# include <windows.h>
#elif defined(OpenFDM_USE_MUTEX)
# include "Mutex.h"
# include "ScopeLock.h"
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
#elif defined(OpenFDM_USE_MIPOSPRO_BUILTINS)
    return __add_and_fetch(&mValue, 1);
#elif defined(OpenFDM_USE_WIN32_INTERLOCKED)
    return InterlockedIncrement(reinterpret_cast<long volatile*>(&mValue));
#elif defined(OpenFDM_USE_MUTEX)
    ScopeLock lock(mMutex);
    return ++mValue;
#else
    return ++mValue;
#endif
  }
  unsigned operator--()
  {
#if defined(OpenFDM_USE_GCC4_BUILTINS)
    return __sync_sub_and_fetch(&mValue, 1);
#elif defined(OpenFDM_USE_MIPOSPRO_BUILTINS)
    return __sub_and_fetch(&mValue, 1);
#elif defined(OpenFDM_USE_WIN32_INTERLOCKED)
    return InterlockedDecrement(reinterpret_cast<long volatile*>(&mValue));
#elif defined(OpenFDM_USE_MUTEX)
    ScopeLock lock(mMutex);
    return --mValue;
#else
    return --mValue;
#endif
  }
  operator unsigned() const
  {
#if defined(OpenFDM_USE_GCC4_BUILTINS)
    __sync_synchronize();
    return mValue;
#elif defined(OpenFDM_USE_MIPOSPRO_BUILTINS)
    __synchronize();
    return mValue;
#elif defined(OpenFDM_USE_WIN32_INTERLOCKED)
    return static_cast<unsigned const volatile &>(mValue);
#elif defined(OpenFDM_USE_MUTEX)
    ScopeLock lock(mMutex);
    return mValue;
#else
    return mValue;
#endif
 }

private:
  Atomic(const Atomic&);
  Atomic& operator=(const Atomic&);

#if defined(OpenFDM_USE_MUTEX)
  mutable Mutex mMutex;
#endif
#if defined(OpenFDM_USE_WIN32_INTERLOCKED)
  __declspec(align(32))
#endif
  unsigned mValue;
};

}  // namespace OpenFDM

#endif
