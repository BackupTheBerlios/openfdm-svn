/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Mutex_H
#define OpenFDM_Mutex_H

#include "OpenFDMConfig.h"

#ifndef OPENFDM_DISABLE_THREADS
#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#endif
#endif

namespace OpenFDM {

class Mutex {
public:
  Mutex(void)
  {
#ifndef OPENFDM_DISABLE_THREADS
#ifdef _WIN32
    InitializeCriticalSection((LPCRITICAL_SECTION)&mCriticalSection);
#else
    pthread_mutex_init(&mMutex, NULL);
#endif
#endif
  }
  ~Mutex(void)
  {
#ifndef OPENFDM_DISABLE_THREADS
#ifdef _WIN32
    LeaveCriticalSection((LPCRITICAL_SECTION)&mCriticalSection);
#else
    pthread_mutex_destroy(&mMutex);
#endif
#endif
  }

  void lock(void)
  {
#ifndef OPENFDM_DISABLE_THREADS
#ifdef _WIN32
    EnterCriticalSection((LPCRITICAL_SECTION)&mCriticalSection);
#else
    pthread_mutex_lock(&mMutex);
#endif
#endif
  }

  void unlock(void)
  {
#ifndef OPENFDM_DISABLE_THREADS
#ifdef _WIN32
    LeaveCriticalSection((LPCRITICAL_SECTION)&mCriticalSection);
#else
    pthread_mutex_unlock(&mMutex);
#endif
#endif
  }

private:
  Mutex(const Mutex&);
  Mutex& operator=(const Mutex&);

#ifndef OPENFDM_DISABLE_THREADS
#ifdef _WIN32
  CRITICAL_SECTION mCriticalSection;
#else
  pthread_mutex_t mMutex;
#endif
#endif
};

} // namespace OpenFDM

#endif
