/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Mutex_H
#define OpenFDM_Mutex_H

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#endif

namespace OpenFDM {

class Mutex {
public:
  Mutex(void)
  {
#ifdef _WIN32
    InitializeCriticalSection((LPCRITICAL_SECTION)&mCriticalSection);
#else
    pthread_mutex_init(&mMutex, NULL);
#endif
  }
  ~Mutex(void)
  {
#ifdef _WIN32
    LeaveCriticalSection((LPCRITICAL_SECTION)&mCriticalSection);
#else
    pthread_mutex_destroy(&mMutex);
#endif
  }

  void lock(void)
  {
#ifdef _WIN32
    EnterCriticalSection((LPCRITICAL_SECTION)&mCriticalSection);
#else
    pthread_mutex_lock(&mMutex);
#endif
  }

  void unlock(void)
  {
#ifdef _WIN32
    LeaveCriticalSection((LPCRITICAL_SECTION)&mCriticalSection);
#else
    pthread_mutex_unlock(&mMutex);
#endif
  }

private:
  Mutex(const Mutex&);
  Mutex& operator=(const Mutex&);

#ifdef _WIN32
  CRITICAL_SECTION mCriticalSection;
#else
  pthread_mutex_t mMutex;
#endif
};

} // namespace OpenFDM

#endif
