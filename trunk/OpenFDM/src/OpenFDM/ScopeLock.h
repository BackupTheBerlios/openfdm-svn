/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ScopeLock_H
#define OpenFDM_ScopeLock_H

#include "Mutex.h"

namespace OpenFDM {

class ScopeLock {
public:
  ScopeLock(Mutex& mutex) : mMutex(mutex)
  { mMutex.lock(); }
  ~ScopeLock(void)
  { mMutex.unlock(); }

private:
  ScopeLock(void);
  ScopeLock(const ScopeLock&);
  ScopeLock& operator=(const ScopeLock&);

  Mutex& mMutex;
};

} // namespace OpenFDM

#endif
