/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_CowPtr_H
#define OpenFDM_CowPtr_H

#include "Assert.h"
#include "Referenced.h"
#include "SharedPtr.h"

namespace OpenFDM {

/// Helper template for a copy on write handle to some class.
/// The class cannot be used by itself, it is intendend to be a bease class
/// a copy on write handle can be derived from.
/// The template argument is assumed to be derived from Referenced and
/// it must implement a clone method.
/// You can use this class on a simple handle class forwarding it's accesses
/// to the original class as well as it can be used on a hierarchy of classes
/// implementing its destructors and clone method as a virtual class member.
template<typename T>
class CowPtr {
public:
  void detach()
  { mutablePtr(); }
  
protected:
  explicit CowPtr(T* ptr) : mPtr(ptr)
  { OpenFDMAssert(mPtr); }

  T* mutablePtr(void)
  {
    // Ok, asking here if the data is shared one time is suficient.
    // For the first cut, it might happen that just past shared returned
    // false a new reference is generated in an other thread.
    // That can not happen, since a reference in an other thread would
    // cause a reference count > 1 what means that we cannot use
    // the shared copy anyway and we have already made a copy anyway.
    if (mPtr.isShared())
      mPtr = new T(*mPtr);
    return mPtr;
  }

  const T* constPtr(void) const
  { return mPtr; }

private:
  SharedPtr<T> mPtr;
};

} // namespace OpenFDM

#endif
