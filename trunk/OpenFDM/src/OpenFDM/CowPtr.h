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
protected:
  explicit CowPtr(T* ptr) : mPtr(ptr)
  { OpenFDMAssert(mPtr); }

  T* ptr(void)
  {
    if (mPtr->isShared())
      mPtr = mPtr->clone();
    return mPtr;
  }

  const T* ptr(void) const
  { return mPtr; }

private:
  SharedPtr<T> mPtr;
};

} // namespace OpenFDM

#endif
