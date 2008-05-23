/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_WeakPtr_H
#define OpenFDM_WeakPtr_H

#include "OpenFDMConfig.h"
#include "WeakReferenced.h"

namespace OpenFDM {

template<typename T>
class WeakPtr {
public:
  WeakPtr(void)
  { }
  WeakPtr(const WeakPtr& p) : mWeakDataPtr(p.mWeakDataPtr)
  { }
  template<typename U>
  WeakPtr(const SharedPtr<U>& p)
  { SharedPtr<T> sharedPtr = p; assign(sharedPtr.get()); }
  template<typename U>
  WeakPtr(const WeakPtr<U>& p)
  { SharedPtr<T> sharedPtr = p.lock(); assign(sharedPtr.get()); }
  WeakPtr(T* ptr) // OpenFDM_DEPRECATED // Hmm, shall we??
  { assign(ptr); }
  ~WeakPtr(void)
  { }
  
  template<typename U>
  WeakPtr& operator=(const SharedPtr<U>& p)
  { SharedPtr<T> sharedPtr = p; assign(sharedPtr.get()); return *this; }
  template<typename U>
  WeakPtr& operator=(const WeakPtr<U>& p)
  { SharedPtr<T> sharedPtr = p.lock(); assign(sharedPtr.get()); return *this; }
  WeakPtr& operator=(const WeakPtr& p)
  { mWeakDataPtr = p.mWeakDataPtr; return *this; }

  SharedPtr<T> lock(void) const
  {
    if (!mWeakDataPtr)
      return SharedPtr<T>();
    SharedPtr<T> sharedPtr;
    sharedPtr.assignNonRef(mWeakDataPtr->get<T>());
    return sharedPtr;
  }

  void clear()
  { mWeakDataPtr = 0; }
  void swap(WeakPtr& weakPtr)
  { mWeakDataPtr.swap(weakPtr.mWeakDataPtr); }

private:
  void assign(T* p)
  {
    if (p)
      mWeakDataPtr = p->mWeakDataPtr;
    else
      mWeakDataPtr = 0;
  }
  
  // The indirect reference itself.
  SharedPtr<WeakReferenced::WeakData> mWeakDataPtr;
};

} // namespace OpenFDM

#endif
