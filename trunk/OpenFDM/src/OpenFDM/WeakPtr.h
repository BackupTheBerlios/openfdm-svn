/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_WeakPtr_H
#define OpenFDM_WeakPtr_H

#include "OpenFDMConfig.h"
#include "WeakReferenced.h"

namespace OpenFDM {

/// FIXME: remove the direct accessors, only copy to a SharedPtr
/// where you can access then, may be similar to the std::tr2::weak_ptr::lock()
/// function. That is to avoid deletion of a currently used object
template<typename T>
class WeakPtr {
public:
  WeakPtr(void)
  { }
  WeakPtr(T* ptr) // OpenFDM_DEPRECATED
  { assign(ptr); }
  WeakPtr(const WeakPtr& p) : mWeakDataPtr(p.mWeakDataPtr)
  { }
  template<typename U>
  WeakPtr(const SharedPtr<U>& p)
  { assign(p.ptr()); }
  template<typename U>
  WeakPtr(const WeakPtr<U>& p)
  { assign(p.ptr()); }
  ~WeakPtr(void)
  { }
  
  template<typename U>
  WeakPtr& operator=(const SharedPtr<U>& p)
  { assign(p.ptr()); return *this; }
  template<typename U>
  WeakPtr& operator=(const WeakPtr<U>& p)
  { assign(p.ptr()); return *this; }
  WeakPtr& operator=(T* p) // OpenFDM_DEPRECATED
  { assign(p); return *this; }
  WeakPtr& operator=(const WeakPtr& p)
  { mWeakDataPtr = p.mWeakDataPtr; return *this; }

  SharedPtr<T> lock(void) const
  {
    SharedPtr<T> sharedPtr;
    if (mWeakDataPtr)
       mWeakDataPtr->get(sharedPtr);
    return sharedPtr;
  }

  T* operator->(void) const // OpenFDM_DEPRECATED
  { return ptr(); }

  T& operator*(void) const // OpenFDM_DEPRECATED
  { return *ptr(); }

  operator T*(void) const // OpenFDM_DEPRECATED
  { return ptr(); }

private:
  T* ptr(void) const // OpenFDM_DEPRECATED
  {
    return lock().release();
  }

  void assign(T* p)
  {
    if (p)
      mWeakDataPtr = p->mWeakDataPtr;
    else
      mWeakDataPtr = 0;
  }
  
  // The indirect reference itself.
  SharedPtr<WeakReferenced::WeakData> mWeakDataPtr;

  template<typename U>
  friend class SharedPtr;
  template<typename U>
  friend class WeakPtr;
};

} // namespace OpenFDM

#endif
