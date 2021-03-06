/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_WeakPtr_H
#define OpenFDM_WeakPtr_H

#include "OpenFDMConfig.h"
#include "WeakReferenced.h"

namespace OpenFDM {

template<typename T>
inline bool operator==(const WeakPtr<T>& p1, const WeakPtr<T>& p2);
template<typename T>
inline bool operator<(const WeakPtr<T>& p1, const WeakPtr<T>& p2);

template<typename T>
class WeakPtr {
public:
  WeakPtr(void)
  { }
  WeakPtr(const WeakPtr& p) : mWeakData(p.mWeakData)
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
  { mWeakData = p.mWeakData; return *this; }

  SharedPtr<T> lock(void) const
  {
    if (!mWeakData)
      return SharedPtr<T>();
    SharedPtr<T> sharedPtr;
    sharedPtr.assignNonRef(static_cast<T*>(mWeakData->getWeakReferenced()));
    return sharedPtr;
  }

  void clear()
  { mWeakData = 0; }
  void swap(WeakPtr& weakPtr)
  { mWeakData.swap(weakPtr.mWeakData); }

private:
  void assign(T* p)
  {
    if (p)
      mWeakData = p->mWeakData;
    else
      mWeakData = 0;
  }
  
  // The indirect reference itself.
  SharedPtr<WeakReferenced::WeakData> mWeakData;

  template<typename S>
  friend bool operator==(const WeakPtr<S>& p1, const WeakPtr<S>& p2);
  template<typename S>
  friend bool operator<(const WeakPtr<S>& p1, const WeakPtr<S>& p2);
};

template<typename T>
inline bool
operator==(const WeakPtr<T>& p1, const WeakPtr<T>& p2)
{ return p1.mWeakData == p2.mWeakData; }

template<typename T>
inline bool
operator!=(const WeakPtr<T>& p1, const WeakPtr<T>& p2)
{ return !(p1 == p2); }

template<typename T>
inline bool
operator<(const WeakPtr<T>& p1, const WeakPtr<T>& p2)
{ return p1.mWeakData < p2.mWeakData; }

template<typename T>
inline bool
operator>(const WeakPtr<T>& p1, const WeakPtr<T>& p2)
{ return p2 < p1; }

template<typename T>
inline bool
operator<=(const WeakPtr<T>& p1, const WeakPtr<T>& p2)
{ return !(p1 > p2); }

template<typename T>
inline bool
operator>=(const WeakPtr<T>& p1, const WeakPtr<T>& p2)
{ return !(p1 < p2); }

} // namespace OpenFDM

#endif
