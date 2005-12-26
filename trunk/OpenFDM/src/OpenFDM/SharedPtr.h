/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SharedPtr_H
#define OpenFDM_SharedPtr_H

#include "Referenced.h"

namespace OpenFDM {

template<typename T>
class SharedPtr;
template<typename T>
class WeakPtr;

/// FIXME make const correct ...
template<typename T>
class SharedPtr {
public:
  SharedPtr(void) : _ptr(0)
  {}
  SharedPtr(T* ptr) : _ptr(ptr)
  { get(_ptr); }
  SharedPtr(const SharedPtr& p) : _ptr(p._ptr)
  { get(_ptr); }
  template<typename U>
  SharedPtr(const SharedPtr<U>& p) : _ptr(p._ptr)
  { get(_ptr); }
  template<typename U>
  SharedPtr(const WeakPtr<U>& p) : _ptr(p._ptr)
  { get(_ptr); }
  ~SharedPtr(void)
  { put(); }
  
  SharedPtr& operator=(const SharedPtr& p)
  { assign(p._ptr); return *this; }
  template<typename U>
  SharedPtr& operator=(const SharedPtr<U>& p)
  { assign(p._ptr); return *this; }
  template<typename U>
  SharedPtr& operator=(U* p)
  { assign(p); return *this; }
  template<typename U>
  SharedPtr& operator=(const WeakPtr<U>& p)
  { assign(p._ptr); return *this; }

  T* operator->(void)
  { return _ptr; }
  const T* operator->(void) const
  { return _ptr; }
  T& operator*(void)
  { return *_ptr; }
  const T& operator*(void) const
  { return *_ptr; }

  operator T*(void)
  { return _ptr; }
  operator const T*(void) const
  { return _ptr; }

  bool isShared(void) const
  { return Referenced::shared(_ptr); }
  unsigned getNumRefs(void) const
  { return Referenced::count(_ptr); }

private:
  template<typename U>
  void assign(U* p)
  { get(p); put(); _ptr = p; }

  template<typename U>
  void get(const U* p) const
  { Referenced::get(p); }
  void put(void)
  { if (!Referenced::put(_ptr)) { delete _ptr; _ptr = 0; } }
  
  // The reference itself.
  T* _ptr;

  template<typename U>
  friend class SharedPtr;
  template<typename U>
  friend class WeakPtr;
};

} // namespace OpenFDM

#endif