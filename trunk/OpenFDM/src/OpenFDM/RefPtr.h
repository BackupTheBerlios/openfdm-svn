/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RefPtr_H
#define OpenFDM_RefPtr_H

#include "Referenced.h"

namespace OpenFDM {

template<typename T>
class shared_ptr;
template<typename T>
class managed_ptr;

template<typename T>
class shared_ptr {
public:
  shared_ptr(void) : _ptr(0)
  {}
  shared_ptr(T* ptr) : _ptr(ptr)
  { get(_ptr); }
  shared_ptr(const shared_ptr& p) : _ptr(p._ptr)
  { get(_ptr); }
  template<typename U>
  shared_ptr(const shared_ptr<U>& p) : _ptr(p._ptr)
  { get(_ptr); }
  template<typename U>
  shared_ptr(const managed_ptr<U>& p) : _ptr(p._ptr)
  { get(_ptr); }
  ~shared_ptr(void)
  { put(); }
  
  shared_ptr& operator=(const shared_ptr& p)
  { assign(p._ptr); return *this; }
  template<typename U>
  shared_ptr& operator=(const shared_ptr<U>& p)
  { assign(p._ptr); return *this; }
  template<typename U>
  shared_ptr& operator=(U* p)
  { assign(p); return *this; }
  template<typename U>
  shared_ptr& operator=(const managed_ptr<U>& p)
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
  friend class shared_ptr;
  template<typename U>
  friend class managed_ptr;
};

} // namespace OpenFDM

#endif
