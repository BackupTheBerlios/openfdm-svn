/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SharedPtr_H
#define OpenFDM_SharedPtr_H

#include "OpenFDMConfig.h"
#include "Referenced.h"

namespace OpenFDM {

template<typename T>
class WeakPtr;

template<typename T>
class SharedPtr {
public:
  SharedPtr(void) : _ptr(0)
  {}
  SharedPtr(T* ptr) : _ptr(ptr)
  { T::get(_ptr); }
  SharedPtr(const SharedPtr& p) : _ptr(p.get())
  { T::get(_ptr); }
  template<typename U>
  SharedPtr(const SharedPtr<U>& p) : _ptr(p.get())
  { T::get(_ptr); }
  ~SharedPtr(void)
  { put(); }
  
  SharedPtr& operator=(const SharedPtr& p)
  { assign(p.get()); return *this; }
  template<typename U>
  SharedPtr& operator=(const SharedPtr<U>& p)
  { assign(p.get()); return *this; }
  template<typename U>
  SharedPtr& operator=(U* p)
  { assign(p); return *this; }

  T* operator->(void) const
  { return _ptr; }

  T& operator*(void) const
  { return *_ptr; }

  operator T*(void) const
  { return _ptr; }

  T* get() const
  { return _ptr; }
  T* release()
  { T* tmp = _ptr; _ptr = 0; T::put(tmp); return tmp; }

  bool isShared(void) const
  { return T::shared(_ptr); }
  unsigned getNumRefs(void) const
  { return T::count(_ptr); }

  void clear()
  { put(); }
  void swap(SharedPtr& sharedPtr)
  { T* tmp = _ptr; _ptr = sharedPtr._ptr; sharedPtr._ptr = tmp; }

private:
  void assign(T* p)
  { T::get(p); put(); _ptr = p; }
  void assignNonRef(T* p)
  { put(); _ptr = p; }

  void put(void)
  { if (!T::put(_ptr)) T::destroy(_ptr); _ptr = 0; }
  
  // The reference itself.
  T* _ptr;

  template<typename U>
  friend class WeakPtr;
};

// Hmmm, what if we get an automatic cast to a shared pointer of a fresh
// allocated object pointer?
// This will be gone after destruction of the temporary object ...
// The same applies to the < operator below ...

// template<typename T>
// inline bool
// operator==(const SharedPtr<T>& sharedPtr0, const SharedPtr<T>& sharedPtr1)
// { return sharedPtr0.get() == sharedPtr1.get(); }

// template<typename T>
// inline bool
// operator<(const SharedPtr<T>& sharedPtr0, const SharedPtr<T>& sharedPtr1)
// { return sharedPtr0.get() < sharedPtr1.get(); }

} // namespace OpenFDM

#endif
