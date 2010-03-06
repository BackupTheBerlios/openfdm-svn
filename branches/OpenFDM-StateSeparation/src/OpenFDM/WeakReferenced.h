/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich
 *
 */

#ifndef OpenFDM_WeakReferenced_H
#define OpenFDM_WeakReferenced_H

#include "Referenced.h"
#include "ScopeLock.h"
#include "SharedPtr.h"

namespace OpenFDM {

template<typename T>
class WeakPtr;

class WeakReferenced {
public:
  /// The object backref and the reference count for this object need to be
  /// there in any case. Also these are per object and shall not be copied nor
  /// assigned.
  /// The reference count for this object is stored in a secondary object that
  /// is shared with all weak pointers to this current object. This way we
  /// have an atomic decision using the reference count of this current object
  /// if the backref is still valid. At the time where the atomic count is
  /// equal to zero the object is considered dead.
  ///
  /// Special care is taken for objects that are not yet assigned to
  /// a shared pointer but weak pointers are already assigned and actions need
  /// to be taken with the weak referenced objects.
  /// To make this work, a new allocated object has its last bit set in the
  /// reference count. Consequently you can already call WeakRef::lock() and
  /// get back a valid object, even if we do not yet have assigned that to
  /// a SharedPtr. Once we assign the pointer to s SharedPtr through a raw
  /// pointer argument, this bit is cleared. This way, the object is not
  /// deleted until we have a more permanent reference to the object assigned.
  /// The following example shows the intented use of that:
  ///
  /// O* object = new Object;
  /// { // assume some scope that might be called in initialization of Object
  ///   WeakPtr<O> w = o;
  ///   SharedPtr<O> s = o.lock(); // valid
  /// }
  /// SharedPtr<O> permanent = object;
  ///
  /// Whereas the following code doesn't work as expected:
  ///
  /// WeakPtr<O> w = new Object;
  /// SharedPtr<O> s = w.lock();
  ///
  /// In this case the last bit is still set and the Object instance is
  /// never destroyed.
  ///
  WeakReferenced(void) :
    mWeakData(new WeakData(this))
  {}
  WeakReferenced(const WeakReferenced& weakReferenced) :
    mWeakData(new WeakData(this))
  {}
  ~WeakReferenced(void)
  { mWeakData->clear(); }

  /// Do not copy the weak backward references ...
  WeakReferenced& operator=(const WeakReferenced&)
  { return *this; }

  /// The usual operations on weak pointers.
  /// The interface should stay the same then what we have in Referenced.
  static void get(const WeakReferenced* ref)
  { if (!ref) return; ref->mWeakData->weakReferencedGet(); }
  static void getFirst(const WeakReferenced* ref)
  { if (!ref) return; ref->mWeakData->weakReferencedGetFirst(); }
  static unsigned put(const WeakReferenced* ref)
  { if (!ref) return ~0u; return ref->mWeakData->weakReferencedPut(); }
  static void release(const WeakReferenced* ref)
  { if (!ref) return; return ref->mWeakData->weakReferencedRelease(); }
  static unsigned count(const WeakReferenced* ref)
  { if (!ref) return ~0u; return ref->mWeakData->weakReferencedCount(); }

  template<typename T>
  static void destruct(T* ref)
  { delete ref; }

private:
  /// Support for weak references, not increasing the reference count
  /// that is done through that small helper class which holds an uncounted
  /// reference which is zeroed out on destruction of the current object
  class WeakData : public Referenced {
  public:
    WeakData(WeakReferenced* weakReferenced);
    ~WeakData();

    void weakReferencedGet()
    { ++mRefcount; }
    void weakReferencedGetFirst();

    unsigned weakReferencedPut()
    { return --mRefcount; }
    void weakReferencedRelease();
    unsigned weakReferencedCount()
    { return mRefcount & (~lastbit()); }

    void clear()
    { mWeakReferenced = 0; }

    WeakReferenced* getWeakReferenced();

    static unsigned lastbit()
    { return ~((~unsigned(0)) >> 1); }

  private:
    WeakData(void);
    WeakData(const WeakData&);
    WeakData& operator=(const WeakData&);

    Atomic mRefcount;
    WeakReferenced* mWeakReferenced;
  };

  SharedPtr<WeakData> mWeakData;

  template<typename T>
  friend class WeakPtr;
};

} // namespace OpenFDM

#endif
