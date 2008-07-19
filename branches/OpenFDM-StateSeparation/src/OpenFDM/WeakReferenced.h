/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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
  WeakReferenced(void) :
    mWeakData(new WeakData(this))
  {}
  WeakReferenced(const WeakReferenced& weakReferenced) :
    mWeakData(new WeakData(this))
  {}
  ~WeakReferenced(void)
  { mWeakData->mWeakReferenced = 0; }

  /// Do not copy the weak backward references ...
  WeakReferenced& operator=(const WeakReferenced&)
  { return *this; }

  /// The usual operations on weak pointers.
  /// The interface should stay the same then what we have in Referenced.
  static unsigned get(const WeakReferenced* ref)
  { if (ref) return ++(ref->mWeakData->mRefcount); else return 0u; }
  static unsigned put(const WeakReferenced* ref)
  { if (ref) return --(ref->mWeakData->mRefcount); else return ~0u; }
  static unsigned count(const WeakReferenced* ref)
  { if (ref) return ref->mWeakData->mRefcount; else return 0u; }

  template<typename T>
  static void destroy(T* ref)
  { delete ref; }

private:
  /// Support for weak references, not increasing the reference count
  /// that is done through that small helper class which holds an uncounted
  /// reference which is zeroed out on destruction of the current object
  class WeakData : public Referenced {
  public:
    WeakData(WeakReferenced* weakReferenced) :
      mRefcount(0u),
      mWeakReferenced(weakReferenced)
    { }

    template<typename T>
    T* getPointer()
    {
      // Try to increment the reference count if the count is greater
      // then zero. Since it should only be incremented iff it is nonzero, we
      // need to check that value and try to do an atomic test and set. If this
      // fails, try again. The usual lockless algorithm ...
      unsigned count;
      do {
        count = mRefcount;
        if (count == 0)
          return 0;
      } while (!mRefcount.compareAndExchange(count, count + 1));
      // We know that as long as the refcount is not zero, the pointer still
      // points to valid data. So it is safe to work on it.
      return static_cast<T*>(mWeakReferenced);
    }

    Atomic mRefcount;
    WeakReferenced* mWeakReferenced;

  private:
    WeakData(void);
    WeakData(const WeakData&);
    WeakData& operator=(const WeakData&);
  };

  SharedPtr<WeakData> mWeakData;

  template<typename T>
  friend class WeakPtr;
};

} // namespace OpenFDM

#endif
