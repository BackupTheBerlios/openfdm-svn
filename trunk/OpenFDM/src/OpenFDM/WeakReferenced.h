/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_WeakReferenced_H
#define OpenFDM_WeakReferenced_H

#include "Mutex.h"
#include "Referenced.h"
#include "ScopeLock.h"
#include "SharedPtr.h"

namespace OpenFDM {

template<typename T>
class WeakPtr;

class WeakReferenced : public Referenced {
public:
  /// Note that object construction is atomic anyway.
  /// This way we do not need to guard setting the mWeakDataPtr variable
  WeakReferenced(void) : mWeakDataPtr(new WeakData(this))
  {}
  // Do not copy the weak backward references ...
  WeakReferenced(const WeakReferenced& weakReferenced) :
    Referenced(weakReferenced), mWeakDataPtr(new WeakData(this))
  {}
  ~WeakReferenced(void)
  { mWeakDataPtr->clear(); }

  /// Do not copy the weak backward references ...
  WeakReferenced& operator=(const WeakReferenced&)
  { Referenced::operator=(*this); return *this; }

private:
  /// Support for weak references, not increasing the reference count
  /// that is done through that small helper class which holds an uncounted
  /// reference which is zeroed out on destruction of the current object
  class WeakData : public Referenced {
  public:
    WeakData(WeakReferenced* object) : mObject(object) {}
    template<typename T>
    T* get()
    {
      // This lock guarantees that the object is not deleted in between.
      ScopeLock scopeLock(mMutex);
      if (WeakReferenced::getNonZero(mObject) == 0)
        return 0;
      else
        return static_cast<T*>(mObject);
    }
    void clear()
    {
      ScopeLock scopeLock(mMutex);
      mObject = 0;
    }
  private:
    WeakData(void);
    WeakData(const WeakData&);
    WeakData& operator=(const WeakData&);

    // Mutex to protect the non atomic pointer assignment and that avoids
    // access to an object that is not yet zeroed out but deletion is in
    // progress.
    Mutex mMutex;
    WeakReferenced* mObject;
  };

  SharedPtr<WeakData> mWeakDataPtr;

  template<typename T>
  friend class WeakPtr;
};

} // namespace OpenFDM

#endif
