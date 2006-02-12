/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_WeakReferenced_H
#define OpenFDM_WeakReferenced_H

#include "Referenced.h"
#include "SharedPtr.h"

namespace OpenFDM {

template<typename T>
class WeakPtr;

class WeakReferenced : public Referenced {
public:
  WeakReferenced(void) : mWeakDataPtr(new WeakData(this))
  {}
  /// Do not copy the weak backward references ...
  WeakReferenced(const WeakReferenced&) : mWeakDataPtr(new WeakData(this))
  {}
  ~WeakReferenced(void)
  { mWeakDataPtr->object = 0; }

  /// Do not copy the weak backward references ...
  WeakReferenced& operator=(const WeakReferenced& wr)
  { return *this; }

private:
  /// Support for weak references, not increasing the reference count
  /// that is done through that small helper class which holds an uncounted
  /// reference which is zeroed out on destruction of the current object
  struct WeakData : public Referenced {
    WeakData(WeakReferenced* o) : object(o) {}
    WeakReferenced* object;
  private:
    WeakData(void);
    WeakData(const WeakData&);
    WeakData& operator=(const WeakData&);
  };

  SharedPtr<WeakData> mWeakDataPtr;

  template<typename T>
  friend class WeakPtr;
};

} // namespace OpenFDM

#endif
