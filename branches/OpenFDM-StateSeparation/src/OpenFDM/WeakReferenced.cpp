/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "WeakReferenced.h"

#include "Referenced.h"
#include "ScopeLock.h"
#include "SharedPtr.h"

namespace OpenFDM {

WeakReferenced::WeakData::WeakData(WeakReferenced* weakReferenced) :
  mRefcount(lastbit()),
  mWeakReferenced(weakReferenced)
{
}

WeakReferenced::WeakData::~WeakData()
{
}

void
WeakReferenced::WeakData::weakReferencedGetFirst()
{
  unsigned count = mRefcount;
  for (;;) {
    unsigned newcount = (count & (~lastbit())) + 1;
    if (mRefcount.compareAndExchange(count, newcount))
      return;
    count = mRefcount;
  }
}

void
WeakReferenced::WeakData::weakReferencedRelease()
{
  unsigned count = mRefcount;
  for (;;) {
    unsigned newcount = count - 1;
    if (newcount == 0)
      newcount |= lastbit();
    if (mRefcount.compareAndExchange(count, newcount))
      return;
    count = mRefcount;
  }
}

WeakReferenced*
WeakReferenced::WeakData::getWeakReferenced()
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
  return mWeakReferenced;
}

} // namespace OpenFDM
