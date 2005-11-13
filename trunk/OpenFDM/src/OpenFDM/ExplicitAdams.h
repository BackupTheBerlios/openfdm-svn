/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ExplicitAdams_H
#define OpenFDM_ExplicitAdams_H

#include "Object.h"
#include "ODESolver.h"

namespace OpenFDM {

class ExplicitAdams
  : public ODESolver {
public:
  ExplicitAdams(void);
  virtual ~ExplicitAdams(void);

  void setMaxOrder(unsigned order);
  unsigned getMaxOrder(void) const
  { return mMaxOrder; }

  virtual void invalidateHistory(void);

  /**
     Note on the starting ramp:
     We just use a series of lower order adams methods for the starting ramp.
     That brings in fact a reduction of accuracy when we compare that method
     with different methods of higher order started with the same initial
     value. In our case (the in air case) these errors will most propably get
     damped out by the huge amount of friction in our physical model.
     Or in other words: small errors in the integration half an hour ago will
     affect the result now only very little.
     But the local error we accumulate when the ramp has finished is O(h^5).
     This strategy might be sufficient for the in air case. For comparison of
     methods with a given starting value and little friction, this errors will
     show up in the end result and thus destroy the order.
   */
  virtual bool integrate(real_type toTEnd);

private:
  enum { MaxAvailOrder = 4 };

  unsigned index(unsigned backidx) const
  { return backidx % MaxAvailOrder; }
 
  unsigned mOrder;
  unsigned mStepNumber;
  unsigned mMaxOrder;
  real_type mHistoryStepSize;
  Vector mFEvals[MaxAvailOrder];
  real_type mCoefs[MaxAvailOrder][4];
};

} // namespace OpenFDM

#endif
