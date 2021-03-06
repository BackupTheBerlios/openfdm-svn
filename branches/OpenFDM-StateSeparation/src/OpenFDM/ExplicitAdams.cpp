/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "ExplicitAdams.h"
#include "Object.h"
#include "Vector.h"
#include "ODESolver.h"

namespace OpenFDM {

ExplicitAdams::ExplicitAdams(void)
{
  // Initialize the coeficients for the explicit adams methods
  mCoefs[0][0] =  real_type(1);

  mCoefs[1][0] =  real_type(3)/real_type(2);
  mCoefs[1][1] = -real_type(1)/real_type(2);

  mCoefs[2][0] =  real_type(23)/real_type(12);
  mCoefs[2][1] = -real_type(16)/real_type(12);
  mCoefs[2][2] =  real_type(5)/real_type(12);

  mCoefs[3][0] =  real_type(55)/real_type(24);
  mCoefs[3][1] = -real_type(59)/real_type(24);
  mCoefs[3][2] =  real_type(37)/real_type(24);
  mCoefs[3][3] = -real_type(9)/real_type(24);

  mOrder = 1;
  mMaxOrder = MaxAvailOrder;
  mHistoryStepSize = 0;
  mStepNumber = 0;
}

ExplicitAdams::~ExplicitAdams(void)
{
}

void
ExplicitAdams::setMaxOrder(unsigned order)
{
  OpenFDMAssert(0 < order);
  OpenFDMAssert(order <= MaxAvailOrder);
  mMaxOrder = order;
  mOrder = 1;
  mHistoryStepSize = 0;
}

void
ExplicitAdams::invalidateHistory(void)
{
  // That forces the adams method to discard all prevous function values.
  mOrder = 1;
  mHistoryStepSize = 0;
}

bool
ExplicitAdams::integrate(real_type toTEnd)
{
  while (!reached(toTEnd)) {
    real_type t = getTime();
    real_type h = maxStepsize(toTEnd);

    if (h != mHistoryStepSize)
      mOrder = 1;

    evalFunction(t, getState(), mFEvals[index(mStepNumber)]);
    for (unsigned i = 0; i < mOrder; ++i)
      mState += (h*mCoefs[mOrder-1][i])*mFEvals[index(mStepNumber-i)];

    mOrder = min(mOrder+1, mMaxOrder);
    ++mStepNumber;
    mHistoryStepSize = h;
    mTime += h;
  }
  return true;
}

} // namespace OpenFDM
