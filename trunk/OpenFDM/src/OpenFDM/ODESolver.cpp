/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "ODESolver.h"

#include "Assert.h"
#include "Math.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"

namespace OpenFDM {

ODESolver::ODESolver(void) :
  mStepsize(1.0/32)
{
}

ODESolver::~ODESolver(void)
{
}

bool
ODESolver::reached(real_type tEnd)
{
  real_type eps = Limits<real_type>::epsilon();
  bool reached = fabs(tEnd - mTime) < 4*eps*max(fabs(tEnd), fabs(mTime));
  if (reached)
    mTime = tEnd;
  return reached;
}

real_type
ODESolver::maxStepsize(real_type tEnd)
{
  return min(tEnd-mTime, mStepsize);
}

void
ODESolver::invalidateHistory(void)
{
}

bool
ODESolver::denseOutput(real_type t, Vector& out)
{
  // Simplest dense output you can imagine ...
  out = mState;
  return true;
}

} // namespace OpenFDM
