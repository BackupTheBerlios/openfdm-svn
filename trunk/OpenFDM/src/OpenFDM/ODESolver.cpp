/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Math.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "ODESolver.h"

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
  bool reached = abs(tEnd - mTime) < 4*eps*max(abs(tEnd), abs(mTime));
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

} // namespace OpenFDM
