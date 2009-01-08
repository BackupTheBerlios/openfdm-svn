/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "ExplicitEuler.h"
#include "Object.h"
#include "Vector.h"
#include "ODESolver.h"

namespace OpenFDM {

ExplicitEuler::ExplicitEuler(void)
{
}

ExplicitEuler::~ExplicitEuler(void)
{
}

bool
ExplicitEuler::integrate(real_type toTEnd)
{
  while (!reached(toTEnd)) {
    real_type t = getTime();
    real_type h = maxStepsize(toTEnd);

    evalFunction(t, getState(), mDeriv);
    mState += h*mDeriv;
    mTime += h;
  }
  return true;
}

bool
ExplicitEuler::denseOutput(real_type t, Vector& out)
{
  // Do linear interpolation
  out = mState - (mTime - t)*mDeriv;
  return true;
}

} // namespace OpenFDM
