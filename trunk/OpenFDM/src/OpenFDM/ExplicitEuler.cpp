/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Object.h"
#include "Vector.h"
#include "ODESolver.h"
#include "ExplicitEuler.h"

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
  Vector deriv;
  while (!reached(toTEnd)) {
    real_type t = getTime();
    real_type h = maxStepsize(toTEnd);

    evalFunction(t, getState(), deriv);
    mState += h*deriv;
    mTime += h;
  }
  return true;
}

} // namespace OpenFDM
