/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "ImplicitEuler.h"

#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Function.h"
#include "ODESolver.h"
#include "Newton.h"

namespace OpenFDM {

ImplicitEuler::ImplicitEuler(void)
{
  Log(TimeStep, Warning) << "Using mostly hacked implicit Euler method!!!"
                             << endl;
  mJacStepsize = 0;
}

ImplicitEuler::~ImplicitEuler(void)
{
}

void
ImplicitEuler::invalidateHistory(void)
{
  mJacStepsize = 0;
}

bool
ImplicitEuler::integrate(real_type toTEnd)
{
  IEFunction iefun(this);

  Vector fState(mState.size());
  fState.clear();

  unsigned dim = mSystem->getNumContinousStates();

  real_type h = 0;
  while (!reached(toTEnd)) {
    h = maxStepsize(toTEnd);

    mCurrentStepsize = h;
    if (mJacStepsize != mCurrentStepsize) {
      iefun.jac(mTime+h, fState /* = 0 */, mJac);
      mJacDecomp = mJac;
      mJacStepsize = mCurrentStepsize;
    }

    if (mJacDecomp.singular())
      Log(TimeStep, Warning) << "Have singular jacobian!" << endl;
    bool conv = Newton(iefun, mJacDecomp, mTime+h, fState, 1e-4, 1e-10);
    if (!conv)
      Log(TimeStep, Warning) << "Have singular jacobian!" << endl;

    mState += fState;
    mTime += h;
  }

  // Save that for dense output
  if (h == 0)
    mDeriv.clear();
  else
    mDeriv = 1/h*fState;

  return true;
}

bool
ImplicitEuler::denseOutput(real_type t, Vector& out)
{
  // Do linear interpolation
  out = mState - (mTime - t)*mDeriv;
  return true;
}

} // namespace OpenFDM
