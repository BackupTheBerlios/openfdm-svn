/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Function.h"
#include "ODESolver.h"
#include "ImplicitEuler.h"
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

bool
ImplicitEuler::integrate(real_type toTEnd)
{
  IEFunction iefun(this);

  Vector fState(mState.size());
  fState.clear();

  unsigned dim = mModel->getNumContinousStates();

  while (!reached(toTEnd)) {
    real_type h = maxStepsize(toTEnd);

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
  return true;
}

void
ImplicitEuler::invalidateHistory(void)
{
  mJacStepsize = 0;
}

} // namespace OpenFDM
