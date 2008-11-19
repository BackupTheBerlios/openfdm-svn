/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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

class ImplicitEuler::IEFunction : public Function {
public:
  IEFunction(ImplicitEuler* i) : ie(i) {}
  
  virtual size_type inSize(void) const
  { return ie->mFunction->inSize(); }
  virtual size_type outSize(void) const
  { return ie->mFunction->outSize(); }
  virtual void eval(real_type t, const invector_type& v, outvector_type& out)
  {
    real_type h = ie->mCurrentStepsize;
    ie->evalFunction(ie->getTime() + h, ie->getState() + v, out);
    out -= (1/h)*v;
  }
  virtual void jac(real_type t, const invector_type& v, jacobian_type& jac)
  {
    real_type h = ie->mCurrentStepsize;
    ie->evalJacobian(ie->getTime() + h, ie->getState() + v, jac);
    size_type dim = ie->mFunction->inSize();
    jac -= (1/h)*LinAlg::Eye<real_type,0,0>(dim, dim);
  }
private:
  ImplicitEuler* ie;
};

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

  unsigned dim = mFunction->inSize();

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
