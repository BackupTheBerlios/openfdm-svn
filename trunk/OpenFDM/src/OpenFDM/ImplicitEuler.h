/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ImplicitEuler_H
#define OpenFDM_ImplicitEuler_H

#include "Object.h"
#include "Matrix.h"
#include "Function.h"
#include "ODESolver.h"

namespace OpenFDM {

class ImplicitEuler
  : public ODESolver {
public:
  ImplicitEuler(void);
  virtual ~ImplicitEuler(void);

  virtual void invalidateHistory(void);
  virtual bool integrate(real_type toTEnd);
  virtual bool denseOutput(real_type t, Vector& out);

private:
  class IEFunction
    : public Function {
  public:
    IEFunction(ImplicitEuler* i) : ie(i) {}

    virtual size_type inSize(void) const
    { return ie->mSystem->getNumContinousStates(); }
    virtual size_type outSize(void) const
    { return ie->mSystem->getNumContinousStates(); }
    virtual void eval(real_type t, const invector_type& v,
                      outvector_type& out)
    {
      real_type h = ie->mCurrentStepsize;
      ie->evalFunction(ie->getTime() + h, ie->getState() + v, out);
      out -= (1/h)*v;
    }
    virtual void jac(real_type t, const invector_type& v, jacobian_type& jac)
    {
      real_type h = ie->mCurrentStepsize;
      ie->evalJacobian(ie->getTime() + h, ie->getState() + v, jac);
      size_type dim = ie->mSystem->getNumContinousStates();
      jac -= (1/h)*LinAlg::Eye<real_type,0,0>(dim, dim);
    }
  private:
    ImplicitEuler* ie;
  };

  real_type mCurrentStepsize;
  real_type mJacStepsize;
  Matrix mJac;
  MatrixFactors mJacDecomp;

  /// Vector storing the derivative of that step. That is used for
  /// dense output.
  Vector mDeriv;
};

} // namespace OpenFDM

#endif
