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

class ImplicitEuler : public ODESolver {
public:
  ImplicitEuler(void);
  virtual ~ImplicitEuler(void);

  virtual void invalidateHistory(void);
  virtual bool integrate(real_type toTEnd);
  virtual bool denseOutput(real_type t, Vector& out);

private:
  class IEFunction;

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
