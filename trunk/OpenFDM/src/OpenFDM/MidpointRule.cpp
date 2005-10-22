/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "ODESolver.h"
#include "MidpointRule.h"

namespace OpenFDM {

MidpointRule::MidpointRule(void)
{
  mCollocationPolynomialValid = false;
}

MidpointRule::~MidpointRule(void)
{
}

void
MidpointRule::invalidateHistory(void)
{
  mCollocationPolynomialValid = false;
}

bool
MidpointRule::integrate(real_type toTEnd)
{
  real_type rtol = 1e-12;
  real_type atol = 1e-10;

  Vector dy(mState.size());
  Vector f;
  while (!reached(toTEnd)) {
    real_type t = getTime();
    real_type h = maxStepsize(toTEnd);

    // Often used values.
    real_type h2 = 0.5*h;
    
    // We assume that the problem is nonstiff.
    // This is a sensible assumption since this symmetric integrator is not
    // stiffly accurate anyway.
    // If it is not stiff, we solve the nonlinear equation
    // with fixpoint iteration.
    // The GNI papers suggest this anyway.
    
    // An initial guess for the fixpoint iteration.
    // Note that the polynomial coefficients p1 and p2 are set to zero at the
    // first step past a reset.
    if (mCollocationPolynomialValid) {
      dy = old_dy;
    } else {
      dy.clear();
    }

    // Solve the implicit equation
    bool converging;
    bool converged = false;
    unsigned maxit = 10;
    real_type prev_err = Limits<real_type>::max();
    do {
      // Compute new approximations to the state at the inner stages
      Vector y = mState + 0.5*dy;

      // Compute new approximations to the state derivatives
      evalFunction(t+h2, y, f);

      // Check if the increment is small enough ...
      real_type err = scaledDiff(y, mState + h2*f, atol, rtol);
      converged = err < 1;

      // Check if we do converge in any way.
      converging = err < prev_err;
      prev_err = err;

      // Use the new approximation
      dy = h*f;

      ++mStats.numIter;
    } while (!converged && 0 < --maxit && converging);

    if (converged) {
      // Update the solution
      mState += dy;

      // Compute the collocation polynomial.
      // Is used for a predictor of the next fixpoint iterate start guess.
      old_dy = dy;
      mCollocationPolynomialValid = true;

      ++mStats.numSteps;
    } else {
      // If we cannot solve the nonlinear equation, do an explicit euler step
      mCollocationPolynomialValid = false;

      Log(TimeStep, Warning) << "MidpointRule did not converge" << endl;

      evalFunction(t, mState, f);
      mState += h*f;

      ++mStats.numFailed;
      ++mStats.numSteps;
    }
    
    // Increment the simulation time ...
    mTime += h;
  }
  return true;
}

} // namespace OpenFDM
