/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Function.h"
#include "Newton.h"

namespace OpenFDM {

/** This function implements the simplified and damped newton iteration
    to find the roots of a nonlinear function.
    
    @see E. Hairer, S. P. Norsett and G. Wanner,
    "Solving Ordinary Differential Equations I", 2nd edition,
    Springer-Verlag, 1991
    @see E. Hairer and G. Wanner,
    "Solving Ordinary Differential Equations II", 2nd edition,
    Springer-Verlag, 1996
    @see E. Hairer, Ch. Lubich and G. Wanner,
    "Geometric Numerical Integration. Structure-Preserving Algorithms for
     Ordinary Differential Equations",
    Springer-Verlag, 2002
    @see L. F. Shampine, I. Gladwell and S. Thompson,
    "Solving ODEs with MATLAB"
    @see L. O. Jay,
    "Inexact simplified newton iterations for implicit Runge-Kutta methods",
    SIAM J. NUMER. ANAL. Vol 38, No. 4, pp 1369-1388

    @author Mathias Froehlich
*/

template<typename T, typename FactorTag>
inline
bool
NewtonTypeMethod(Function& f,
                 LinAlg::MatrixFactors<T,0,0,FactorTag>& jacInv,
                 real_type t,
                 Vector& x,
                 real_type atol, real_type rtol,
                 unsigned *itCount,
                 unsigned maxit,
                 unsigned maxjac,
                 real_type lambdamin)
{
  // The initial damping factor.
  // lambda == 1 means nondapmed newton method.
  real_type lambda = 1;

  // Flag if we observe convergence or not.
  // If not abort and hope that the caller knows what todo ...
  bool converging;

  // True if the method is converged.
  // That is the error is below a given tolerance.
  bool converged = false;
  
  // x_new is the potential solution at the next iteration step.
  Vector x_new;

  Log(NewtonMethod, Debug) << "__________________________" << endl;
  // The displacement of the undamped newton method for the first
  // iteration step.
  Vector err;
  f.eval(t, x, err);
  Log(NewtonMethod, Debug1) << "err " << trans(err) << endl;
  Vector dx_bar = jacInv.solve(err);
  do {
    // Increment the iteration counter. Just statistics ...
    if (itCount)
      ++(*itCount);
    --maxit;
    
    // dx contains the displacement of the undamped newton iteration in the
    // current iteration step.
    Vector dx = dx_bar;

    // Compute the the error norm of the increment.
    real_type normdx = norm(dx);
    if (normdx == 0.0)
      return true;

    Log(NewtonMethod, Debug1) << "outer " << normdx << endl;

    // Damped newton method:
    // Use lambda*dx with 0 < lambda <= 1 instead of just dx as displacement.
    // Be optimistic and try twice the displacement from the prevous step.
    lambda = min(static_cast<real_type>(1), 2.0*lambda);


    // Convergence rate, that is an estimate of || e_{n+1} ||/|| e_{n} ||
    real_type convergenceRate;

    do {
      // Compute the new approximation to the solution with the current damping
      // factor lambda.
      x_new = x - lambda*dx;
      // Compute the error of that new approximation.
      f.eval(t, x_new, err);

      Log(NewtonMethod, Debug) << "err " << trans(err) << endl;

      // Check if we get some kind of convergence with this lambda.
      // This jacobian evaluation will also be used for the next step if
      // this lambda truns out to be acceptable.
      dx_bar = jacInv.solve(err);

      // The convergence criterion parameter theta.
      real_type theta = 1.0 - 0.5*lambda;

      // Compute the norm of dx_bar and check if we get a better approximation
      // to the current solution.
      real_type normdx_bar = norm(dx_bar);

      Log(NewtonMethod, Debug) << "inner " << normdx_bar << endl;

      const real_type min_conv_rate = 1e-10;
      if (normdx == 0.0) {
        Log(NewtonMethod, Error) << "Whow: we have most likely an exact "
          "solution and we iterate furter:  normdx = " << normdx << endl;
        convergenceRate = min_conv_rate;
      } else
        convergenceRate = max(min_conv_rate, normdx_bar/normdx);
      converging = normdx_bar < theta*normdx;
      if (converging)
        break;

      // If we are still not converging, half the damping factor and try again.
      lambda *= 0.5;
    } while (lambdamin < lambda);

    if (converging) {
      // Ok, we found a better approximation to the solution.
    
      // Finally check for convergence:
      // Compute the scaled norm of our last displacement ...
      real_type enormdx = scaledDiff(x, x_new, atol, rtol);
      // ... and check if either the convergence rate is very high, which
      // signals that we are very near to the zero crossing or if our last
      // displacement is *very* short
      converged = enormdx*max(static_cast<real_type>(1e-2),convergenceRate)
        < (1-convergenceRate);

      // Use the newly computed solution.
      x = x_new;
    } else if (0 <= maxjac) {
      --maxjac;
      // get a new jacobian ...
      f.jac(t, x, jacInv.data());
      jacInv.factorize();

      if (jacInv.singular())
        Log(NewtonMethod, Warning) << "Have singular jacobian!" << endl;

      converging = true;
    }

    // Iterate as long as either the iteration converged or the
    // maximum iteration count is reached.
  } while (!converged && converging && 0 < maxit);
  
  // Tell the caller if it worked or not.
  return converged;
}

bool
Newton(Function& f,
       MatrixFactors& jacInv,
       real_type t,
       Vector& x,
       real_type atol, real_type rtol,
       unsigned *itCount,
       unsigned maxit,
       unsigned maxjac,
       real_type lambdamin)
{
  return NewtonTypeMethod(f, jacInv, t, x, atol, rtol,
                      itCount, maxit, maxjac, lambdamin);
}

bool
GaussNewton(Function& f,
            real_type t,
            Vector& x,
            real_type atol, real_type rtol,
            unsigned *itCount,
            unsigned maxit,
            unsigned maxjac,
            real_type lambdamin)
{
  LinAlg::MatrixFactors<real_type,0,0,LinAlg::QRTag> jacFactors;
  f.jac(t, x, jacFactors.data());
  
  jacFactors.factorize();
  return NewtonTypeMethod(f, jacFactors, t, x, atol, rtol,
                          itCount, maxit, maxjac, lambdamin);
}

} // namespace OpenFDM
