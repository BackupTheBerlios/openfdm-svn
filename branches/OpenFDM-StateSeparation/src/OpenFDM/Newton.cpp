/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#undef NDEBUG

#include "Newton.h"

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Function.h"

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

  Log(NewtonMethod, Debug) << "__________________________" << std::endl;
  // The displacement of the undamped newton method for the first
  // iteration step.
  Vector err;
  f.eval(t, x, err);
  Log(NewtonMethod, Debug1) << "err " << trans(err) << std::endl;
  Vector dx_bar = jacInv.solve(err);
  Log(NewtonMethod, Debug2) << "dx_bar " << trans(dx_bar) << std::endl;
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

    Log(NewtonMethod, Debug1) << "outer " << normdx << std::endl;

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

      Log(NewtonMethod, Debug) << "err " << trans(err) << std::endl;

      // Check if we get some kind of convergence with this lambda.
      // This jacobian evaluation will also be used for the next step if
      // this lambda truns out to be acceptable.
      dx_bar = jacInv.solve(err);
      Log(NewtonMethod, Debug2) << "dx_bar " << trans(dx_bar) << std::endl;

      // The convergence criterion parameter theta.
      real_type theta = 1.0 - 0.5*lambda;

      // Compute the norm of dx_bar and check if we get a better approximation
      // to the current solution.
      real_type normdx_bar = norm(dx_bar);

      Log(NewtonMethod, Debug) << "inner " << normdx_bar << std::endl;

      const real_type min_conv_rate = 1e-10;
      if (normdx == 0.0) {
        Log(NewtonMethod, Error) << "Whow: we have most likely an exact "
          "solution and we iterate furter:  normdx = " << normdx << std::endl;
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

      Log(NewtonMethod, Debug) << "Computing new jacobian" << std::endl;

      // get a new jacobian ...
      f.jac(t, x, jacInv.data());
      Log(NewtonMethod, Debug2) << jacInv.data() << std::endl;
      jacInv.factorize();
      Log(NewtonMethod, Debug2) << "decomposed qr\n" << jacInv.data() << std::endl;

      if (jacInv.singular())
        Log(NewtonMethod, Warning) << "Have singular jacobian!" << std::endl;

      converging = true;
    }

    // Iterate as long as either the iteration converged or the
    // maximum iteration count is reached.
  } while (!converged && converging && 0 < maxit);

  Log(NewtonMethod, Info) << "Newton type method: converged = "
                          << converged << std::endl;
  
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

Vector
LineSearch(Function& f, real_type t, const Vector& xk, const Vector& dk,
           real_type maxWide, real_type thresh)
{
  static const real_type vfac = (3-sqrt(5.0))/2;
  static const real_type wfac = (sqrt(5.0)-1)/2;

  thresh = fabs(maxWide)*thresh;

  Vector fx;

  Vector v = xk;
  f.eval(t, v, fx);
  real_type fv = norm(fx);

  Vector w = xk + maxWide*dk;
  f.eval(t, w, fx);
  real_type fw = norm(fx);

  while (norm1(v - w) > thresh) {
    Log(NewtonMethod, Debug2) << " Line Search: errv = " << fv
                              << ", errw = " << fw << std::endl;
    // check for isfinite ...
    if (fv > fw) {
      v = v + vfac*(w-v);
      f.eval(t, v, fx);
      fv = norm(fx);
    } else {
      w = v + wfac*(w-v);
      f.eval(t, w, fx);
      fw = norm(fx);
    }
  }

  return 0.5*(v+w);
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
  Vector err, dx;
  Matrix J;
#define USE_QR
#ifdef USE_QR
  LinAlg::MatrixFactors<real_type,0,0,LinAlg::QRTag> jacFactors;
#else
  LinAlg::MatrixFactors<real_type,0,0,LinAlg::LUTag> jacFactors;
#endif

  bool converged;
  do {
    // Compute in each step a new jacobian
    f.jac(t, x, J);
    Log(NewtonMethod, Debug) << "Jacobian is:\n" << J << std::endl;
#ifdef USE_QR
    jacFactors = J;
#else
    jacFactors = trans(J)*J;
#endif
    Log(NewtonMethod, Debug) << "Jacobian is "
                             << (jacFactors.singular() ? "singular" : "ok")
                             << std::endl;
   
    // Compute the actual error
    f.eval(t, x, err);

    // Compute the search direction
#ifdef USE_QR
    dx = jacFactors.solve(err);
#else
    dx = jacFactors.solve(trans(J)*err);
#endif
    Log(NewtonMethod, Debug) << "dx residual "
                             << trans(J*dx - err) << std::endl
                             << trans(J*dx - err)*J
                             << std::endl;

    // Get a better search guess
    if (1 < norm(dx))
      dx = normalize(dx);
    Vector xnew = LineSearch(f, t, x, -dx, 1.0, atol);

    // check convergence
    converged = norm1(xnew - x) < atol;
    

    Log(NewtonMethod, Debug) << "Convergence test: |dx| = " << norm(xnew - x)
                             << ", converged = " << converged << std::endl;
    // New guess is the better one
    x = xnew;
  } while (!converged);

  return converged;
}

bool
LevenbergMarquart(Function& f,
                  real_type t,
                  Vector& x,
                  real_type atol, real_type rtol,
                  unsigned *itCount,
                  unsigned maxit)
{
  Log(NewtonMethod, Debug3) << "Start guess\nx = " << trans(x) << std::endl;

  Matrix J;
  LinAlg::MatrixFactors<real_type,0,0,LinAlg::LUTag> jacFactors;

  bool converged = false;
  real_type tau = 1e-1;
  real_type nu = 2;

  // Compute in each step a new jacobian
  f.jac(t, x, J);
  Log(NewtonMethod, Debug3) << "Jacobian is:\n" << J << std::endl;
  real_type mu = tau*norm1(J);

  Vector fx;
  // Compute the actual error
  f.eval(t, x, fx);
  Vector g = trans(J)*fx;

  do {
    jacFactors = trans(J)*J + mu*LinAlg::Eye<real_type,0,0>(rows(x), rows(x));
    Log(NewtonMethod, Debug) << "Jacobian is "
                             << (jacFactors.singular() ? "singular" : "ok")
                             << std::endl;
   
    // Compute the search direction
    Vector h = jacFactors.solve(-g);
    Log(NewtonMethod, Debug) << "Solve Residual "
                             << norm(trans(J)*J*h + mu*h + g)/norm(g) << std::endl;

    // Get a better search guess
    Vector xnew = x + h;

    // check convergence
    converged = equal(x, xnew, atol, rtol);
    Log(NewtonMethod, Debug) << "Convergence test: ||h||_1 = " << norm1(h)
                             << ", converged = " << converged << std::endl;
    if (converged)
      break;

    f.eval(t, x, fx);
    real_type Fx = norm(fx);
    f.eval(t, xnew, fx);
    real_type Fxnew = norm(fx);
    real_type rho = (Fx - Fxnew)/(0.5*dot(h, mu*h - g));
    Log(NewtonMethod, Debug) << "Rho = " << rho
                             << ", Fxnew = " << Fxnew 
                             << ", Fx = " << Fx
                             << std::endl;
    if (0 < rho) {
      Log(NewtonMethod, Debug) << "Accepted step!" << std::endl;
      Log(NewtonMethod, Debug3) << "xnew = " << trans(xnew) << std::endl;
      Log(NewtonMethod, Debug3) << "h    = " << trans(h) << std::endl;

      // New guess is the better one
      x = xnew;

      f.jac(t, x, J);
      Log(NewtonMethod, Debug3) << "Jacobian is:\n" << J << std::endl;
      // Compute the actual error
      f.eval(t, x, fx);
      g = trans(J)*fx;
      converged = norm1(g) < atol;
      Log(NewtonMethod, Debug) << "||g||_1 = " << norm1(g) << std::endl;

      mu = mu * max(real_type(1)/3, 1-pow(2*rho-1, real_type(3)));
      nu = 2;

    } else {
      mu = mu * nu;
      nu = 2 * nu;
    }
  } while (!converged);

  return converged;
}

} // namespace OpenFDM
