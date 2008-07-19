/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MidpointRule_H
#define OpenFDM_MidpointRule_H

#include "Object.h"
#include "ODESolver.h"

namespace OpenFDM {

/**
   Solve an ordinary differential equation with the implicit midpoint rule.

   The nonlinear equation is solved using a fixed point iteration.

   The implicit midpoint rule reads
   \f[
   y_{n+1} = y_{n} + h f_{n+1/2}
   \f]
   where \f$ f_{n+1/2} \f$ is defined by the implicit equation
   \f[
   f_{n+1/2} = f(t_{n} + \frac{1}{2} h,\,y_{n} + \frac{1}{2} h f_{n+1/2}).
   \f]
   The fixedpoint iteration is done in the increment \f$ h\,f_{n+1/2} \f$.
    
   \see E. Hairer, S. P. Norsett and G. Wanner,
   "Solving Ordinary Differential Equations I", 2nd edition,
   Springer-Verlag, 1991
   \see E. Hairer and G. Wanner,
   "Solving Ordinary Differential Equations II", 2nd edition,
   Springer-Verlag, 1996
   \see E. Hairer, Ch. Lubich and G. Wanner,
   "Geometric Numerical Integration. Structure-Preserving Algorithms for
   Ordinary Differential Equations",
   Springer-Verlag, 2002
   \see L. F. Shampine, I. Gladwell and S. Thompson,
   "Solving ODEs with MATLAB"
   
   @author Mathias Froehlich
*/

class MidpointRule
  : public ODESolver {
public:
  MidpointRule(void);
  virtual ~MidpointRule(void);

  virtual void invalidateHistory(void);
  virtual bool integrate(real_type toTEnd);
  virtual bool denseOutput(real_type t, Vector& out);

private:
  bool mCollocationPolynomialValid;
  Vector old_dy;
  /// Vector storing the derivative of that step. That is used for
  /// dense output.
  Vector mDeriv;
};

} // namespace OpenFDM

#endif
