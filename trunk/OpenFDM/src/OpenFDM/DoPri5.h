/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DoPri5_H
#define OpenFDM_DoPri5_H

#include "Object.h"
#include "ODESolver.h"

namespace OpenFDM {

/** This class implements an explicit Runge-Kutta scheme of oder 5 with an
    embedded scheme of order 4.
    Is at the moment mostly used for getting reference solutions and to
    see how the error estimates behave.
    
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

    @author Mathias Froehlich
*/

class DoPri5
  : public ODESolver {
public:
  DoPri5(void);
  virtual ~DoPri5(void);

  virtual bool integrate(real_type toTEnd);

private:
  // The coefficients of the method.
  static const real_type c2, c3, c4, c5;
  static const real_type d1, d3, d4, d5, d6, d7;
  static const real_type e1, e3, e4, e5, e6, e7;
  static const real_type a21, a31, a32, a41, a42, a43, a51, a52, a53, a54;
  static const real_type a61, a62, a63, a64, a65, a71, a73, a74, a75, a76;
};

} // namespace OpenFDM

#endif
