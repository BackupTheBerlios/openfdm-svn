/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Newton_H
#define OpenFDM_Newton_H

#include "Vector.h"
#include "Matrix.h"
#include "Function.h"

namespace OpenFDM {

bool
Newton(Function& f,
       MatrixFactors& jacInv,
       real_type t,
       Vector& x,
       real_type atol, real_type rtol,
       unsigned *itCount = 0,
       unsigned maxit = 10,
       unsigned maxjac = 2,
       real_type lambdamin = 1e-3);

bool
GaussNewton(Function& f,
            real_type t,
            Vector& x,
            real_type atol, real_type rtol,
            unsigned *itCount = 0,
            unsigned maxit = 50,
            unsigned maxjac = 20,
            real_type lambdamin = 1e-5);

} // namespace OpenFDM

#endif
