/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Object.h"
#include "Limits.h"
#include "Vector.h"
#include "Matrix.h"
#include "Function.h"

namespace OpenFDM {

Function::~Function(void)
{
}

void
Function::jac(value_type t, const invector_type& v, jacobian_type& jac)
{
  numJac(t, v, jac);
}

void
Function::numJac(value_type t, const invector_type& v, jacobian_type& jac)
{
  // Compute numerical jacobian.

  size_type insize = inSize();
  size_type outsize = outSize();

  // Create space ...
  jac.resize(outsize, insize);

  // Get the function value at the current position.
  Vector fv;
  eval(t, v, fv);

  value_type sqrteps = 1e4*sqrt(Limits<value_type>::epsilon());

  Vector tmpv = v;
  Vector tmpfv;
  size_type i;
  for (i = 1; i <= insize; ++i) {
    tmpv(i) += sqrteps;

    // Evaluate then function ...
    eval(t, tmpv, tmpfv);

    // ... and compute the differencequotient to approximate the derivative.
    jac(Range(1, outsize), i) = (1/sqrteps)*(tmpfv-fv);

    // Restore the original value.
    tmpv(i) = v(i);
  }
}

} // namespace OpenFDM
