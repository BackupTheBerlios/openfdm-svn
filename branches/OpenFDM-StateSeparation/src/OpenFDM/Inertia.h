/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Inertia_H
#define OpenFDM_Inertia_H

#include "Assert.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"

namespace OpenFDM {

typedef LinAlg::SymMatrix3<real_type> InertiaMatrix;

typedef LinAlg::SymMatrix6<real_type> SpatialInertia;

OpenFDM_FORCE_INLINE
Vector6
solve(const SpatialInertia& m, const Vector6& v)
{
  Matrix66Factors inv(m);
  return inv.solve(v);
}

OpenFDM_FORCE_INLINE
SpatialInertia
rank1(const Vector6& v)
{
  SpatialInertia ret;
  for (int j=1;j<=6;++j)
    for (int i=j;i<=6;++i)
      ret(i, j) = v(i)*v(j);
  return ret;
}

} // namespace OpenFDM

#endif
