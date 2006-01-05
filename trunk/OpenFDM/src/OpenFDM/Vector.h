/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Vector_H
#define OpenFDM_Vector_H

#include "Assert.h"
#include "Types.h"
#include "Math.h"
#include "LinAlg/Misc.h"
#include "LinAlg/Base.h"
#include "LinAlg/IO.h"
#include "LinAlg/Proxy.h"
#include "LinAlg/Array.h"
#include "LinAlg/Container.h"
#include "LinAlg/Expressions.h"
#include "LinAlg/Algorithm.h"

namespace OpenFDM {

using LinAlg::Zeros;
using LinAlg::Range;
using LinAlg::trans;

typedef LinAlg::Vector2<real_type> Vector2;
typedef LinAlg::Vector3<real_type> Vector3;
typedef LinAlg::Vector4<real_type> Vector4;
typedef LinAlg::Vector6<real_type> Vector6;

typedef LinAlg::Vector<real_type> Vector;

using LinAlg::Size;

enum {
  iU     = 1, iV    = 2, iW    = 3,
  iP     = 1, iQ    = 2, iR    = 3,
  iL     = 1, iM    = 2, iN    = 3,
  iX     = 1, iY    = 2, iZ    = 3,
  iPhi   = 1, iTht  = 2, iPsi  = 3,
  iNorth = 1, iEast = 2, iDown = 3,
  sP     = 1, sQ    = 2, sR    = 3, sU = 4, sV = 5, sW = 6,
  sL     = 1, sM    = 2, sN    = 3, sX = 4, sY = 5, sZ = 6
};

} // namespace OpenFDM

#endif
