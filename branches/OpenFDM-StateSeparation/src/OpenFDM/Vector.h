/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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

typedef LinAlg::Vector1<real_type> Vector1;
typedef LinAlg::Vector2<real_type> Vector2;
typedef LinAlg::Vector3<real_type> Vector3;
typedef LinAlg::Vector4<real_type> Vector4;
typedef LinAlg::Vector6<real_type> Vector6;

typedef LinAlg::Vector<real_type> Vector;

using LinAlg::Size;

enum {
  iU     = 0, iV    = 1, iW    = 2,
  iP     = 0, iQ    = 1, iR    = 2,
  iL     = 0, iM    = 1, iN    = 2,
  iX     = 0, iY    = 1, iZ    = 2,
  iPhi   = 0, iTht  = 1, iPsi  = 2,
  iNorth = 0, iEast = 1, iDown = 2,
  sP     = 0, sQ    = 1, sR    = 2, sU = 3, sV = 4, sW = 5,
  sL     = 0, sM    = 1, sN    = 2, sX = 3, sY = 4, sZ = 5
};

} // namespace OpenFDM

#endif
