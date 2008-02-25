/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Unit.h"

#include "Types.h"
#include "Math.h"
#include "Vector.h"

namespace OpenFDM {

#ifdef PI
#undef PI
#endif
#define PI static_cast<real_type>(3.1415926535897932384626433832795029L)

const real_type pi2 = 2*PI;
const real_type pi = PI;
const real_type pi05 = static_cast<real_type>(0.5L)*PI;
const real_type pi025 = static_cast<real_type>(0.25L)*PI;

const real_type deg2rad = PI/static_cast<real_type>(180);
const real_type rad2deg = static_cast<real_type>(180)/PI;

// The newtonian gravity constant.
const real_type gravity_constant = 6.673e-11;

} // namespace OpenFDM

