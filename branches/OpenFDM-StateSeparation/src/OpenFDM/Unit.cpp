/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "Unit.h"

#include "Types.h"
#include "Math.h"
#include "Vector.h"

namespace OpenFDM {

const real_type pi2 = 2*Constants<real_type>::pi();
const real_type pi = Constants<real_type>::pi();
const real_type pi05 = Constants<real_type>::pi()/real_type(2);
const real_type pi025 = Constants<real_type>::pi()/real_type(4);

const real_type deg2rad = Constants<real_type>::pi()/real_type(180);
const real_type rad2deg = real_type(180)/Constants<real_type>::pi();

// The newtonian gravity constant.
const real_type gravity_constant = real_type(6.673e-11);

} // namespace OpenFDM

