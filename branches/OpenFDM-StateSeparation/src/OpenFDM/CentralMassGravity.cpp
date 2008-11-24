/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "CentralMassGravity.h"

#include "Types.h"
#include "Object.h"
#include "Unit.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Gravity.h"

namespace OpenFDM {

CentralMassGravity::CentralMassGravity(void)
{
  setPlanetMass(5.9742e24);
}

CentralMassGravity::~CentralMassGravity(void)
{
}

real_type
CentralMassGravity::getPlanetMass(void) const
{
  return mMass;
}

void
CentralMassGravity::setPlanetMass(real_type mass)
{
  mMass = mass;
}

Vector3
CentralMassGravity::getGravityAcceleration(const Environment&,
                                       const Vector3& cart) const
{
  real_type dist = norm(cart);
  return (-mMass*gravity_constant/(dist*dist*dist))*cart;
}

} // namespace OpenFDM
