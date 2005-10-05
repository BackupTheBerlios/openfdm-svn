/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Types.h"
#include "Object.h"
#include "Units.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Gravity.h"
#include "DefaultGravity.h"

namespace OpenFDM {

DefaultGravity::DefaultGravity(void)
{
  setPlanetMass(5.9742e24);
}

DefaultGravity::~DefaultGravity(void)
{
}

real_type
DefaultGravity::getPlanetMass(void) const
{
  return mMass;
}

void
DefaultGravity::setPlanetMass(real_type mass)
{
  mMass = mass;
}

Vector3
DefaultGravity::gravityAccel(const Vector3& cart) const
{
  real_type dist = norm(cart);
  return (-mMass*gravity_constant/(dist*dist*dist))*cart;
}

} // namespace OpenFDM
