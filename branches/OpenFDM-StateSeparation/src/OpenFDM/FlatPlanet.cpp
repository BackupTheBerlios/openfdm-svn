/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "FlatPlanet.h"

#include "Types.h"
#include "Unit.h"
#include "Vector.h"

namespace OpenFDM {

FlatPlanet::FlatPlanet(void)
{
}

FlatPlanet::~FlatPlanet(void)
{
}

Plane
FlatPlanet::getHorizont(const Vector3& position) const
{
  return Plane(Vector3::unit(2), 0);
}

Vector3
FlatPlanet::getGravityAcceleration(const Vector3&) const
{
  return Vector3(0, 0, 9.81);
}

Vector3
FlatPlanet::getAngularVelocity(const real_type&) const
{
  return Vector3::zeros();
}

Vector6
FlatPlanet::getAcceleration(const real_type&) const
{
  return Vector6::zeros();
}

} // namespace OpenFDM
