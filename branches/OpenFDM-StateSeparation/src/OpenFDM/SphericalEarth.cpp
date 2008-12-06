/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "SphericalEarth.h"

#include "Types.h"
#include "Object.h"
#include "Unit.h"
#include "Vector.h"
#include "Quaternion.h"

namespace OpenFDM {

SphericalEarth::SphericalEarth(void) :
  mMass(5.9742e24),
  mRadius(6371009),
  mAngularVelocity(0, 0, pi2/(24*60*60))
{
}

SphericalEarth::~SphericalEarth(void)
{
}

const real_type&
SphericalEarth::getMass(void) const
{
  return mMass;
}

void
SphericalEarth::setMass(const real_type& mass)
{
  mMass = mass;
}

const real_type&
SphericalEarth::getRadius(void) const
{
  return mRadius;
}

void
SphericalEarth::setRadius(const real_type& radius)
{
  mRadius = radius;
}

const Vector3&
SphericalEarth::getAngularVelocity(void) const
{
  return mAngularVelocity;
}

void
SphericalEarth::setAngularVelocity(const Vector3& angularVelocity)
{
  mAngularVelocity = angularVelocity;
}

Plane
SphericalEarth::getHorizont(const Vector3& position) const
{
  Vector3 up = normalize(position);
  return Plane(-up, mRadius*up);
}

Vector3
SphericalEarth::getGravityAcceleration(const Vector3& cart) const
{
  real_type dist = norm(cart);
  return (-mMass*gravity_constant/(dist*dist*dist))*cart;
}

Vector3
SphericalEarth::getAngularVelocity(const real_type&) const
{
  return mAngularVelocity;
}

Vector6
SphericalEarth::getAcceleration(const real_type&) const
{
  return Vector6::zeros();
}

} // namespace OpenFDM
