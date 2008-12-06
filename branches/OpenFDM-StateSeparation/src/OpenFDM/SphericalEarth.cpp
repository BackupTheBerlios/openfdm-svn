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

Geocentric
SphericalEarth::toGeoc(const Vector3& cart) const
{
  real_type lon = (cart(0) == 0 && cart(1) == 0)
    ? real_type(0) : atan2(cart(1), cart(0));
  real_type nxy = sqrt(cart(0)*cart(0)+cart(1)*cart(1));
  real_type lat = (nxy == 0 && cart(2) == 0)
    ? real_type(0) : atan2(cart(2), nxy);
  return Geocentric(lat, lon, norm(cart));
}

Vector3
SphericalEarth::toCart(const Geocentric& geoc) const
{
  real_type slat = sin(geoc.latitude);
  real_type clat = cos(geoc.latitude);
  real_type slon = sin(geoc.longitude);
  real_type clon = cos(geoc.longitude);
  return geoc.radius*Vector3( clat*clon, clat*slon, slat );
}

Quaternion
SphericalEarth::getGeocHLOrientation(const Vector3& pos) const
{
  return getGeocHLOrientation(toGeoc(pos));
}

Quaternion
SphericalEarth::getGeocHLOrientation(const Geocentric& pos) const
{
  return Quaternion::fromLonLat(pos.longitude, pos.latitude);
}

Vector3
SphericalEarth::getGoecHLRate(const Geocentric& pos, const Vector3& ecVel) const
{
  Quaternion hlOrientation = getGeocHLOrientation(pos);
  Vector3 hlVel = hlOrientation.transform(ecVel);
  Vector3 hlRate = Vector3(hlVel(1), -hlVel(0), -hlVel(1)*tan(pos.latitude));
  return hlOrientation.backTransform((1/pos.radius)*hlRate);
}

Vector3
SphericalEarth::getGoecHLRate(const Vector3& pos, const Vector3& ecVel) const
{
  return getGoecHLRate(toGeoc(pos), ecVel);
}

std::ostream&
operator<<(std::ostream& os, const Geocentric& geoc)
{
  return os << "[ lon = " << rad2deg*geoc.longitude
            << ", lat = " << rad2deg*geoc.latitude
            << ", rad = " << geoc.radius
            << " ]";
}

} // namespace OpenFDM
