/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Planet.h"

#include <iosfwd>

#include "Types.h"
#include "Object.h"
#include "Math.h"
#include "Vector.h"
#include "Quaternion.h"

namespace OpenFDM {

Planet::Planet(void)
{
}

Planet::~Planet(void)
{
}

Geocentric
Planet::toGeoc(const Vector3& cart) const
{
  real_type lon = (cart(0) == 0 && cart(1) == 0) ? 0 : atan2(cart(1), cart(0));
  real_type nxy = sqrt(cart(0)*cart(0)+cart(1)*cart(1));
  real_type lat = (nxy == 0 && cart(2) == 0) ? 0 : atan2(cart(2), nxy);
  return Geocentric(lat, lon, norm(cart));
}

Vector3
Planet::toCart(const Geocentric& geoc) const
{
  real_type slat = std::sin(geoc.latitude);
  real_type clat = cos(geoc.latitude);
  real_type slon = sin(geoc.longitude);
  real_type clon = cos(geoc.longitude);
  return geoc.radius*Vector3( clat*clon, clat*slon, slat );
}

Geocentric
Planet::toGeoc(const Geodetic& geod) const
{
  return toGeoc(toCart(geod));
}

Geodetic
Planet::toGeod(const Geocentric& geoc) const
{
  return toGeod(toCart(geoc));
}

Quaternion
Planet::getGeodHLOrientation(const Geodetic& pos) const
{
  return Quaternion::fromLonLat(pos.longitude, pos.latitude);
}

Quaternion
Planet::getGeodHLOrientation(const Vector3& pos) const
{
  return getGeodHLOrientation(toGeod(pos));
}

Quaternion
Planet::getGeodHLOrientation(const Geocentric& pos) const
{
  return getGeodHLOrientation(toCart(pos));
}

Quaternion
Planet::getGeocHLOrientation(const Geodetic& pos) const
{
  return getGeocHLOrientation(toCart(pos));
}

Quaternion
Planet::getGeocHLOrientation(const Vector3& pos) const
{
  return getGeocHLOrientation(toGeoc(pos));
}

Quaternion
Planet::getGeocHLOrientation(const Geocentric& pos) const
{
  return Quaternion::fromLonLat(pos.longitude, pos.latitude);
}

Vector3
Planet::getGoecHLRate(const Geocentric& pos, const Vector3& ecVel) const
{
  Quaternion hlOrientation = getGeocHLOrientation(pos);
  Vector3 hlVel = hlOrientation.transform(ecVel);
  Vector3 hlRate = Vector3(hlVel(1), -hlVel(0), -hlVel(1)*tan(pos.latitude));
  return hlOrientation.backTransform((1/pos.radius)*hlRate);
}

Vector3
Planet::getGoecHLRate(const Vector3& pos, const Vector3& ecVel) const
{
  return getGoecHLRate(toGeoc(pos), ecVel);
}

Vector3
Planet::getGoecHLRate(const Geodetic& pos, const Vector3& ecVel) const
{
  return getGoecHLRate(toCart(pos), ecVel);
}

std::ostream&
operator<<(std::ostream& os, const Geodetic& geod)
{
  return os << "[ lon = " << rad2deg*geod.longitude
            << ", lat = " << rad2deg*geod.latitude
            << ", alt = " << geod.altitude
            << " ]";
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
