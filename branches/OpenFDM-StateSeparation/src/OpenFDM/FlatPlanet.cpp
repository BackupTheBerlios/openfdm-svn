/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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

Geodetic
FlatPlanet::toGeod(const Vector3& cart) const
{
  return Geodetic(cart(0)/(111000*rad2deg),
                  -cart(1)/(111000*rad2deg),
                  -cart(2));
}

Vector3
FlatPlanet::toCart(const Geodetic& geod) const
{
  return Vector3(111000*rad2deg*geod.latitude,
                 -111000*rad2deg*geod.longitude,
                 -geod.altitude);
}

Quaternion
FlatPlanet::getGeodHLOrientation(const Geodetic&) const
{
  return Quaternion::unit();
}

} // namespace OpenFDM
