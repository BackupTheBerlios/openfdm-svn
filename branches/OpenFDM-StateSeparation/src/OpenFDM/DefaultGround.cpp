/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "DefaultGround.h"

#include "Types.h"
#include "Object.h"
#include "Plane.h"
#include "Frame.h"
#include "Planet.h"
#include "Environment.h"
#include "Ground.h"

namespace OpenFDM {

DefaultGround::DefaultGround(void)
{
}

DefaultGround::~DefaultGround(void)
{
}

GroundValues
DefaultGround::getGroundPlane(real_type, const Vector3& refPos) const
{
  // The default implementation just uses the reference ellipsoid as ground.
  const Planet* planet = getEnvironment()->getPlanet();

  // Get the unit down vector.
  Quaternion hlOr = planet->getGeodHLOrientation(refPos);
  Vector3 unitDown = hlOr.backTransform(Vector3::unit(3));
  
  // Get the distance from the planets center.
  Geodetic geod = planet->toGeod(refPos);
  geod.altitude = 0;
  Vector3 groundOff = planet->toCart(geod);

  // Build up and return the aprioriate plane.
  return GroundValues(Plane(unitDown, groundOff));
}

} // namespace OpenFDM
