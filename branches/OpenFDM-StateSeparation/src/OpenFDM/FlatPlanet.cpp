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

Plane
FlatPlanet::getHorizont(const Vector3& position) const
{
  return Plane(Vector3::unit(2), 0);
}

} // namespace OpenFDM
