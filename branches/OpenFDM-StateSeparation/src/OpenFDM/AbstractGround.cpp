/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "AbstractGround.h"

#include "Types.h"
#include "Plane.h"
#include "Environment.h"

namespace OpenFDM {

AbstractGround::AbstractGround(void)
{
}

AbstractGround::~AbstractGround(void)
{
}

GroundValues
AbstractGround::getGroundPlane(const Environment& environment,
                               const real_type& t, const Vector3& refPos) const
{
  return GroundValues(environment.getHorizontalLocalPlane(refPos));
}

bool
AbstractGround::getCatapultValues(real_type t, const Vector3& refPos,
                                  CatapultValues& catVal) const
{
  return false;
}

bool
AbstractGround::caughtWire(const HookPosition& old,
                           const HookPosition& current) const
{
  return false;
}

bool
AbstractGround::getWireEnds(real_type t, WireValues& wireVal) const
{
  return false;
}

void
AbstractGround::releaseWire(void) const
{
}

} // namespace OpenFDM
