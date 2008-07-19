/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Ground.h"

#include "Types.h"
#include "Object.h"
#include "Plane.h"
#include "Frame.h"
#include "Planet.h"
#include "Environment.h"

namespace OpenFDM {

Ground::Ground(void)
{
}

Ground::~Ground(void)
{
}

bool
Ground::getCatapultValues(real_type t, const Vector3& refPos,
                          CatapultValues& catVal) const
{
  return false;
}

bool
Ground::caughtWire(const HookPosition& old, const HookPosition& current) const
{
  return false;
}

bool
Ground::getWireEnds(real_type t, WireValues& wireVal) const
{
  return false;
}

void
Ground::releaseWire(void) const
{
}

} // namespace OpenFDM
