/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Environment.h"

#include "FlatPlanet.h"

namespace OpenFDM {

Environment::Environment() :
  mInertial(new AbstractInertial),
  mGravity(new AbstractGravity),
  mWind(new AbstractWind),
  mPlanet(new FlatPlanet)
{
}

Environment::~Environment(void)
{
}

} // namespace OpenFDM
