/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Environment.h"

#include "FlatPlanet.h"
#include "AtmosphereSTD1976.h"

namespace OpenFDM {

Environment::Environment() :
  mInertial(new AbstractInertial),
  mGravity(new AbstractGravity),
  mWind(new AbstractWind),
  mPlanet(new FlatPlanet),
  mAtmosphere(new AtmosphereSTD1976),
  mGround(new AbstractGround)
{
}

Environment::~Environment(void)
{
}

} // namespace OpenFDM
