/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "Environment.h"

#include "FlatPlanet.h"
#include "AtmosphereSTD1976.h"

namespace OpenFDM {

Environment::Environment() :
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
