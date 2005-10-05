/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Object.h"
#include "EnvironmentObject.h"
#include "AtmosphereSTD1976.h"
#include "DefaultGravity.h"
#include "DefaultGround.h"
#include "DefaultPlanet.h"
#include "Turbulence.h"
#include "Wind.h"
#include "Environment.h"

namespace OpenFDM {

Environment::Environment(void)
{
  setAtmosphere(new AtmosphereSTD1976);
  setGravity(new DefaultGravity);
  setGround(new DefaultGround);
  setPlanet(new DefaultPlanet);
  setWind(new Wind);
}

Environment::~Environment(void)
{
}

void
Environment::setAtmosphere(Atmosphere* atmosphere)
{
  detachEnvironmentObject(mAtmosphere);
  mAtmosphere = atmosphere;
  attachEnvironmentObject(atmosphere);
}

void
Environment::setGravity(Gravity* gravity)
{
  detachEnvironmentObject(mGravity);
  mGravity = gravity;
  attachEnvironmentObject(gravity);
}

void
Environment::setGround(Ground* ground)
{
  detachEnvironmentObject(mGround);
  mGround = ground;
  attachEnvironmentObject(ground);
}

void
Environment::setPlanet(Planet* planet)
{
  detachEnvironmentObject(mPlanet);
  mPlanet = planet;
  attachEnvironmentObject(planet);
}

void
Environment::setTurbulence(Turbulence* turbulence)
{
  detachEnvironmentObject(mTurbulence);
  mTurbulence = turbulence;
  attachEnvironmentObject(turbulence);
}

void
Environment::setWind(Wind* wind)
{
  detachEnvironmentObject(mWind);
  mWind = wind;
  attachEnvironmentObject(wind);
}

void
Environment::attachEnvironmentObject(EnvironmentObject* environmentObject)
{
  if (!environmentObject)
    return;

  environmentObject->attachToEnvironment(this);
}

void
Environment::detachEnvironmentObject(EnvironmentObject* environmentObject)
{
  if (!environmentObject)
    return;

  environmentObject->attachToEnvironment(0);
}

} // namespace OpenFDM
