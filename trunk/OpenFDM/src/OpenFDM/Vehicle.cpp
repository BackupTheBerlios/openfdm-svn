/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Vehicle.h"

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Force.h"
#include "RigidBody.h"
#include "MobileRootJoint.h"
#include "Planet.h"
#include "Wind.h"
#include "ExplicitEuler.h"
#include "ExplicitAdams.h"
#include "ImplicitEuler.h"
#include "MidpointRule.h"
#include "DoPri5.h"
#include "ModelGroup.h"
#include "System.h"

namespace OpenFDM {

Vehicle::Vehicle(void)
{
  mSystem = new System("Top Vehicle System");

//   mSystem->setTimestepper(new ExplicitEuler);
//   mSystem->setTimestepper(new ExplicitAdams);
  mSystem->setTimestepper(new DoPri5);
//   mSystem->setTimestepper(new ImplicitEuler);
//   mSystem->setTimestepper(new MidpointRule);

  mModelGroup = new ModelGroup("Flight Control System");
  mSystem->addModel(mModelGroup);

  mMultiBodySystem = new ModelGroup("Multi Body System");
  mSystem->addModel(mMultiBodySystem);

  mTopBody = new RigidBody("Topmost rigid body");
  mMultiBodySystem->addModel(mTopBody);

  mMobileRootJoint = new MobileRootJoint("Mobile vehicle base");
  mMultiBodySystem->addModel(mMobileRootJoint);
  mTopBody->setInboardJoint(mMobileRootJoint);
}

Vehicle::~Vehicle(void)
{
}

bool
Vehicle::init(void)
{
  return mSystem->init();
}

void
Vehicle::output(void)
{
}

void
Vehicle::update(real_type dt)
{
  mSystem->simulate(mSystem->getTime() + dt);
}

bool
Vehicle::trim(void)
{
  return mSystem->trim();
}

void
Vehicle::setPlanet(Planet* p)
{
  mSystem->getEnvironment()->setPlanet(p);
}

void
Vehicle::setGround(Ground* p)
{
  mSystem->getEnvironment()->setGround(p);
}

void
Vehicle::setAtmosphere(Atmosphere* p)
{
  mSystem->getEnvironment()->setAtmosphere(p);
}

void
Vehicle::setWind(Wind* w)
{
  mSystem->getEnvironment()->setWind(w);
}

Vector3
Vehicle::getCartPosition(void) const
{
  return mTopBody->getFrame()->getRefPosition();/*FIXME*/
}

void
Vehicle::setCartPosition(const Vector3& pos)
{
  mMobileRootJoint->setRefPosition(pos);/*FIXME*/
}

Geodetic
Vehicle::getGeodPosition(void) const
{
  return getPlanet()->toGeod(getCartPosition());
}

void
Vehicle::setGeodPosition(const Geodetic& geod)
{
  setCartPosition(getPlanet()->toCart(geod));
}

Geocentric
Vehicle::getGeocPosition(void) const
{
  return getPlanet()->toGeoc(getCartPosition());
}

void
Vehicle::setGeocPosition(const Geocentric& geoc)
{
  setCartPosition(getPlanet()->toCart(geoc));
}

Quaternion
Vehicle::getCartOrientation(void) const
{
  return mTopBody->getFrame()->getRefOrientation();/*FIXME*/
}

void
Vehicle::setCartOrientation(const Quaternion& o)
{
  mMobileRootJoint->setRefOrientation(o);/*FIXME*/
}

Quaternion
Vehicle::getGeocOrientation(void) const
{
  Quaternion hlOr = getPlanet()->getGeocHLOrientation(getCartPosition());
  return inverse(hlOr)*getCartOrientation();
}

void
Vehicle::setGeocOrientation(const Quaternion& o)
{
  Quaternion hlOr = getPlanet()->getGeocHLOrientation(getCartPosition());
  setCartOrientation(hlOr*o);
}

Quaternion
Vehicle::getGeodOrientation(void) const
{
  Quaternion hlOr = getPlanet()->getGeodHLOrientation(getCartPosition()); 
  return inverse(hlOr)*getCartOrientation();
}

void
Vehicle::setGeodOrientation(const Quaternion& o)
{
  Quaternion hlOr = getPlanet()->getGeodHLOrientation(getCartPosition()); 
  setCartOrientation(hlOr*o);
}

} // namespace OpenFDM
