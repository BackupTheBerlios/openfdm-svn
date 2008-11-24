/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Environment_H
#define OpenFDM_Environment_H

#include "AbstractAtmosphere.h"
#include "AbstractGravity.h"
#include "AbstractGround.h"
#include "AbstractInertial.h"
#include "AbstractPlanet.h"
#include "AbstractWind.h"
#include "Matrix.h"
#include "Plane.h"
#include "Referenced.h"
#include "SharedPtr.h"
#include "Vector.h"

namespace OpenFDM {

class Environment : public Referenced {
public:
  Environment();
  virtual ~Environment();

  ///////////////////////////////////////////////////////////////////////////
  // Inertial related
  // FIXME may be this can be split out???

  // The the global coordinate frames angular velocity and acceleration.
  // Note that the acceleration and velocity must fit together to simulate
  // something useful.
  Vector3 getAngularVelocity(const real_type& t) const
  { return mInertial->getAngularVelocity(t); }
  Vector6 getAcceleration(const real_type& t) const
  { return mInertial->getAcceleration(t); }

  ///////////////////////////////////////////////////////////////////////////
  // Gravity related

  // The gravity acceleration vector in the global coordinate system
  Vector3 getGravityAcceleration(const Vector3& position) const
  { return mGravity->getGravityAcceleration(*this, position); }

  ///////////////////////////////////////////////////////////////////////////
  // Wind sensing related

  // The wind velocity vector in the global coordinate system
  Vector6 getWindVelocity(const real_type& t, const Vector3& position) const
  { return mWind->getWindVelocity(*this, t, position); }

  ///////////////////////////////////////////////////////////////////////////
  // Planet related
  // FIXME: find out what we really need here, may be some kind of
  // abstract horizontal local coordinates?
  // Is responsible for altitude computations ...

  // From the planet these are so far what we need.
  // FIXME, invent an abstraction for that, may be optimize???
protected:
  Vector3 getHorizontalLocalDown(const Vector3& position) const
  {
    Quaternion hlOr = mPlanet->getGeodHLOrientation(position);
    return hlOr.backTransform(Vector3::unit(3));
  }
  Vector3 getHorizontalLocalOffset(const Vector3& position) const
  {
    Geodetic geod = mPlanet->toGeod(position);
    geod.altitude = 0;
    return mPlanet->toCart(geod);
  }
public:
  Plane getHorizontalLocalPlane(const Vector3& position) const
  {
    // Get the unit down vector.
    Vector3 unitDown = getHorizontalLocalDown(position);
    // Get the distance from the planets center.
    Vector3 groundOff = getHorizontalLocalOffset(position);
    // Then we know the plane ...
    return Plane(unitDown, groundOff);
  }

  ///////////////////////////////////////////////////////////////////////////
  // Atmosphere sensing related
  AtmosphereData getAtmosphereData(const Vector3& position) const
  {
    Geodetic geod = mPlanet->toGeod(position);
    return mAtmosphere->getData(geod.altitude);
  }

  ///////////////////////////////////////////////////////////////////////////
  // Intersection interaction related


private:
  SharedPtr<const AbstractInertial> mInertial;
  SharedPtr<const AbstractGravity> mGravity;
  SharedPtr<const AbstractWind> mWind;
  SharedPtr<const AbstractPlanet> mPlanet;
  SharedPtr<const AbstractAtmosphere> mAtmosphere;
  SharedPtr<const AbstractGround> mGround;
};

} // namespace OpenFDM

#endif
