/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Environment_H
#define OpenFDM_Environment_H

#include "AbstractAtmosphere.h"
#include "AbstractGround.h"
#include "AbstractPlanet.h"
#include "AbstractWind.h"
#include "CoordinateSystem.h"
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

  void setWind(const AbstractWind* wind)
  { mWind = wind; }
  const AbstractWind* getWind() const
  { return mWind; }

  void setPlanet(const AbstractPlanet* planet)
  { mPlanet = planet; }
  const AbstractPlanet* getPlanet() const
  { return mPlanet; }

  void setAtmosphere(const AbstractAtmosphere* atmosphere)
  { mAtmosphere = atmosphere; }
  const AbstractAtmosphere* getAtmosphere() const
  { return mAtmosphere; }

  void setGround(const AbstractGround* ground)
  { mGround = ground; }
  const AbstractGround* getGround() const
  { return mGround; }

  ///////////////////////////////////////////////////////////////////////////
  // Inertial related
  // FIXME may be this can be split out???

  // The the global coordinate frames angular velocity and acceleration.
  // Note that the acceleration and velocity must fit together to simulate
  // something useful.
  Vector3 getAngularVelocity(const real_type& t) const
  { return mPlanet->getAngularVelocity(t); }
  Vector6 getAcceleration(const real_type& t) const
  { return mPlanet->getAcceleration(t); }

  ///////////////////////////////////////////////////////////////////////////
  // Gravity related

  // The gravity acceleration vector in the global coordinate system
  Vector3 getGravityAcceleration(const Vector3& position) const
  { return mPlanet->getGravityAcceleration(position); }

  ///////////////////////////////////////////////////////////////////////////
  // Wind sensing related

  // The wind velocity vector in the global coordinate system
  Vector6 getWindVelocity(const real_type& t, const Vector3& position) const
  { return mWind->getWindVelocity(*this, t, position); }

  ///////////////////////////////////////////////////////////////////////////
  // Planet related
  real_type getAltitude(const Vector3& position) const
  { return - mPlanet->getHorizont(position).getDist(position); }
  Plane getHorizontalLocalPlane(const Vector3& position) const
  { return mPlanet->getHorizont(position); }
  real_type getAboveGroundLevel(const real_type& t, const Vector3& pos) const
  {
    // Get the unit down vector.
    Vector3 unitDown = mPlanet->getHorizont(pos).getNormal();
    GroundValues groundValues = getGroundPlane(t, pos);
    Vector3 intersectPoint;
    if (groundValues.plane.intersectLine(pos, unitDown, intersectPoint))
      return dot(unitDown, intersectPoint - pos);
    else
      return Limits<real_type>::max();
  }

  ///////////////////////////////////////////////////////////////////////////
  // Atmosphere sensing related
  AtmosphereData
  getAtmosphereData(const real_type& t, const real_type& altitude) const
  { return mAtmosphere->getData(t, altitude); }

  ///////////////////////////////////////////////////////////////////////////
  // Intersection interaction related
  GroundValues getGroundPlane(const real_type& t, const Vector3& pos) const
  {
    return mGround->getGroundPlane(*this, t, pos);
  }


  /// Return the plane at the position in the given coordinate system.
  /// The plane is also returned in the given coordinate system
  GroundValues
  getGroundPlane(const CoordinateSystem& cs, const real_type& t) const
  {
    GroundValues groundValues = getGroundPlane(t, cs.getPosition());
    return GroundValues(cs.planeToLocal(groundValues.plane),
                        cs.rotToLocal(groundValues.vel),
                        groundValues.friction);
  }

private:
  SharedPtr<const AbstractWind> mWind;
  SharedPtr<const AbstractPlanet> mPlanet;
  SharedPtr<const AbstractAtmosphere> mAtmosphere;
  SharedPtr<const AbstractGround> mGround;
};

} // namespace OpenFDM

#endif
