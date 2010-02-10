/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SphericalEarth_H
#define OpenFDM_SphericalEarth_H

#include "AbstractPlanet.h"
#include "Quaternion.h"
#include "Vector.h"

namespace OpenFDM {

/**
 * Datatype for a position in speric coordinates.
 */
struct Geocentric {
  Geocentric(real_type lat = 0.0, real_type lon = 0.0, real_type rad = 0.0)
    : latitude(lat), longitude(lon), radius(rad)
  {}
  real_type latitude;
  real_type longitude;
  real_type radius;
};

/**
 * The SphericalEarth class.
 *
 * It holds some information about the gravity the simulation is running on.
 */
class SphericalEarth : public AbstractPlanet {
public:
  /** CentralMass constructor.
   */
  SphericalEarth(void);

  /** CentralMass destructor.
   */
  virtual ~SphericalEarth(void);

  /** Get planet mass.
   */
  const real_type& getMass(void) const;

  /** Set planet mass.
   */
  void setMass(const real_type& mass);

  /** Get planet radius.
   */
  const real_type& getRadius(void) const;

  /** Set planet radius.
   */
  void setRadius(const real_type& radius);

  /** Get planets angular velocity.
   */
  const Vector3& getAngularVelocity(void) const;

  /** Set planes angular velocity.
   */
  void setAngularVelocity(const Vector3& angularVelocity);


  /** Returns the horizontal plane at zero altitude.
   *  Plane normal points downward.
   */
  virtual Plane getHorizont(const Vector3& position) const;

  /** Returns the gravitational acceleration for the given position.
   *  Note that this should not contain the effects of a non inertial
   *  reference frame as this effect is captured by the inertial
   *  frame methods.
   */
  virtual Vector3 getGravityAcceleration(const Vector3&) const;

  /** Return the global reference frames velocity and acceleration.
   *  Note that these both must fit together to make the simulation
   *  simulate something usable.
   */
  virtual Vector3 getAngularVelocity(const real_type& t) const;
  virtual Vector6 getAcceleration(const real_type& t) const;

  /** Transform cartesian coordinates to geocentric coordinates.
   */
  Geocentric toGeoc(const Vector3& cart) const;

  /** Transform geocentric coordinates to cartesian coordinates.
   */
  Vector3 toCart(const Geocentric& geoc) const;

  /** Orientation of the Geocentric horizontal local frame.
   */
  Quaternion getGeocHLOrientation(const Vector3& pos) const;

  /** Orientation of the Geocentric horizontal local frame.
   */
  Quaternion getGeocHLOrientation(const Geocentric& pos) const;

  /** Rotation rate of the Geocentric horizontal local frame.
   */
  Vector3 getGoecHLRate(const Geocentric& pos, const Vector3& ecVel) const;

  /** Rotation rate of the Geocentric horizontal local frame.
   */
  Vector3 getGoecHLRate(const Vector3& pos, const Vector3& ecVel) const;

private:
  real_type mMass;
  real_type mRadius;
  Vector3 mAngularVelocity;
};

/** Pretty printing of geocentric coordinates.
 */
std::ostream& operator<<(std::ostream& os, const Geocentric& geoc);

} // namespace OpenFDM

#endif
