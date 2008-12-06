/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SphericalEarth_H
#define OpenFDM_SphericalEarth_H

#include "Types.h"
#include "Vector.h"
#include "AbstractPlanet.h"

namespace OpenFDM {

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

private:
  real_type mMass;
  real_type mRadius;
  Vector3 mAngularVelocity;
};

} // namespace OpenFDM

#endif
