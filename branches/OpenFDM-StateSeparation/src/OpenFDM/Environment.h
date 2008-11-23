/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Environment_H
#define OpenFDM_Environment_H

#include "Matrix.h"
#include "Referenced.h"
#include "SharedPtr.h"
#include "Vector.h"

namespace OpenFDM {

class Environment;

class AbstractInertial : public Referenced {
public:
  virtual ~AbstractInertial() {}
  virtual Vector3 getAngularVelocity(const real_type& t) const
  { return Vector3::zeros(); }
//   { return Vector3(0, 0, pi2/(24*60*60)); }
  virtual Vector6 getAcceleration(const real_type& t) const
  { return Vector6::zeros(); }
};

class AbstractGravity : public Referenced {
public:
  virtual ~AbstractGravity() {}
  virtual Vector3
  getGravityAcceleration(const Environment&, const Vector3&) const
  { return Vector3(0, 0, 9.81); }
};

class AbstractWind : public Referenced {
public:
  virtual ~AbstractWind() {}
  virtual Vector6
  getWindVelocity(const Environment&, const real_type& t, const Vector3&) const
  { return Vector6::zeros(); }
};

class Environment : public Referenced {
public:
  Environment();
  virtual ~Environment();

  // The the global coordinate frames angular velocity and acceleration.
  // Note that the acceleration and velocity must fit together to simulate
  // something useful.
  Vector3 getAngularVelocity(const real_type& t) const
  { return mInertial->getAngularVelocity(t); }
  Vector6 getAcceleration(const real_type& t) const
  { return mInertial->getAcceleration(t); }

  // The gravity acceleration vector in the global coordinate system
  Vector3 getGravityAcceleration(const Vector3& position) const
  { return mGravity->getGravityAcceleration(*this, position); }

  // The wind velocity vector in the global coordinate system
  Vector6 getWindVelocity(const real_type& t, const Vector3& position) const
  { return mWind->getWindVelocity(*this, t, position); }

private:
  SharedPtr<const AbstractInertial> mInertial;
  SharedPtr<const AbstractGravity> mGravity;
  SharedPtr<const AbstractWind> mWind;
//   SharedPtr<const AbstractPlanet> mPlanet;
//   SharedPtr<const AbstractAtmosphere> mAtmosphere;
//   SharedPtr<const AbstractGround> mGround;
};

} // namespace OpenFDM

#endif
