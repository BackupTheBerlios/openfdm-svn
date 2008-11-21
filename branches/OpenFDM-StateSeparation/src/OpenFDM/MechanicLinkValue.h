/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicLinkValue_H
#define OpenFDM_MechanicLinkValue_H

#include "Inertia.h"
#include "PortValue.h"
#include "Frame.h"

namespace OpenFDM {

class EnvironmentCache;

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
  getGravityAcceleration(const EnvironmentCache&, const Vector3&) const
  { return Vector3(0, 0, 9.81); }
};

class AbstractWind : public Referenced {
public:
  virtual ~AbstractWind() {}
  virtual Vector6 getWindVelocity(const EnvironmentCache&, const Vector3&) const
  { return Vector6::zeros(); }
};

class EnvironmentCache : public Referenced {
public:
  EnvironmentCache() :
    mInertial(new AbstractInertial),
    mGravity(new AbstractGravity),
    mWind(new AbstractWind)
  {
  }
  EnvironmentCache(const AbstractInertial* inertial,
                   const AbstractGravity* gravity,
                   const AbstractWind* wind) :
    mInertial(inertial),
    mGravity(gravity),
    mWind(wind)
  {
  }
  virtual ~EnvironmentCache() {}

  // The the global coordinate frames angular velocity and acceleration.
  // Note that the acceleration and velocity must fit together to simulate
  // something useful.
  Vector3 getAngularVelocity(const real_type& t) const
  { return mInertial->getAngularVelocity(t); }
  Vector6 getAcceleration(const real_type& t) const
  { return mInertial->getAcceleration(t); }

  // Sets a new RootJoint position, evaluate environmental stuff
  void setRootJointPosition(const Vector3& position)
  {
    mRootJointPosition = position;
    mGravityAcceleration = getGravityAcceleration(position);
    mWindVelocity = getWindVelocity(position);
  }
  const Vector3& getRootJointPosition() const
  { return mRootJointPosition; }

  Vector3 getGravityAcceleration(const Vector3& position) const
  { return mGravity->getGravityAcceleration(*this, position); }
  const Vector3& getGravityAccelerationAtRoot() const
  { return mGravityAcceleration; }

  Vector6 getWindVelocity(const Vector3& position) const
  { return mWind->getWindVelocity(*this, position); }
  const Vector6& getWindVelocityAtRoot() const
  { return mWindVelocity; }

private:
  Vector3 mRootJointPosition;

  SharedPtr<const AbstractInertial> mInertial;

  Vector3 mGravityAcceleration;
  SharedPtr<const AbstractGravity> mGravity;

  Vector6 mWindVelocity;
  SharedPtr<const AbstractWind> mWind;

//   SharedPtr<const AbstractPlanet> mPlanet;
//   SharedPtr<const AbstractAtmosphere> mAtmosphere;
//   SharedPtr<const AbstractGround> mGround;
};


class MechanicLinkValue : public PortValue {
public:
  MechanicLinkValue();
  virtual ~MechanicLinkValue();

  virtual MechanicLinkValue* toMechanicLinkValue() { return this; }
  virtual const MechanicLinkValue* toMechanicLinkValue() const { return this; }

  const Frame& getFrame() const
  { return mFrame; }
  Frame& getFrame()
  { return mFrame; }

  const SpatialInertia& getInertia() const
  { return mArticulatedInertia; }
  void setInertia(const SpatialInertia& inertia)
  { mArticulatedInertia = inertia; }

  const Vector6& getForce() const
  { return mArticulatedForce; }
  void setForce(const Vector6& force)
  { mArticulatedForce = force; }

  void applyForce(const Vector6& force)
  { mArticulatedForce += force; }
  void applyForce(const Vector3& force)
  { applyForce(Vector6(Vector3::zeros(), force)); }
  void applyTorque(const Vector3& torque)
  { applyForce(Vector6(torque, Vector3::zeros())); }

  void applyInertia(const SpatialInertia& inertia)
  { mArticulatedInertia += inertia; }


  void setPosAndVel(const MechanicLinkValue& linkValue)
  {
    mArticulatedInertia = SpatialInertia::zeros();
    mArticulatedForce = Vector6::zeros();
    mFrame.setPosAndVel(linkValue.getFrame());
  }
  void setAccel(const MechanicLinkValue& linkValue)
  { mFrame.setAccel(linkValue.getFrame()); }
  void setPosAndVel(const MechanicLinkValue& linkValue, const Vector3& position,
                    const Quaternion& orientation, const Vector6& velocity)
  {
    mArticulatedInertia = SpatialInertia::zeros();
    mArticulatedForce = Vector6::zeros();
    mFrame.setPosAndVel(linkValue.getFrame(), position, orientation, velocity);
  }
  void setAccel(const MechanicLinkValue& linkValue, const Vector6& accel)
  { mFrame.setAccel(linkValue.getFrame(), accel); }
  void setPosAndVel(const Vector3& parentAngularVel, const Vector3& position,
                    const Quaternion& orientation, const Vector6& velocity)
  {
    mArticulatedInertia = SpatialInertia::zeros();
    mArticulatedForce = Vector6::zeros();
    mFrame.setPosAndVel(parentAngularVel, position, orientation, velocity);
  }

  void applyArticulation(const MechanicLinkValue& linkValue)
  {
    applyForce(linkValue.mArticulatedForce);
    applyInertia(linkValue.mArticulatedInertia);
  }

  const Vector3& getDesignPosition() const
  { return mDesignPosition; }
  void setDesignPosition(const Vector3& designPosition)
  { mDesignPosition = designPosition; }

  // This is a per link value because of interacts that can be child of two
  // different roots.
  // FIXME, enforce setting that in the contructor
  const EnvironmentCache* getEnvironment() const
  { return mEnvironment; }

  void setEnvironment(const EnvironmentCache* environment)
  { OpenFDMAssert(environment); mEnvironment = environment; }

protected:
  // May be build a class hierarchy that accounts for different inputs
  // and outputs a rigid body can have.
  // Example: force port, force and inertia, frame port, velocity port
  Frame mFrame;
  Vector6 mArticulatedForce;
  SpatialInertia mArticulatedInertia;

  Vector3 mDesignPosition;

  SharedPtr<const EnvironmentCache> mEnvironment;
};

} // namespace OpenFDM

#endif
