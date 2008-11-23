/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicLinkValue_H
#define OpenFDM_MechanicLinkValue_H

#include "Inertia.h"
#include "PortValue.h"
#include "Frame.h"
#include "Environment.h"

namespace OpenFDM {

class EnvironmentCache : public Environment {
public:
  virtual ~EnvironmentCache() {}

  // Sets a new RootJoint position, evaluate environmental stuff
  void setPosition(const real_type& t, const Vector3& position)
  {
    mTime = t;
    mRootJointPosition = position;
    mGravityAcceleration = getGravityAcceleration(position);
    mWindVelocity = getWindVelocity(t, position);
  }
  const Vector3& getRootJointPosition() const
  { return mRootJointPosition; }

  const Vector3& getGravityAccelerationAtRoot() const
  { return mGravityAcceleration; }

  const Vector6& getWindVelocityAtRoot() const
  { return mWindVelocity; }

private:
  real_type mTime;
  Vector3 mRootJointPosition;
  Vector3 mGravityAcceleration;
  Vector6 mWindVelocity;
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
