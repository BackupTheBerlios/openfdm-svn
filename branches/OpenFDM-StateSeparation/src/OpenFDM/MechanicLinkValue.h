/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicLinkValue_H
#define OpenFDM_MechanicLinkValue_H

#include "CoordinateSystem.h"
#include "Inertia.h"
#include "PortValue.h"

namespace OpenFDM {

class MechanicLinkValue : public PortValue {
public:
  MechanicLinkValue();
  virtual ~MechanicLinkValue();

  virtual MechanicLinkValue* toMechanicLinkValue();
  virtual const MechanicLinkValue* toMechanicLinkValue() const;

  /// This is an attempt to seperate the coordinate system stuff away from
  /// the reference frame handling.
  const CoordinateSystem& getCoordinateSystem() const
  { return mCoordinateSystem; }
  CoordinateSystem& getCoordinateSystem()
  { return mCoordinateSystem; }
  void setCoordinateSystem(const CoordinateSystem& coordinateSystem)
  { mCoordinateSystem = coordinateSystem; }


  const SpatialInertia& getInertia() const
  { return mArticulatedInertia; }
  void setInertia(const SpatialInertia& inertia)
  { mArticulatedInertia = inertia; }

  const Vector6& getForce() const
  { return mArticulatedForce; }
  void setForce(const Vector6& force)
  { mArticulatedForce = force; }


  void applyForce(const Vector6& force)
  { mArticulatedForce -= force; }
  void applyForce(const Vector3& position, const Vector6& force)
  { applyForce(forceFrom(position, force)); }
  void applyForce(const Vector3& force)
  { applyForce(Vector6(Vector3::zeros(), force)); }
  void applyForce(const Vector3& position, const Vector3& force)
  { applyForce(forceFrom(position, force)); }
  void applyTorque(const Vector3& torque)
  { applyForce(Vector6(torque, Vector3::zeros())); }

  void addForce(const Vector6& force)
  { mArticulatedForce += force; }
  void addInertia(const SpatialInertia& inertia)
  { mArticulatedInertia += inertia; }

  /// Returns the coordinate system of link wrt this links coordinate system
  CoordinateSystem
  getRelativeCoordinateSystem(const MechanicLinkValue& link) const
  { return mCoordinateSystem.toLocal(link.mCoordinateSystem); }

  void setLocalVelocity(const Vector6& velocity)
  { mVelocity = mCoordinateSystem.rotToReference(velocity); }
  Vector6 getLocalVelocity() const
  { return mCoordinateSystem.rotToLocal(mVelocity); }

  void setLocalAcceleration(const Vector6& acceleration)
  { mAcceleration = mCoordinateSystem.rotToReference(acceleration); }
  Vector6 getLocalAcceleration() const
  { return mCoordinateSystem.rotToLocal(mAcceleration); }


  void setVelocity(const Vector6& velocity)
  { mVelocity = velocity; }
  const Vector6& getVelocity() const
  { return mVelocity; }

  void setAcceleration(const Vector6& acceleration)
  { mAcceleration = acceleration; }
  const Vector6& getAcceleration() const
  { return mAcceleration; }


  void setRelativeVelocity(const MechanicLinkValue& linkValue,
                           const Vector6& velocity)
  {
    mArticulatedInertia = SpatialInertia::zeros();
    mArticulatedForce = Vector6::zeros();

    Vector3 positionDiff = mCoordinateSystem.getPosition();
    positionDiff -= linkValue.mCoordinateSystem.getPosition();
    Vector6 parentVel = motionTo(positionDiff, linkValue.mVelocity);
    mVelocity = mCoordinateSystem.rotToReference(velocity) + parentVel;
  }
  void setBaseVelocity(const Vector3& baseAngularVel, const Vector6& velocity)
  {
    mArticulatedInertia = SpatialInertia::zeros();
    mArticulatedForce = Vector6::zeros();

    Vector3 referencePos = mCoordinateSystem.getPosition();
    Vector6 parentVel = angularMotionTo(referencePos, baseAngularVel);
    mVelocity = mCoordinateSystem.rotToReference(velocity) + parentVel;
  }

  const Vector3& getDesignPosition() const
  { return mDesignPosition; }
  void setDesignPosition(const Vector3& designPosition)
  { mDesignPosition = designPosition; }

protected:
  /// The local coordinate system of the mechanic link.
  CoordinateSystem mCoordinateSystem;

  /// The spatial velocity at the origin of the coordinate system
  /// measured in global coordinate systems coordinates.
  /// The spacial velocities pivot point is at the coordinate systems origin.
  Vector6 mVelocity;
  /// The spatial acceleration at the origin of the coordinate system
  /// measured in global coordinate systems coordinates.
  /// The spacial velocities pivot point is at the coordinate systems origin.
  Vector6 mAcceleration;

  Vector6 mArticulatedForce;
  SpatialInertia mArticulatedInertia;

  Vector3 mDesignPosition;
};

class ParentLink {
public:
  ParentLink(MechanicLinkValue* mechanicLinkValue = 0) :
    mMechanicLinkValue(mechanicLinkValue),
    mLinkRelPos(Vector3::zeros())
  { }

  bool isConnected() const
  { return mMechanicLinkValue; }

  const Vector3& getLinkRelPos() const
  {
    OpenFDMAssert(isConnected());
    return mLinkRelPos;
  }

  const MechanicLinkValue& getMechanicLinkValue() const
  {
    OpenFDMAssert(isConnected());
    return *mMechanicLinkValue;
  }

  CoordinateSystem getCoordinateSystem() const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getCoordinateSystem().getRelative(mLinkRelPos);
  }

  CoordinateSystem getRelativeCoordinateSystem(const ParentLink& link) const
  {
    return getCoordinateSystem().toLocal(link.getCoordinateSystem());
  }

  Vector3 getRefPos() const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getCoordinateSystem().toReference(mLinkRelPos);
  }
  const Rotation& getRefOr() const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getCoordinateSystem().getOrientation();
  }

  Vector6 getLocalVelocityAtLink() const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getLocalVelocity();
  }
  Vector6 getLocalAccelerationAtLink() const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getLocalAcceleration();
  }
  Vector6 getLocalVelocity() const
  {
    OpenFDMAssert(isConnected());
    return motionTo(mLinkRelPos, mMechanicLinkValue->getLocalVelocity());
  }
  Vector6 getLocalAcceleration() const
  {
    OpenFDMAssert(isConnected());
    return motionTo(mLinkRelPos, mMechanicLinkValue->getLocalAcceleration());
  }

  void setDesignPosition(const Vector3& position)
  {
    OpenFDMAssert(isConnected());
    mLinkRelPos = position - mMechanicLinkValue->getDesignPosition();
  }

  void applyBodyForce(const Vector6& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyForce(mLinkRelPos, force);
  }
  void applyBodyForce(const Vector3& bodyPosition, const Vector6& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyForce(bodyPosition + mLinkRelPos, force);
  }
  
  void applyBodyForce(const Vector3& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyForce(mLinkRelPos, force);
  }
  void applyBodyForce(const Vector3& bodyPosition, const Vector3& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyForce(bodyPosition + mLinkRelPos, force);
  }
  
  void applyBodyTorque(const Vector3& torque)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyTorque(torque);
  }


  void applyGlobalForce(const Vector3& force)
  {
    OpenFDMAssert(isConnected());
    const CoordinateSystem& cs = mMechanicLinkValue->getCoordinateSystem();
    Vector3 bodyForce = cs.rotToLocal(force);
    mMechanicLinkValue->applyForce(mLinkRelPos, bodyForce);
  }
  void applyGlobalForce(const Vector3& bodyPosition, const Vector3& force)
  {
    OpenFDMAssert(isConnected());
    const CoordinateSystem& cs = mMechanicLinkValue->getCoordinateSystem();
    Vector3 bodyForce = cs.rotToLocal(force);
    mMechanicLinkValue->applyForce(bodyPosition + mLinkRelPos, bodyForce);
  }

  void applyGlobalTorque(const Vector3& torque)
  {
    OpenFDMAssert(isConnected());
    const CoordinateSystem& cs = mMechanicLinkValue->getCoordinateSystem();
    mMechanicLinkValue->applyTorque(cs.rotToLocal(torque));
  }


  void applyForceAtLink(const Vector6& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyForce(force);
  }
  void addForceAtLink(const Vector6& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->addForce(force);
  }
  void addInertiaAtLink(const SpatialInertia& inertia)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->addInertia(inertia);
  }

private:
  SharedPtr<MechanicLinkValue> mMechanicLinkValue;
  Vector3 mLinkRelPos;
};

class ChildLink {
public:
  ChildLink(MechanicLinkValue* mechanicLinkValue) :
    mMechanicLinkValue(mechanicLinkValue)
  { OpenFDMAssert(mMechanicLinkValue); }

  const MechanicLinkValue& getMechanicLinkValue() const
  { return *mMechanicLinkValue; }

  void setDesignPosition(const Vector3& position)
  { mMechanicLinkValue->setDesignPosition(position); }

  void setLocalAcceleration(const Vector6& accel)
  { mMechanicLinkValue->setLocalAcceleration(accel); }
  Vector6 getLocalAcceleration() const
  { return mMechanicLinkValue->getLocalAcceleration(); }

  void setAcceleration(const Vector6& accel)
  { mMechanicLinkValue->setAcceleration(accel); }
  const Vector6& getAcceleration() const
  { return mMechanicLinkValue->getAcceleration(); }

  void setBaseVelocity(const Vector3& angularBaseVel, const Vector6& velocity)
  { mMechanicLinkValue->setBaseVelocity(angularBaseVel, velocity); }

  void setRelativeVelocity(const ParentLink& parentLink,
                           const Vector6& velocity)
  {
    const MechanicLinkValue& link = parentLink.getMechanicLinkValue();
    mMechanicLinkValue->setRelativeVelocity(link, velocity);
  }

  void setCoordinateSystem(const CoordinateSystem& coordinateSystem)
  { return mMechanicLinkValue->setCoordinateSystem(coordinateSystem); }
  const CoordinateSystem& getCoordinateSystem() const
  { return mMechanicLinkValue->getCoordinateSystem(); }

  const Vector6& getForce() const
  { return mMechanicLinkValue->getForce(); }
  const SpatialInertia& getInertia() const
  { return mMechanicLinkValue->getInertia(); }
 
private:
  SharedPtr<MechanicLinkValue> mMechanicLinkValue;
};

} // namespace OpenFDM

#endif
