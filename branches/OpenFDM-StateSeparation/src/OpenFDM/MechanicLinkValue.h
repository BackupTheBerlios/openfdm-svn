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

  /// Dynamic cast methods
  virtual MechanicLinkValue* toMechanicLinkValue();
  virtual const MechanicLinkValue* toMechanicLinkValue() const;

  /// The Coordinate system of this mechanic link
  ///
  /// The Coordinates position is the reference point of the reference frame
  /// that moves with this mechanic link.
  /// The spatial velocities, accelerations, forces and the inertia are all
  /// based on this reference point.
  const CoordinateSystem& getCoordinateSystem() const
  { return mCoordinateSystem; }
  CoordinateSystem& getCoordinateSystem()
  { return mCoordinateSystem; }
  void setCoordinateSystem(const CoordinateSystem& coordinateSystem)
  { mCoordinateSystem = coordinateSystem; }

  /// The spatial velocity of this mechanic link
  ///
  /// The velocity is given in global coordinates and at this mechanic links
  /// coordinate systems origin.
  void setVelocity(const Vector6& velocity)
  { mVelocity = velocity; }
  const Vector6& getVelocity() const
  { return mVelocity; }

  void setInertialVelocity(const Vector6& inertialVelocity)
  { mInertialVelocity = inertialVelocity; }
  const Vector6& getInertialVelocity() const
  { return mInertialVelocity; }

  /// The spatial acceleration of this mechanic link
  ///
  /// The acceleration is given in global coordinates and at this mechanic links
  /// coordinate systems origin.
  void setAcceleration(const Vector6& acceleration)
  { mAcceleration = acceleration; }
  const Vector6& getAcceleration() const
  { return mAcceleration; }

  /// The spatial force of this mechanic link
  ///
  /// The force is given in global coordinates and at this mechanic links
  /// coordinate systems origin.
  const Vector6& getSpatialForce() const
  { return mForce; }
  void setSpatialForce(const Vector6& force)
  { mForce = force; }

  /// The spatial inertia of this mechanic link
  ///
  /// The inertia is given in global coordinates and at this mechanic links
  /// coordinate systems origin.
  const SpatialInertia& getSpatialInertia() const
  { return mInertia; }
  void setSpatialInertia(const SpatialInertia& inertia)
  { mInertia = inertia; }


  /// Returns the coordinate system of link wrt this links coordinate system
  CoordinateSystem
  getRelativeCoordinateSystem(const MechanicLinkValue& link) const
  { return mCoordinateSystem.toLocal(link.mCoordinateSystem); }


  /// Conveninent getters at different positions/coordinatesystems
  Vector6 getSpatialVelocity(const Vector3& pos) const
  { return motionTo(pos - mCoordinateSystem.getPosition(), getVelocity()); }
  Vector6 getSpatialVelocity(const CoordinateSystem& cs) const
  { return cs.rotToLocal(getSpatialVelocity(cs.getPosition())); }

  Vector6 getInertialVelocity(const Vector3& pos) const
  { return motionTo(pos - mCoordinateSystem.getPosition(), getInertialVelocity()); }
  Vector6 getInertialVelocity(const CoordinateSystem& cs) const
  { return cs.rotToLocal(getInertialVelocity(cs.getPosition())); }

  Vector6 getSpatialAcceleration(const Vector3& pos) const
  { return motionTo(pos - mCoordinateSystem.getPosition(), getAcceleration()); }
  Vector6 getSpatialAcceleration(const CoordinateSystem& cs) const
  { return cs.rotToLocal(getSpatialAcceleration(cs.getPosition())); }

  Vector6 getSpatialForce(const Vector3& pos) const
  { return forceTo(pos - mCoordinateSystem.getPosition(), getSpatialForce()); }
  Vector6 getSpatialForce(const CoordinateSystem& cs) const
  { return cs.rotToLocal(getSpatialForce(cs.getPosition())); }

  SpatialInertia getSpatialInertia(const Vector3& pos) const
  {
    // is inertiaTo(pos - cs.getPosition(), ...)
    return inertiaFrom(mCoordinateSystem.getPosition() - pos,
                       getSpatialInertia());
  }
  SpatialInertia getSpatialInertia(const CoordinateSystem& cs) const
  { return cs.rotToLocal(getSpatialInertia(cs.getPosition())); }


  void applyForce(const Vector3& position, const Vector3& force)
  {
    Vector3 positionDiff = position - mCoordinateSystem.getPosition();
    mForce -= forceFrom(positionDiff, force);
  }
  void applyForce(const Vector3& position, const Vector6& force)
  {
    Vector3 positionDiff = position - mCoordinateSystem.getPosition();
    mForce -= forceFrom(positionDiff, force);
  }
  void applyForce(const CoordinateSystem& cs, const Vector3& force)
  {
    Vector3 forceInGlobal = cs.rotToReference(force);
    Vector3 positionDiff = cs.getPosition() - mCoordinateSystem.getPosition();
    mForce -= forceFrom(positionDiff, forceInGlobal);
  }
  void applyForce(const CoordinateSystem& cs, const Vector6& force)
  {
    Vector6 forceInGlobal = cs.rotToReference(force);
    Vector3 positionDiff = cs.getPosition() - mCoordinateSystem.getPosition();
    mForce -= forceFrom(positionDiff, forceInGlobal);
  }


  void addSpatialForce(const Vector3& position, const Vector6& force)
  {
    Vector3 offset = position - mCoordinateSystem.getPosition();
    mForce += forceFrom(offset, force);
  }
  void addSpatialInertia(const Vector3& position, const SpatialInertia& inertia)
  {
    if (position == mCoordinateSystem.getPosition()) {
      mInertia += inertia;
    } else {
      Vector3 offset = position - mCoordinateSystem.getPosition();
      mInertia += inertiaFrom(offset, inertia);
    }
  }


  SpatialInertia getLocalInertia() const
  { return getSpatialInertia(mCoordinateSystem); }
  Vector6 getLocalForce() const
  { return getSpatialForce(mCoordinateSystem); }

  void applyGlobalTorque(const Vector3& torque)
  {
    mForce -= Vector6(torque, Vector3::zeros());
  }
  void applyGlobalForce(const Vector3& globalPosition, const Vector3& force)
  {
    Vector3 positionDiff = globalPosition - mCoordinateSystem.getPosition();
    mForce -= forceFrom(positionDiff, force);
  }

  void applyLocalForce(const Vector6& force)
  { mForce -= mCoordinateSystem.rotToReference(force); }
  void applyLocalForce(const Vector3& position, const Vector6& force)
  { applyLocalForce(forceFrom(position, force)); }
  void applyLocalForce(const Vector3& force)
  { applyLocalForce(Vector6(Vector3::zeros(), force)); }
  void applyLocalForce(const Vector3& position, const Vector3& force)
  { applyLocalForce(forceFrom(position, force)); }
  void applyLocalTorque(const Vector3& torque)
  { applyLocalForce(Vector6(torque, Vector3::zeros())); }

  void setLocalVelocity(const Vector6& velocity)
  { mVelocity = mCoordinateSystem.rotToReference(velocity); }
  Vector6 getLocalVelocity() const
  { return mCoordinateSystem.rotToLocal(mVelocity); }

  Vector6 getLocalAcceleration() const
  { return mCoordinateSystem.rotToLocal(mAcceleration); }

  void addInertia(const Vector3& position,
                  const InertiaMatrix& inertia, const real_type& mass)
  {
    Vector3 offset = position - mCoordinateSystem.getPosition();
    mInertia += SpatialInertia(offset, inertia, mass);
  }

protected:
  /// The local coordinate system of the mechanic link.
  CoordinateSystem mCoordinateSystem;

  /// The spatial velocity at the origin of the coordinate system
  /// measured in global coordinate systems coordinates.
  /// The spacial velocities pivot point is at the coordinate systems origin.
  Vector6 mVelocity;
  /// Inertial velocity
  Vector6 mInertialVelocity;

  /// The spatial acceleration at the origin of the coordinate system
  /// measured in global coordinate systems coordinates.
  /// The spacial velocities pivot point is at the coordinate systems origin.
  Vector6 mAcceleration;

  /// The articulated force at the origin of the coordinate system
  /// measured in global coordinate systems coordinates.
  /// The articulated force pivot point is at the coordinate systems origin.
  Vector6 mForce;

  /// The articulated inertia at the origin of the coordinate system
  /// measured in global coordinate systems coordinates.
  /// The articulated inertia pivot point is at the coordinate systems origin.
  SpatialInertia mInertia;
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

  Vector6 getVelocity(const Vector3& position) const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getSpatialVelocity(position);
  }
  Vector6 getVelocity(const CoordinateSystem& cs) const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getSpatialVelocity(cs);
  }

  Vector6 getInertialVelocity(const Vector3& position) const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getInertialVelocity(position);
  }
  Vector6 getInertialVelocity(const CoordinateSystem& cs) const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getInertialVelocity(cs);
  }

  Vector6 getAcceleration(const Vector3& position) const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getSpatialAcceleration(position);
  }
  Vector6 getAcceleration(const CoordinateSystem& cs) const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getSpatialAcceleration(cs);
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
  const Vector6& getAcceleration() const
  { return mMechanicLinkValue->getAcceleration(); }
  const Vector6& getVelocity() const
  { return mMechanicLinkValue->getVelocity(); }
  const Vector6& getInertialVelocity() const
  { return mMechanicLinkValue->getInertialVelocity(); }

  void setDesignPosition(const Vector3& position)
  {
    OpenFDMAssert(isConnected());
    mLinkRelPos = position;
    mLinkRelPos -= mMechanicLinkValue->getCoordinateSystem().getPosition();
  }

  void applyBodyForce(const Vector6& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyLocalForce(mLinkRelPos, force);
  }
  void applyBodyForce(const Vector3& bodyPosition, const Vector6& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyLocalForce(bodyPosition + mLinkRelPos, force);
  }
  
  void applyBodyForce(const Vector3& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyLocalForce(mLinkRelPos, force);
  }
  void applyBodyForce(const Vector3& bodyPosition, const Vector3& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyLocalForce(bodyPosition + mLinkRelPos, force);
  }
  
  void applyBodyTorque(const Vector3& torque)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyLocalTorque(torque);
  }


  void applyGlobalForce(const Vector3& force)
  {
    OpenFDMAssert(isConnected());
    const CoordinateSystem& cs = mMechanicLinkValue->getCoordinateSystem();
    mMechanicLinkValue->applyGlobalForce(cs.toReference(mLinkRelPos), force);
  }

  void applyGlobalTorque(const Vector3& torque)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyGlobalTorque(torque);
  }

  void applyLocalForceAtLink(const Vector6& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyLocalForce(force);
  }

  void applyForce(const Vector3& position, const Vector3& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyForce(position, force);
  }
  void applyForce(const CoordinateSystem& cs, const Vector3& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyForce(cs, force);
  }
  void applyForce(const Vector3& position, const Vector6& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyForce(position, force);
  }
  void applyForce(const CoordinateSystem& cs, const Vector6& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyForce(cs, force);
  }

  void addSpatialForce(const Vector3& position, const Vector6& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->addSpatialForce(position, force);
  }
  void addSpatialInertia(const Vector3& position, const SpatialInertia& inertia)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->addSpatialInertia(position, inertia);
  }
  void addInertia(const Vector3& position,
                  const InertiaMatrix& inertia, const real_type& mass)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->addInertia(position, inertia, mass);
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
  { mMechanicLinkValue->setCoordinateSystem(CoordinateSystem(position)); }

  void setAcceleration(const Vector6& accel)
  { mMechanicLinkValue->setAcceleration(accel); }
  const Vector6& getAcceleration() const
  { return mMechanicLinkValue->getAcceleration(); }

  void setVelocity(const Vector6& velocity)
  { mMechanicLinkValue->setVelocity(velocity); }
  const Vector6& getVelocity() const
  { return mMechanicLinkValue->getVelocity(); }

  void setInertialVelocity(const Vector6& velocity)
  { mMechanicLinkValue->setInertialVelocity(velocity); }
  const Vector6& getInertialVelocity() const
  { return mMechanicLinkValue->getInertialVelocity(); }

  void setCoordinateSystem(const CoordinateSystem& coordinateSystem)
  { return mMechanicLinkValue->setCoordinateSystem(coordinateSystem); }
  const CoordinateSystem& getCoordinateSystem() const
  { return mMechanicLinkValue->getCoordinateSystem(); }

  void setSpatialForce(const Vector6& force)
  { return mMechanicLinkValue->setSpatialForce(force); }
  const Vector6& getSpatialForce() const
  { return mMechanicLinkValue->getSpatialForce(); }

  void setSpatialInertia(const SpatialInertia& inertia) const
  { return mMechanicLinkValue->setSpatialInertia(inertia); }
  const SpatialInertia& getSpatialInertia() const
  { return mMechanicLinkValue->getSpatialInertia(); }
 
private:
  SharedPtr<MechanicLinkValue> mMechanicLinkValue;
};

} // namespace OpenFDM

#endif
