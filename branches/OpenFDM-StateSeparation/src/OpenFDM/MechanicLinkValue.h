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

  /// The velocity of this mechanic link
  ///
  /// The velocity is given in global coordinates and at this mechanic links
  /// coordinate systems origin.
  void setVelocity(const Vector6& velocity)
  { mVelocity = velocity; }
  const Vector6& getVelocity() const
  { return mVelocity; }

  /// The inertial velocity of this mechanic link
  ///
  /// The velocity is given in global coordinates and at this mechanic links
  /// coordinate systems origin.
  void setInertialVelocity(const Vector6& inertialVelocity)
  { mInertialVelocity = inertialVelocity; }
  const Vector6& getInertialVelocity() const
  { return mInertialVelocity; }

  /// The inertial acceleration of this mechanic link
  ///
  /// The acceleration is given in global coordinates and at this mechanic links
  /// coordinate systems origin.
  void setInertialAcceleration(const Vector6& inertialAcceleration)
  { mInertialAcceleration = inertialAcceleration; }
  const Vector6& getInertialAcceleration() const
  { return mInertialAcceleration; }

  /// The force of this mechanic link
  ///
  /// The force is given in global coordinates and at this mechanic links
  /// coordinate systems origin.
  const Vector6& getForce() const
  { return mForce; }
  void setForce(const Vector6& force)
  { mForce = force; }

  /// The inertia of this mechanic link
  ///
  /// The inertia is given in global coordinates and at this mechanic links
  /// coordinate systems origin.
  const SpatialInertia& getInertia() const
  { return mInertia; }
  void setInertia(const SpatialInertia& inertia)
  { mInertia = inertia; }


  /// Returns the coordinate system of link wrt this links coordinate system
  CoordinateSystem
  getRelativeCoordinateSystem(const MechanicLinkValue& link) const
  { return mCoordinateSystem.toLocal(link.mCoordinateSystem); }


  /// Conveninent getters at different positions/coordinatesystems
  Vector6 getVelocity(const Vector3& pos) const
  { return motionTo(pos - mCoordinateSystem.getPosition(), getVelocity()); }
  Vector6 getVelocity(const CoordinateSystem& cs) const
  { return cs.rotToLocal(getVelocity(cs.getPosition())); }

  Vector6 getInertialVelocity(const Vector3& pos) const
  { return motionTo(pos - mCoordinateSystem.getPosition(), getInertialVelocity()); }
  Vector6 getInertialVelocity(const CoordinateSystem& cs) const
  { return cs.rotToLocal(getInertialVelocity(cs.getPosition())); }

  Vector6 getInertialAcceleration(const Vector3& pos) const
  { return motionTo(pos - mCoordinateSystem.getPosition(), getInertialAcceleration()); }
  Vector6 getInertialAcceleration(const CoordinateSystem& cs) const
  { return cs.rotToLocal(getInertialAcceleration(cs.getPosition())); }

  Vector6 getForce(const Vector3& pos) const
  { return forceTo(pos - mCoordinateSystem.getPosition(), getForce()); }
  Vector6 getForce(const CoordinateSystem& cs) const
  { return cs.rotToLocal(getForce(cs.getPosition())); }


  /// Conveninent setters for the forces
  void applyGlobalTorque(const Vector3& torque)
  {
    mForce -= Vector6(torque, Vector3::zeros());
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
    Vector3 positionDiff = cs.getPosition() - mCoordinateSystem.getPosition();
    mForce -= forceFrom(positionDiff, cs.rotToReference(force));
  }
  void applyForce(const CoordinateSystem& cs, const Vector6& force)
  {
    Vector3 positionDiff = cs.getPosition() - mCoordinateSystem.getPosition();
    mForce -= forceFrom(positionDiff, cs.rotToReference(force));
  }


  void addForce(const Vector3& position, const Vector6& force)
  {
    Vector3 offset = position - mCoordinateSystem.getPosition();
    mForce += forceFrom(offset, force);
  }

  SpatialInertia getInertia(const Vector3& pos) const
  {
    // is inertiaTo(pos - cs.getPosition(), ...)
    return inertiaFrom(mCoordinateSystem.getPosition() - pos, getInertia());
  }
  SpatialInertia getInertia(const CoordinateSystem& cs) const
  { return cs.rotToLocal(getInertia(cs.getPosition())); }

  void addInertia(const Vector3& position, const SpatialInertia& inertia)
  {
    if (position == mCoordinateSystem.getPosition()) {
      mInertia += inertia;
    } else {
      Vector3 offset = position - mCoordinateSystem.getPosition();
      mInertia += inertiaFrom(offset, inertia);
    }
  }
  void addInertia(const Vector3& position,
                  const InertiaMatrix& inertia, const real_type& mass)
  {
    Vector3 offset = position - mCoordinateSystem.getPosition();
    mInertia += SpatialInertia(offset, inertia, mass);
  }

  Vector6 getLocalVelocity() const
  { return mCoordinateSystem.rotToLocal(mVelocity); }

  Vector6 getLocalAcceleration() const
  { return mCoordinateSystem.rotToLocal(mInertialAcceleration); }

protected:
  /// The local coordinate system of the mechanic link.
  CoordinateSystem mCoordinateSystem;

  /// The velocity at the origin of the coordinate system
  /// measured in global coordinate systems coordinates.
  /// The spacial velocities pivot point is at the coordinate systems origin.
  Vector6 mVelocity;

  /// The inertial velocity at the origin of the coordinate system
  /// measured in global coordinate systems coordinates.
  /// The spacial velocities pivot point is at the coordinate systems origin.
  Vector6 mInertialVelocity;

  /// The inertial acceleration at the origin of the coordinate system
  /// measured in global coordinate systems coordinates.
  /// The spacial accelerations pivot point is at the coordinate systems origin.
  Vector6 mInertialAcceleration;

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
  { return mLinkRelPos; }

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
  { return getCoordinateSystem().toLocal(link.getCoordinateSystem()); }

  Vector6 getVelocity(const Vector3& position) const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getVelocity(position);
  }
  Vector6 getVelocity(const CoordinateSystem& cs) const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getVelocity(cs);
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

  Vector6 getInertialAcceleration(const Vector3& position) const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getInertialAcceleration(position);
  }
  Vector6 getInertialAcceleration(const CoordinateSystem& cs) const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getInertialAcceleration(cs);
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
  const Vector6& getInertialAcceleration() const
  { return mMechanicLinkValue->getInertialAcceleration(); }
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
    mMechanicLinkValue->applyForce(cs.toReference(mLinkRelPos), force);
  }

  void applyGlobalTorque(const Vector3& torque)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->applyGlobalTorque(torque);
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

  void addForce(const Vector3& position, const Vector6& force)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->addForce(position, force);
  }
  void addInertia(const Vector3& position, const SpatialInertia& inertia)
  {
    OpenFDMAssert(isConnected());
    mMechanicLinkValue->addInertia(position, inertia);
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

  void setInertialAcceleration(const Vector6& accel)
  { mMechanicLinkValue->setInertialAcceleration(accel); }
  const Vector6& getInertialAcceleration() const
  { return mMechanicLinkValue->getInertialAcceleration(); }

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

  void setForce(const Vector6& force)
  { return mMechanicLinkValue->setForce(force); }
  const Vector6& getForce() const
  { return mMechanicLinkValue->getForce(); }

  void setInertia(const SpatialInertia& inertia) const
  { return mMechanicLinkValue->setInertia(inertia); }
  const SpatialInertia& getInertia() const
  { return mMechanicLinkValue->getInertia(); }
 
private:
  SharedPtr<MechanicLinkValue> mMechanicLinkValue;
};

} // namespace OpenFDM

#endif
