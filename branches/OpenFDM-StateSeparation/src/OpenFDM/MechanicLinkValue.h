/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicLinkValue_H
#define OpenFDM_MechanicLinkValue_H

#include "CoordinateSystem.h"
#include "Frame.h"
#include "Inertia.h"
#include "PortValue.h"

namespace OpenFDM {

class MechanicLinkValue : public PortValue {
public:
  MechanicLinkValue();
  virtual ~MechanicLinkValue();

  virtual MechanicLinkValue* toMechanicLinkValue() { return this; }
  virtual const MechanicLinkValue* toMechanicLinkValue() const { return this; }

  /// Currently duplicate information from the Frame.
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

  /// Returns the velocity of the link measured in this links coordinate
  /// system and measured in this links reference frame
  Vector6 getRelativeVelocity(const MechanicLinkValue& link) const
  {
    CoordinateSystem csys = getRelativeCoordinateSystem(link);
    return csys.motionToReference(link.getSpVel()) - getSpVel();
  }


//   const Vector6& getSpVel() const
  Vector6 getSpVel() const
  { return mFrame.getSpVel(); }
//   const Vector6& getSpAccel() const
  Vector6 getSpAccel() const
  { return mFrame.getSpAccel(); }




  void setPosAndVel(const MechanicLinkValue& linkValue, const Vector3& position,
                    const Quaternion& orientation, const Vector6& velocity)
  {
    mArticulatedInertia = SpatialInertia::zeros();
    mArticulatedForce = Vector6::zeros();
    mFrame.setPosAndVel(linkValue.mFrame, position, orientation, velocity);
  }
  void setAccel(const MechanicLinkValue& linkValue, const Vector6& accel)
  { mFrame.setAccel(linkValue.mFrame, accel); }
  void setPosAndVel(const Vector3& parentAngularVel, const Vector3& position,
                    const Quaternion& orientation, const Vector6& velocity)
  {
    mArticulatedInertia = SpatialInertia::zeros();
    mArticulatedForce = Vector6::zeros();
    mFrame.setPosAndVel(parentAngularVel, position, orientation, velocity);
  }
  void setSpAccel(const Vector6& accel)
  { mFrame.setSpAccel(accel); }

  Vector6 getRelVelDot() const
  { return mFrame.getRelVelDot(); }

  const Vector3& getDesignPosition() const
  { return mDesignPosition; }
  void setDesignPosition(const Vector3& designPosition)
  { mDesignPosition = designPosition; }

  /// Returns the spatial reference velocity at the local position
  Vector6 getReferenceVelocity(const Vector3& position) const
  { return motionTo(position, mFrame.getRefVel()); }
  Vector6 getReferenceVelocity() const
  { return mFrame.getRefVel(); }

protected:
  /// The local coordinate system of the mechanic link.
  CoordinateSystem mCoordinateSystem;

  Frame mFrame;
  Vector6 mArticulatedForce;
  SpatialInertia mArticulatedInertia;

  Vector3 mDesignPosition;
};

class ChildLink {
public:
  ChildLink(MechanicLinkValue* mechanicLinkValue) :
    mMechanicLinkValue(mechanicLinkValue)
  { OpenFDMAssert(mMechanicLinkValue); }

  const MechanicLinkValue& getMechanicLinkValue() const
  { return *mMechanicLinkValue; }
  MechanicLinkValue& getMechanicLinkValue()
  { return *mMechanicLinkValue; }

  void setDesignPosition(const Vector3& position)
  { mMechanicLinkValue->setDesignPosition(position); }

  void setPosAndVel(const MechanicLinkValue& link, const Vector3& position,
                    const Quaternion& orientation, const Vector6& velocity)
  { mMechanicLinkValue->setPosAndVel(link, position, orientation, velocity); }
  void setAccel(const MechanicLinkValue& link, const Vector6& accel)
  { mMechanicLinkValue->setAccel(link, accel); }

  void setCoordinateSystem(const CoordinateSystem& coordinateSystem)
  {
    OpenFDMAssert(mMechanicLinkValue);
    return mMechanicLinkValue->setCoordinateSystem(coordinateSystem);
  }
  const CoordinateSystem& getCoordinateSystem() const
  {
    OpenFDMAssert(mMechanicLinkValue);
    return mMechanicLinkValue->getCoordinateSystem();
  }

  const Vector6& getForce() const
  {
    OpenFDMAssert(mMechanicLinkValue);
    return mMechanicLinkValue->getForce();
  }
  const SpatialInertia& getInertia() const
  {
    OpenFDMAssert(mMechanicLinkValue);
    return mMechanicLinkValue->getInertia();
  }
 
private:
  SharedPtr<MechanicLinkValue> mMechanicLinkValue;
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
  MechanicLinkValue& getMechanicLinkValue()
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

  Vector6 getSpVelAtLink() const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getSpVel();
  }
  Vector6 getSpAccelAtLink() const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getSpAccel();
  }
  Vector6 getSpVel() const
  {
    OpenFDMAssert(isConnected());
    return motionTo(mLinkRelPos, mMechanicLinkValue->getSpVel());
  }
  Vector6 getRefVel() const
  {
    OpenFDMAssert(isConnected());
    return mMechanicLinkValue->getReferenceVelocity(mLinkRelPos);
  }
  Vector6 getSpAccel() const
  {
    OpenFDMAssert(isConnected());
    return motionTo(mLinkRelPos, mMechanicLinkValue->getSpAccel());
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

} // namespace OpenFDM

#endif
