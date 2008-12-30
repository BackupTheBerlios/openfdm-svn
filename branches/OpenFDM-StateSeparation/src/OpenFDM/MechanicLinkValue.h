/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
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

  const Frame& getFrame() const
  { return mFrame; }
  Frame& getFrame()
  { return mFrame; }

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

} // namespace OpenFDM

#endif
