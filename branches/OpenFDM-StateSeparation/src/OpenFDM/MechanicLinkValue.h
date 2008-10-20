/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicLinkValue_H
#define OpenFDM_MechanicLinkValue_H

#include "Inertia.h"
#include "PortValue.h"
#include "Frame.h"

namespace OpenFDM {

class MechanicLinkValue : public PortValue {
public:
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
  { mArticulatedForce = force; }
  void applyForce(const Vector3& force)
  { applyForce(Vector6(Vector3::zeros(), force)); }
  void applyTorque(const Vector3& torque)
  { applyForce(Vector6(torque, Vector3::zeros())); }

  void applyInertia(const SpatialInertia& inertia)
  { mArticulatedInertia = inertia; }


  void setPosAndVel(const MechanicLinkValue& linkValue)
  {
    mFrame.setPosAndVel(linkValue.getFrame());
  }
  void setPosAndVel(const MechanicLinkValue& linkValue, const Vector3& position,
                    const Quaternion& orientation, const Vector6& velocity)
  {
    mFrame.setPosAndVel(linkValue.getFrame(), position, orientation, velocity);
  }

  void applyArticulation(const MechanicLinkValue& linkValue)
  {
    applyForce(linkValue.mArticulatedForce);
    applyInertia(linkValue.mArticulatedInertia);
  }

protected:
  // May be build a class hierarchy that accounts for different inputs
  // and outputs a rigid body can have.
  // Example: force port, force and inertia, frame port, velocity port
  Frame mFrame;
  Vector6 mArticulatedForce;
  SpatialInertia mArticulatedInertia;
};

} // namespace OpenFDM

#endif
