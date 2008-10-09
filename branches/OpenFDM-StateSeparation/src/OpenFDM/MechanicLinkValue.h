/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicLinkValue_H
#define OpenFDM_MechanicLinkValue_H

#include "Inertia.h"
#include "PortValue.h"
#include "Rotation.h"
#include "Vector.h"

namespace OpenFDM {

class MechanicLinkValue : public PortValue {
public:
  virtual ~MechanicLinkValue();

  virtual MechanicLinkValue* toMechanicLinkValue() { return this; }
  virtual const MechanicLinkValue* toMechanicLinkValue() const { return this; }

// protected:
  // FIXME:
  // Since the interact side is the provider port, an interact might provide
  // different typed ports, the Rigid body can test for at init time and
  // avoid inertia computations for ports not contributing that ...
  // May be build a class hierarchy that accounts for different inputs
  // and outputs a rigid body can have.
  // Example: force port, force and inertia, frame port, velocity port
  Vector3 mPosition;
  Rotation mOrientation;
  Vector6 mSpatialVelocity;
  Vector6 mSpatialAcceleration;
  Vector6 mArticulatedForce;
  SpatialInertia mArticulatedInertia;
  // Frame????
  // FIXME: how to show who is responsible for setting the values???
  // Solution:
  // Ok, if set the joint is responsible for filling the velocities
  // and accelerations, the body is responsible for the inertia -
  // if unset it is the other way round ...
  bool mDownStream;
};

} // namespace OpenFDM

#endif
