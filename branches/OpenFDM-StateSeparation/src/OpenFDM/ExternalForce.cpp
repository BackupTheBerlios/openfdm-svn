/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "ExternalForce.h"

#include "MechanicLinkValue.h"
#include "NumericPortValue.h"
#include "PortValueList.h"
#include "Task.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(ExternalForce, Sensor)
  DEF_OPENFDM_PROPERTY(Bool, LocalCoordinates, Serialized)
  END_OPENFDM_OBJECT_DEF

ExternalForce::ExternalForce(const std::string& name) :
  Sensor(name),
  mForcePort(this, "force", Size(6, 1), true),
  mLocalCoordinates(true)
{
}

ExternalForce::~ExternalForce(void)
{
}

void
ExternalForce::articulation(const Task&, const Environment&,
                            const ContinousStateValueVector&,
                            PortValueList& portValues) const
{
  // FIXME, for now relative position
  Vector3 position = mPosition - portValues[mMechanicLink].getDesignPosition();
  if (mLocalCoordinates) {
    Vector6 force = -portValues[mForcePort];
    force = forceFrom(position, force);
    portValues[mMechanicLink].applyForce(force);
  } else {
    const Frame& frame = portValues[mMechanicLink].getFrame();
    Vector6 force = -portValues[mForcePort];
    force = forceTo(Vector3::zeros(), frame.getRefOrientation(), force);
    force = forceFrom(position, force);
    portValues[mMechanicLink].applyForce(force);
  }
}

void
ExternalForce::setLocalCoordinates(bool localCoordinates)
{
  mLocalCoordinates = localCoordinates;
}

bool
ExternalForce::getLocalCoordinates() const
{
  return mLocalCoordinates;
}

} // namespace OpenFDM
