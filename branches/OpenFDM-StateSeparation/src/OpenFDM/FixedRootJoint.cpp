/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "FixedRootJoint.h"

#include "Assert.h"
#include "LeafContext.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Gravity.h"
#include "MechanicContext.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(FixedRootJoint, RootJoint)
  END_OPENFDM_OBJECT_DEF

FixedRootJoint::FixedRootJoint(const std::string& name) :
  RootJoint(name),
  mMechanicLink(newMechanicLink("link"))
{
}

FixedRootJoint::~FixedRootJoint()
{
}

void
FixedRootJoint::init(const Task&, DiscreteStateValueVector&,
                      ContinousStateValueVector& continousState,
                      const PortValueList& portValues) const
{
}

void
FixedRootJoint::velocity(const Task&,
                          const ContinousStateValueVector& continousState,
                          PortValueList& portValues) const
{
  Vector3 position(0, 0, 0);
  Quaternion orientation = Quaternion::unit();
  Vector6 velocity = Vector6::zeros();

  portValues[mMechanicLink].setPosAndVel(getAngularBaseVelocity(),
                                         position, orientation, velocity);
}

void
FixedRootJoint::articulation(const Task&, const ContinousStateValueVector&,
                              PortValueList&) const
{
  /// In this case a noop.
}

void
FixedRootJoint::acceleration(const Task&, const ContinousStateValueVector&,
                              PortValueList& portValues) const
{
  // Assumption: body is small compared to the distance to the planets
  // center of mass. That means gravity could be considered equal for the
  // whole vehicle.
  // See Featherstone, Orin: Equations and Algorithms

  // FIXME
  Vector6 grav = Vector6(Vector3::zeros(), portValues[mMechanicLink].getFrame().rotFromRef(Vector3(0, 0, 9.81)));

  Vector6 spatialAcceleration = grav;
  portValues[mMechanicLink].getFrame().setSpAccel(spatialAcceleration);
}

} // namespace OpenFDM
