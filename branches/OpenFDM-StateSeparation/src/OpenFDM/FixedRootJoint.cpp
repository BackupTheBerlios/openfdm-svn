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
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  DEF_OPENFDM_PROPERTY(Quaternion, Orientation, Serialized)
  END_OPENFDM_OBJECT_DEF

FixedRootJoint::FixedRootJoint(const std::string& name) :
  RootJoint(name),
  mMechanicLink(newMechanicLink("link")),
  mPosition(0, 0, 0),
  mOrientation(Quaternion::unit())
{
}

FixedRootJoint::~FixedRootJoint()
{
}

const Vector3&
FixedRootJoint::getPosition() const
{
  return mPosition;
}

void
FixedRootJoint::setPosition(const Vector3& position)
{
  mPosition = position;
}

const Quaternion&
FixedRootJoint::getOrientation() const
{
  return mOrientation;
}

void
FixedRootJoint::setOrientation(const Quaternion& orientation)
{
  mOrientation = orientation;
}

void
FixedRootJoint::init(const Task&, DiscreteStateValueVector&,
                      ContinousStateValueVector& continousState,
                      const PortValueList& portValues) const
{
}

void
FixedRootJoint::initDesignPosition(PortValueList& portValues) const
{
  portValues[mMechanicLink].setDesignPosition(Vector3::zeros());
}

void
FixedRootJoint::velocity(const Task&,
                          const ContinousStateValueVector& continousState,
                          PortValueList& portValues) const
{
  portValues[mMechanicLink].setPosAndVel(getAngularBaseVelocity(), mPosition,
                                         mOrientation, Vector6::zeros());
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
