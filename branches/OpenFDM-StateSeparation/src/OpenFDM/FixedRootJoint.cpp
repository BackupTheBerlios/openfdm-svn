/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "FixedRootJoint.h"

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Task.h"
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
FixedRootJoint::velocity(const Task& task,
                         const ContinousStateValueVector& continousState,
                         PortValueList& portValues) const
{
  const Environment* environment;
  environment = portValues[mMechanicLink].getEnvironment();
  Vector3 angularBaseVelocity = environment->getAngularVelocity(task.getTime());
  portValues[mMechanicLink].setCoordinateSystem(CoordinateSystem(mPosition,
                                                                 mOrientation));
  portValues[mMechanicLink].setPosAndVel(angularBaseVelocity, mPosition,
                                         mOrientation, Vector6::zeros());
}

void
FixedRootJoint::articulation(const Task&, const ContinousStateValueVector&,
                              PortValueList&) const
{
  /// In this case a noop.
}

void
FixedRootJoint::acceleration(const Task& task, const ContinousStateValueVector&,
                              PortValueList& portValues) const
{
  const Environment* environment;
  environment = portValues[mMechanicLink].getEnvironment();
  Vector6 spatialAcceleration = environment->getAcceleration(task.getTime());
  portValues[mMechanicLink].getFrame().setSpAccel(spatialAcceleration);
}

} // namespace OpenFDM
