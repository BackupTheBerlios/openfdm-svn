/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
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
  DEF_OPENFDM_PROPERTY(Vector3, RootPosition, Serialized)
  DEF_OPENFDM_PROPERTY(Quaternion, RootOrientation, Serialized)
  END_OPENFDM_OBJECT_DEF

FixedRootJoint::FixedRootJoint(const std::string& name) :
  RootJoint(name),
  mMechanicLink(new MechanicLink(this, "link")),
  mRootPosition(0, 0, 0),
  mRootOrientation(Quaternion::unit())
{
}

FixedRootJoint::~FixedRootJoint()
{
}

const Vector3&
FixedRootJoint::getRootPosition() const
{
  return mRootPosition;
}

void
FixedRootJoint::setRootPosition(const Vector3& rootPosition)
{
  mRootPosition = rootPosition;
}

const Quaternion&
FixedRootJoint::getRootOrientation() const
{
  return mRootOrientation;
}

void
FixedRootJoint::setRootOrientation(const Quaternion& rootOrientation)
{
  mRootOrientation = rootOrientation;
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
  portValues[*mMechanicLink].setDesignPosition(getPosition());
}

void
FixedRootJoint::velocity(const Task& task, const Environment& environment,
                         const ContinousStateValueVector& continousState,
                         PortValueList& portValues) const
{
  Vector3 angularBaseVelocity = environment.getAngularVelocity(task.getTime());
  portValues[*mMechanicLink].setCoordinateSystem(CoordinateSystem(mRootPosition,
                                         mRootOrientation));
  portValues[*mMechanicLink].setPosAndVel(angularBaseVelocity, mRootPosition,
                                         mRootOrientation, Vector6::zeros());
}

void
FixedRootJoint::articulation(const Task&, const Environment& environment,
                             const ContinousStateValueVector&,
                             PortValueList&) const
{
  /// In this case a noop.
}

void
FixedRootJoint::acceleration(const Task& task, const Environment& environment,
                             const ContinousStateValueVector&,
                             PortValueList& portValues) const
{
  Vector6 spatialAcceleration = environment.getAcceleration(task.getTime());
  portValues[*mMechanicLink].getFrame().setSpAccel(spatialAcceleration);
}

} // namespace OpenFDM
