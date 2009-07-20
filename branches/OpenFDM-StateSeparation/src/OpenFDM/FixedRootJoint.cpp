/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "FixedRootJoint.h"

#include "Assert.h"
#include "JointContext.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Task.h"
#include "MechanicContext.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

class FixedRootJoint::Context : public JointContext {
public:
  Context(const FixedRootJoint* rootJoint, const Environment* environment,
          MechanicLinkValue* parentLinkValue, MechanicLinkValue* childLinkValue,
          PortValueList& portValueList) :
    JointContext(environment, parentLinkValue, childLinkValue, portValueList),
    mFixedRootJoint(rootJoint)
  {}
  virtual ~Context() {}
  
  virtual const FixedRootJoint& getNode() const
  { return *mFixedRootJoint; }
  
  virtual void initDesignPosition()
  {
    mChildLink.setDesignPosition(mFixedRootJoint->getPosition());
  }

  virtual void velocities(const Task& task)
  {
    Vector3 angularVel = getEnvironment().getAngularVelocity(task.getTime());
    CoordinateSystem coordSys(mFixedRootJoint->mRootPosition,
                              mFixedRootJoint->mRootOrientation);
    mChildLink.setCoordinateSystem(coordSys);
    Vector6 vel = angularMotionTo(mFixedRootJoint->mRootPosition, angularVel);
    mChildLink.setVelocity(Vector6::zeros());
    mChildLink.setInertialVelocity(vel);

    mChildLink.setForce(Vector6::zeros());
    mChildLink.setInertia(SpatialInertia::zeros());
  }
  virtual void accelerations(const Task& task)
  {
    Vector6 spAccel = getEnvironment().getAcceleration(task.getTime());
    spAccel = motionTo(mFixedRootJoint->mRootPosition, spAccel);
    mChildLink.setInertialAcceleration(spAccel);
  }
  
private:
  SharedPtr<const FixedRootJoint> mFixedRootJoint;
};

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

JointContext*
FixedRootJoint::newJointContext(const Environment* environment,
                                MechanicLinkValue* parentLinkValue,
                                MechanicLinkValue* childLinkValue,
                                PortValueList& portValueList) const
{
  SharedPtr<Context> context = new Context(this, environment, parentLinkValue,
                                           childLinkValue, portValueList);
  if (!context->allocStates()) {
    Log(Model, Warning) << "Could not alloc for model \""
                        << getName() << "\"" << endl;
    return false;
  }
  return context.release();
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

} // namespace OpenFDM
