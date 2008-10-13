/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "MobileRootJoint.h"

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Gravity.h"

namespace OpenFDM {

MobileRootJoint::MobileRootJoint(const std::string& name) :
  RootJoint(name),
  mMechanicLink(newMechanicLink("link")),
  mPositionStateInfo(new Vector3StateInfo),
  mOrientationStateInfo(new Vector4StateInfo),
  mVelocityStateInfo(new Vector6StateInfo)
{
  addContinousStateInfo(mPositionStateInfo);
  addContinousStateInfo(mOrientationStateInfo);
  addContinousStateInfo(mVelocityStateInfo);
}

MobileRootJoint::~MobileRootJoint()
{
}

void
MobileRootJoint::velocity(const Task&, const ContinousStateValueVector&,
                          PortValueList&) const
{
}

void
MobileRootJoint::articulation(const Task&, const ContinousStateValueVector&,
                              PortValueList&) const
{
}

void
MobileRootJoint::acceleration(const Task&, const ContinousStateValueVector&,
                              PortValueList&) const
{
}

void
MobileRootJoint::derivative(const DiscreteStateValueVector&,
                            const ContinousStateValueVector&,
                            const PortValueList&,
                            ContinousStateValueVector&) const
{
}

} // namespace OpenFDM
