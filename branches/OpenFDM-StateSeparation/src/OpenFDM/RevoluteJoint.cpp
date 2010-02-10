/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "RevoluteJoint.h"
#include "Assert.h"
#include "LogStream.h"
#include "Limits.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "PortValueList.h"
#include "ContinousStateValueVector.h"
#include "MechanicContext.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(RevoluteJoint, Joint)
  DEF_OPENFDM_PROPERTY(Vector3, Axis, Serialized)
  DEF_OPENFDM_PROPERTY(Real, InitialPosition, Serialized)
  DEF_OPENFDM_PROPERTY(Real, InitialVelocity, Serialized)
  END_OPENFDM_OBJECT_DEF

RevoluteJoint::RevoluteJoint(const std::string& name) :
  CartesianJoint<1>(name),
  mPositionPort(this, "position", Size(1, 1)),
  mVelocityPort(this, "velocity", Size(1, 1)),
  mPositionStateInfo(new Vector1StateInfo),
  mVelocityStateInfo(new Vector1StateInfo),
  mAxis(Vector3(1, 0, 0)),
  mInitialPosition(0),
  mInitialVelocity(0)
{
  addContinousStateInfo(mPositionStateInfo);
  addContinousStateInfo(mVelocityStateInfo);
}

RevoluteJoint::~RevoluteJoint(void)
{
}

const Vector3&
RevoluteJoint::getAxis() const
{
  return mAxis;
}

void
RevoluteJoint::setAxis(const Vector3& axis)
{
  real_type nrm = norm(axis);
  if (nrm <= Limits<real_type>::min()) {
    Log(Initialization, Error) << "JointAxis is zero ..." << std::endl;
    return;
  }
  mAxis = (1/nrm)*axis;
}

const real_type&
RevoluteJoint::getInitialPosition() const
{
  return mInitialPosition;
}

void
RevoluteJoint::setInitialPosition(const real_type& initialPosition)
{
  mInitialPosition = initialPosition;
}

const real_type&
RevoluteJoint::getInitialVelocity() const
{
  return mInitialVelocity;
}

void
RevoluteJoint::setInitialVelocity(const real_type& initialVelocity)
{
  mInitialVelocity = initialVelocity;
}

void
RevoluteJoint::setEnableExternalForce(bool enable)
{
  if (enable == getEnableExternalForce())
    return;
  if (enable)
    mForcePort = MatrixInputPort(this, "force", Size(1, 1), true);
  else
    mForcePort.clear();
}

bool
RevoluteJoint::getEnableExternalForce() const
{
  return !mForcePort.empty();
}

void
RevoluteJoint::init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector& continousState,
                    const PortValueList&) const
{
  continousState[*mPositionStateInfo] = mInitialPosition;
  continousState[*mVelocityStateInfo] = mInitialVelocity;
}

RevoluteJoint::Matrix6N
RevoluteJoint::getJointMatrix() const
{
  return Vector6(mAxis, Vector3::zeros());
}

void
RevoluteJoint::velocity(const Task&, Context& context,
                        const ContinousStateValueVector& states,
                        PortValueList& portValues) const
{
  VectorN jointPos = states[*mPositionStateInfo];
  if (!mPositionPort.empty())
    portValues[mPositionPort] = jointPos;
  
  VectorN jointVel = states[*mVelocityStateInfo];
  if (!mVelocityPort.empty())
    portValues[mVelocityPort] = jointVel;
  
  Quaternion orientation = Quaternion::fromAngleAxis(jointPos(0), mAxis);
  context.setPosAndVel(Vector3::zeros(), orientation, jointVel);
}

void
RevoluteJoint::articulation(const Task&, Context& context,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues) const
{
  VectorN jointForce;
  if (mForcePort.empty())
    jointForce.clear();
  else
    jointForce = portValues[mForcePort];
  
  context.applyJointForce(jointForce);
}

void
RevoluteJoint::acceleration(const Task&, Context& context,
                            const ContinousStateValueVector&,
                            PortValueList&) const
{
  context.accelerateDueToForce();
}

void
RevoluteJoint::derivative(const Task&, Context& context,
                          const ContinousStateValueVector& states,
                          const PortValueList&,
                          ContinousStateValueVector& derivative) const
{
  derivative[*mPositionStateInfo] = states[*mVelocityStateInfo];
  derivative[*mVelocityStateInfo] = context.getVelDot();
}

} // namespace OpenFDM
