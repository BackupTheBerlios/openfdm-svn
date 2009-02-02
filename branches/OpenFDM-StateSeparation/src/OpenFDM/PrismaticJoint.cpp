/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "PrismaticJoint.h"
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

BEGIN_OPENFDM_OBJECT_DEF(PrismaticJoint, Joint)
  DEF_OPENFDM_PROPERTY(Vector3, Axis, Serialized)
  DEF_OPENFDM_PROPERTY(Real, InitialPosition, Serialized)
  DEF_OPENFDM_PROPERTY(Real, InitialVelocity, Serialized)
  END_OPENFDM_OBJECT_DEF

PrismaticJoint::PrismaticJoint(const std::string& name) :
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

  // FIXME
  setAxis(mAxis);
}

PrismaticJoint::~PrismaticJoint(void)
{
}

const Vector3&
PrismaticJoint::getAxis() const
{
  return mAxis;
}

void
PrismaticJoint::setAxis(const Vector3& axis)
{
  real_type nrm = norm(axis);
  if (nrm <= Limits<real_type>::min()) {
    Log(Initialization, Error) << "JointAxis is zero ..." << endl;
    return;
  }
  mAxis = (1/nrm)*axis;
}

const real_type&
PrismaticJoint::getInitialPosition() const
{
  return mInitialPosition;
}

void
PrismaticJoint::setInitialPosition(const real_type& initialPosition)
{
  mInitialPosition = initialPosition;
}

const real_type&
PrismaticJoint::getInitialVelocity() const
{
  return mInitialVelocity;
}

void
PrismaticJoint::setInitialVelocity(const real_type& initialVelocity)
{
  mInitialVelocity = initialVelocity;
}

void
PrismaticJoint::setEnableExternalForce(bool enable)
{
  if (enable == getEnableExternalForce())
    return;
  if (enable)
    mForcePort = MatrixInputPort(this, "force", Size(1, 1), true);
  else
    mForcePort.clear();
}

bool
PrismaticJoint::getEnableExternalForce() const
{
  return !mForcePort.empty();
}

void
PrismaticJoint::init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector& continousState,
                    const PortValueList&) const
{
  continousState[*mPositionStateInfo] = mInitialPosition;
  continousState[*mVelocityStateInfo] = mInitialVelocity;
}

PrismaticJoint::Matrix6N
PrismaticJoint::getJointMatrix() const
{
  return Vector6(Vector3::zeros(), mAxis);
}

void
PrismaticJoint::velocity(const Task&, Context& context,
                         const ContinousStateValueVector& states,
                         PortValueList& portValues) const
{
  VectorN jointPos = states[*mPositionStateInfo];
  if (!mPositionPort.empty())
    portValues[mPositionPort] = jointPos;
  
  VectorN jointVel = states[*mVelocityStateInfo];
  if (!mVelocityPort.empty())
    portValues[mVelocityPort] = jointVel;
  
  context.setPosAndVel(mAxis*jointPos, Quaternion::unit(), jointVel);
}

void
PrismaticJoint::articulation(const Task&, Context& context,
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
PrismaticJoint::acceleration(const Task&, Context& context,
                             const ContinousStateValueVector&,
                             PortValueList&) const
{
  context.accelerateDueToForce();
}

void
PrismaticJoint::derivative(const Task&, Context& context,
                           const ContinousStateValueVector& states,
                           const PortValueList&,
                           ContinousStateValueVector& derivative) const
{
  derivative[*mPositionStateInfo] = states[*mVelocityStateInfo];
  derivative[*mVelocityStateInfo] = context.getVelDot();
}

} // namespace OpenFDM
