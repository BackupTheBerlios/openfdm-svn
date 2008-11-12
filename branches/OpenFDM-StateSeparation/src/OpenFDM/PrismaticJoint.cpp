/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(PrismaticJoint, Joint)
  DEF_OPENFDM_PROPERTY(Vector3, Axis, Serialized)
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  END_OPENFDM_OBJECT_DEF

PrismaticJoint::PrismaticJoint(const std::string& name) :
  CartesianJoint<1>(name),
  mPositionPort(this, "position", Size(1, 1)),
  mVelocityPort(this, "velocity", Size(1, 1)),
  mPositionStateInfo(new Vector1StateInfo),
  mVelocityStateInfo(new Vector1StateInfo),
  mAxis(Vector3(1, 0, 0)),
  mPosition(Vector3(0, 0, 0))
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
  setJointMatrix(Vector6(Vector3::zeros(), mAxis));
}

const Vector3&
PrismaticJoint::getPosition() const
{
  return mPosition;
}

void
PrismaticJoint::setPosition(const Vector3& position)
{
  mPosition = position;
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
PrismaticJoint::initDesignPosition(const MechanicLinkValue& parentLink,
                                   MechanicLinkValue& childLink) const
{
  childLink.setDesignPosition(mPosition);
}

void
PrismaticJoint::init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector& continousState,
                    const PortValueList&) const
{
  continousState[*mPositionStateInfo] = 0;
  continousState[*mVelocityStateInfo] = 0;
}

void
PrismaticJoint::velocity(const MechanicLinkValue& parentLink,
                        MechanicLinkValue& childLink,
                        const ContinousStateValueVector& states,
                        PortValueList& portValues) const
{
  VectorN jointPos = states[*mPositionStateInfo];
  if (!mPositionPort.empty())
    portValues[mPositionPort] = jointPos;
  
  VectorN jointVel = states[*mVelocityStateInfo];
  if (!mVelocityPort.empty())
    portValues[mVelocityPort] = jointVel;
  
  Vector3 position = mAxis*jointPos + mPosition - parentLink.getDesignPosition();
  velocity(parentLink, childLink, position, Quaternion::unit(),
           getJointMatrix()*jointVel);
}

void
PrismaticJoint::articulation(MechanicLinkValue& parentLink,
                            const MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            MatrixFactorsNN& hIh) const
{
  VectorN jointForce;
  if (mForcePort.empty())
    jointForce.clear();
  else
    jointForce = portValues[mForcePort];
  
  articulation(parentLink, childLink, jointForce, hIh);
}

void
PrismaticJoint::acceleration(const MechanicLinkValue& parentLink,
                            MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            const MatrixFactorsNN& hIh, VectorN& velDot) const
{
  VectorN jointForce;
  if (mForcePort.empty())
    jointForce.clear();
  else
    jointForce = portValues[mForcePort];
  
  acceleration(parentLink, childLink, jointForce, hIh, velDot);
}

void
PrismaticJoint::derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector& states,
                          const PortValueList&, const VectorN& velDot,
                          ContinousStateValueVector& derivative) const
{
  derivative[*mPositionStateInfo] = states[*mVelocityStateInfo];
  derivative[*mVelocityStateInfo] = velDot;
}

} // namespace OpenFDM
