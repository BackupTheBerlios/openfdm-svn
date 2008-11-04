/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(RevoluteJoint, Joint)
  DEF_OPENFDM_PROPERTY(Vector3, Axis, Serialized)
  END_OPENFDM_OBJECT_DEF

RevoluteJoint::RevoluteJoint(const std::string& name) :
  CartesianJoint<1>(name, Vector6(Vector3(1, 0, 0), Vector3::zeros())),
  mForcePort(this, "force", Size(1, 1), true),
  mPositionPort(this, "position", Size(1, 1)),
  mVelocityPort(this, "velocity", Size(1, 1)),
  mPositionStateInfo(new Vector1StateInfo),
  mVelocityStateInfo(new Vector1StateInfo),
  mAxis(Vector3(1, 0, 0))
{
  addContinousStateInfo(mPositionStateInfo);
  addContinousStateInfo(mVelocityStateInfo);

  // FIXME
  setAxis(mAxis);
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
    Log(Initialization, Error) << "JointAxis is zero ..." << endl;
    return;
  }
  mAxis = (1/nrm)*axis;
  setJointMatrix(Vector6(mAxis, Vector3::zeros()));
}

void
RevoluteJoint::init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector& continousState,
                    const PortValueList&) const
{
  continousState[*mPositionStateInfo] = 0;
  continousState[*mVelocityStateInfo] = 0;
}

void
RevoluteJoint::velocity(const MechanicLinkValue& parentLink,
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
  
  Vector3 position(0, 0, 0);
  Quaternion orientation(Quaternion::fromAngleAxis(jointPos(0), mAxis));

  velocity(parentLink, childLink, position, orientation, getJointMatrix()*jointVel);
}

void
RevoluteJoint::articulation(MechanicLinkValue& parentLink,
                            const MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            Matrix& hIh) const
{
  VectorN jointForce;
  if (mForcePort.empty())
    jointForce.clear();
  else
    jointForce = portValues[mForcePort];
  
  articulation(parentLink, childLink, jointForce, hIh);
}

void
RevoluteJoint::acceleration(const MechanicLinkValue& parentLink,
                            MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            const Matrix& hIh, Vector& velDot) const
{
  VectorN jointForce;
  if (mForcePort.empty())
    jointForce.clear();
  else
    jointForce = portValues[mForcePort];
  
  acceleration(parentLink, childLink, jointForce, hIh, velDot);
}

void
RevoluteJoint::derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector& states,
                          const PortValueList&, const Vector& velDot,
                          ContinousStateValueVector& derivative) const
{
  derivative[*mPositionStateInfo] = states[*mVelocityStateInfo];
  derivative[*mVelocityStateInfo] = velDot;
}

} // namespace OpenFDM
