/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "RevoluteActuator.h"
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

BEGIN_OPENFDM_OBJECT_DEF(RevoluteActuator, Joint)
  DEF_OPENFDM_PROPERTY(Vector3, Axis, Serialized)
  END_OPENFDM_OBJECT_DEF

RevoluteActuator::RevoluteActuator(const std::string& name) :
  CartesianJoint<1>(name, Vector6(Vector3(1, 0, 0), Vector3::zeros())),
  mInputPort(this, "input", Size(1, 1), true),
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

RevoluteActuator::~RevoluteActuator(void)
{
}

const Vector3&
RevoluteActuator::getAxis() const
{
  return mAxis;
}

void
RevoluteActuator::setAxis(const Vector3& axis)
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
RevoluteActuator::init(const Task&, DiscreteStateValueVector&,
                       ContinousStateValueVector& continousState,
                       const PortValueList&) const
{
  continousState[*mPositionStateInfo] = 0;
  continousState[*mVelocityStateInfo] = 0;
}

void
RevoluteActuator::velocity(const MechanicLinkValue& parentLink,
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
RevoluteActuator::articulation(MechanicLinkValue& parentLink,
                            const MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            Matrix&) const
{
  VectorN velDot;
  velDot.clear();
  articulation(parentLink, childLink, velDot);
}

void
RevoluteActuator::acceleration(const MechanicLinkValue& parentLink,
                               MechanicLinkValue& childLink,
                               const ContinousStateValueVector& states,
                               PortValueList& portValues,
                               const Matrix&, Vector& _velDot) const
{
  VectorN velDot;
  velDot.clear();
  _velDot = velDot;
  acceleration(parentLink, childLink, velDot);
}

void
RevoluteActuator::derivative(const DiscreteStateValueVector&,
                             const ContinousStateValueVector& states,
                             const PortValueList&, const Vector& velDot,
                             ContinousStateValueVector& derivative) const
{
  derivative[*mPositionStateInfo] = states[*mVelocityStateInfo];
  derivative[*mVelocityStateInfo] = velDot;
}

} // namespace OpenFDM
