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
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, VelocityControl, Serialized)
  DEF_OPENFDM_PROPERTY(Real, MaxVel, Serialized)
  DEF_OPENFDM_PROPERTY(Real, VelGain, Serialized)
  DEF_OPENFDM_PROPERTY(Real, VelDotGain, Serialized)
  END_OPENFDM_OBJECT_DEF

RevoluteActuator::RevoluteActuator(const std::string& name) :
  CartesianJoint<1>(name),
  mInputPort(this, "input", Size(1, 1), true),
  mPositionPort(this, "position", Size(1, 1)),
  mVelocityPort(this, "velocity", Size(1, 1)),
  mPositionStateInfo(new Vector1StateInfo),
  mVelocityStateInfo(new Vector1StateInfo),
  mAxis(Vector3(1, 0, 0)),
  mVelocityControl(false),
  mVelGain(1),
  mVelDotGain(1),
  mMaxVel(1)
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

const Vector3&
RevoluteActuator::getPosition() const
{
  return CartesianJoint<1>::getPosition();
}

void
RevoluteActuator::setPosition(const Vector3& position)
{
  CartesianJoint<1>::setPosition(position);
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
  
  Vector3 position = getPosition() - parentLink.getDesignPosition();
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

  // This is a simple second order system with velocity limits.
  // the joints accelerations, velocities and positions must fit together
  // otherwise the articulated body dynamics get fooled ...

  if (!mVelocityControl) {
    // The desired position input
    VectorN desiredPos = portValues[mInputPort];
    // Compute the error ...
    VectorN posErr = desiredPos - states[*mPositionStateInfo];
    // ... and compute a desired velocity within the given limits from that.
    VectorN desiredVel;
    desiredVel(0) = smoothSaturate(mVelGain*posErr(0), mMaxVel);
    // The usual control loops: there we get a velocity error
    VectorN velErr = desiredVel - states[*mVelocityStateInfo];
    // and accelerate that proportional to that error ...
    velDot = mVelDotGain*velErr;
  } else {
    // The desired velocity input
    VectorN desiredVel = portValues[mInputPort];
    // The usual control loops: there we get a velocity error
    VectorN velErr = desiredVel - states[*mVelocityStateInfo];
    // and accelerate that proportional to that error ...
    velDot = mVelDotGain*velErr;
  }

  articulation(parentLink, childLink, velDot);
}

void
RevoluteActuator::acceleration(const MechanicLinkValue& parentLink,
                               MechanicLinkValue& childLink,
                               const ContinousStateValueVector& states,
                               PortValueList& portValues,
                               const Matrix&, VectorN& velDot) const
{
  // This is a simple second order system with velocity limits.
  // the joints accelerations, velocities and positions must fit together
  // otherwise the articulated body dynamics get fooled ...

  if (!mVelocityControl) {
    // The desired position input
    VectorN desiredPos = portValues[mInputPort];
    // Compute the error ...
    VectorN posErr = desiredPos - states[*mPositionStateInfo];
    // ... and compute a desired velocity within the given limits from that.
    VectorN desiredVel;
    desiredVel(0) = smoothSaturate(mVelGain*posErr(0), mMaxVel);
    // The usual control loops: there we get a velocity error
    VectorN velErr = desiredVel - states[*mVelocityStateInfo];
    // and accelerate that proportional to that error ...
    velDot = mVelDotGain*velErr;
  } else {
    // The desired velocity input
    VectorN desiredVel = portValues[mInputPort];
    // The usual control loops: there we get a velocity error
    VectorN velErr = desiredVel - states[*mVelocityStateInfo];
    // and accelerate that proportional to that error ...
    velDot = mVelDotGain*velErr;
  }

  acceleration(parentLink, childLink, velDot);
}

void
RevoluteActuator::derivative(const DiscreteStateValueVector&,
                             const ContinousStateValueVector& states,
                             const PortValueList&, const VectorN& velDot,
                             ContinousStateValueVector& derivative) const
{
  derivative[*mPositionStateInfo] = states[*mVelocityStateInfo];
  derivative[*mVelocityStateInfo] = velDot;
}

} // namespace OpenFDM
