/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
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
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(RevoluteActuator, Joint)
  DEF_OPENFDM_PROPERTY(Vector3, Axis, Serialized)
  DEF_OPENFDM_PROPERTY(Real, InitialPosition, Serialized)
  DEF_OPENFDM_PROPERTY(Real, InitialVelocity, Serialized)
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
  mInitialPosition(0),
  mInitialVelocity(0),
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
}

const real_type&
RevoluteActuator::getInitialPosition() const
{
  return mInitialPosition;
}

void
RevoluteActuator::setInitialPosition(const real_type& initialPosition)
{
  mInitialPosition = initialPosition;
}

const real_type&
RevoluteActuator::getInitialVelocity() const
{
  return mInitialVelocity;
}

void
RevoluteActuator::setInitialVelocity(const real_type& initialVelocity)
{
  mInitialVelocity = initialVelocity;
}

const bool&
RevoluteActuator::getVelocityControl(void) const
{
  return mVelocityControl;
}

void
RevoluteActuator::setVelocityControl(const bool& velocityControl)
{
  mVelocityControl = velocityControl;
}

const real_type&
RevoluteActuator::getMaxVel(void) const
{
  return mMaxVel;
}

void
RevoluteActuator::setMaxVel(const real_type& maxVel)
{
  mMaxVel = maxVel;
}

const real_type&
RevoluteActuator::getVelGain(void) const
{
  return mVelGain;
}

void
RevoluteActuator::setVelGain(const real_type& velGain)
{
  mVelGain = velGain;
}

const real_type&
RevoluteActuator::getVelDotGain(void) const
{
  return mVelDotGain;
}

void
RevoluteActuator::setVelDotGain(const real_type& velDotGain)
{
  mVelDotGain = velDotGain;
}

void
RevoluteActuator::init(const Task&, DiscreteStateValueVector&,
                       ContinousStateValueVector& continousState,
                       const PortValueList&) const
{
  continousState[*mPositionStateInfo] = mInitialPosition;
  continousState[*mVelocityStateInfo] = mInitialVelocity;
}

RevoluteActuator::Matrix6N
RevoluteActuator::getJointMatrix() const
{
  return Vector6(mAxis, Vector3::zeros());
}

void
RevoluteActuator::velocity(const Task&, Context& context,
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
RevoluteActuator::articulation(const Task&, Context& context,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues) const
{
  // This is a simple second order system with velocity limits.
  // the joints accelerations, velocities and positions must fit together
  // otherwise the articulated body dynamics get fooled ...

  VectorN velDot;
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

  context.applyActuatorForce(velDot);
}

void
RevoluteActuator::acceleration(const Task&, Context& context,
                            const ContinousStateValueVector&,
                            PortValueList&) const
{
  context.accelerateDueToVelDot();
}

void
RevoluteActuator::derivative(const Task&, Context& context,
                          const ContinousStateValueVector& states,
                          const PortValueList&,
                          ContinousStateValueVector& derivative) const
{
  derivative[*mPositionStateInfo] = states[*mVelocityStateInfo];
  derivative[*mVelocityStateInfo] = context.getVelDot();
}

} // namespace OpenFDM
