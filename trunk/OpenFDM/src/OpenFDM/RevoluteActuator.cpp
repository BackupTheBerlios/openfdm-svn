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
#include "Frame.h"
#include "RigidBody.h"
#include "RevoluteActuatorFrame.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(RevoluteActuator, Joint)
  DEF_OPENFDM_PROPERTY(Real, MaxVel, Serialized)
  DEF_OPENFDM_PROPERTY(Real, VelGain, Serialized)
  DEF_OPENFDM_PROPERTY(Real, VelDotGain, Serialized)
  END_OPENFDM_OBJECT_DEF

BEGIN_OPENFDM_OBJECT_DEF(RevoluteActuatorFrame, Frame)
  END_OPENFDM_OBJECT_DEF

RevoluteActuator::RevoluteActuator(const std::string& name) :
  Joint(name),
  mMaxVel(10),
  mVelGain(1e2),
  mVelDotGain(1e2)
{
  setNumContinousStates(2);

  mRevoluteActuatorFrame = new RevoluteActuatorFrame(name);

  setNumInputPorts(1);
  setInputPortName(0, "position");

  setNumOutputPorts(2);
  setOutputPort(0, "jointPos", this, &RevoluteActuator::getJointPos);
  setOutputPort(1, "jointVel", this, &RevoluteActuator::getJointVel);
}

RevoluteActuator::~RevoluteActuator(void)
{
}

bool
RevoluteActuator::init(void)
{
  mDesiredPositionPort = getInputPort(0)->toRealPortHandle();
  if (!mDesiredPositionPort.isConnected()) {
    Log(Model, Error) << "Initialization of RevoluteActuator model \""
                      << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }

  recheckTopology();
  return Joint::init();
}

void
RevoluteActuator::recheckTopology(void)
{
  if (!getOutboardBody() || !getInboardBody())
    return;
  
  // check for the inboard frame
  Frame* inFrame = getInboardBody()->getFrame();
  if (!inFrame)
    return;
  
  Frame* outFrame = getOutboardBody()->getFrame();
  if (!outFrame) {
    getOutboardBody()->setFrame(mRevoluteActuatorFrame);
  }
  outFrame = getOutboardBody()->getFrame();
  if (!outFrame->isDirectChildFrameOf(inFrame)) {
    inFrame->addChildFrame(mRevoluteActuatorFrame);
  }
}

void
RevoluteActuator::setJointAxis(const Vector3& axis)
{
  real_type nrm = norm(axis);
  if (nrm <= Limits<real_type>::min()) {
    Log(Initialization, Error) << "JointAxis is zero ..." << endl;
    return;
  }
  mRevoluteActuatorFrame->setJointAxis((1/nrm)*axis);
}

const real_type&
RevoluteActuator::getJointPos(void) const
{
  return mRevoluteActuatorFrame->getJointPos();
}

void
RevoluteActuator::setJointPos(real_type pos)
{
  mRevoluteActuatorFrame->setJointPos(pos);
}

const real_type&
RevoluteActuator::getJointVel(void) const
{
  return mRevoluteActuatorFrame->getJointVel();
}

void
RevoluteActuator::setJointVel(real_type vel)
{
  mRevoluteActuatorFrame->setJointVel(vel);
}

void
RevoluteActuator::setPosition(const Vector3& position)
{
  mRevoluteActuatorFrame->setPosition(position);
}

void
RevoluteActuator::setOrientation(const Quaternion& orientation)
{
  mRevoluteActuatorFrame->setZeroOrientation(orientation);
}

void
RevoluteActuator::jointArticulation(SpatialInertia& artI, Vector6& artF,
                                 const SpatialInertia& outI,
                                 const Vector6& outF)
{
  // This is a simple second order system with velocity limits.
  // the joints accelerations, velocities and positions must fit together
  // otherwise the articulated body dynamics get fooled ...

  // The desired position input
  real_type desiredPos = mDesiredPositionPort.getRealValue();
  // Compute the error ...
  real_type posErr = desiredPos - mRevoluteActuatorFrame->getJointPos();
  // ... and compute a desired velocity within the given limits from that.
  real_type desiredVel = smoothSaturate(mVelGain*posErr, mMaxVel);
  // The usual control loops: there we get a velocity error
  real_type velErr = desiredVel - mRevoluteActuatorFrame->getJointVel();
  // and accelerate that proportional to that error ...
  mRevoluteActuatorFrame->setJointVelDot(mVelDotGain*velErr);

  // now that the joints acceleration is known, compute the articulated
  // body force and inertia ...
  mRevoluteActuatorFrame->jointArticulation(artI, artF, outF, outI);
}

void
RevoluteActuator::setState(const StateStream& state)
{
  CartesianActuatorFrame<1>::VectorN v;
  state.readSubState(v);
  mRevoluteActuatorFrame->setJointPos(v(0));
  state.readSubState(v);
  mRevoluteActuatorFrame->setJointVel(v(0));
}

void
RevoluteActuator::getState(StateStream& state) const
{
  state.writeSubState(mRevoluteActuatorFrame->getJointPos());
  state.writeSubState(mRevoluteActuatorFrame->getJointVel());
}

void
RevoluteActuator::getStateDeriv(StateStream& stateDeriv)
{
  stateDeriv.writeSubState(mRevoluteActuatorFrame->getJointVel());
  stateDeriv.writeSubState(mRevoluteActuatorFrame->getJointVelDot());
}

} // namespace OpenFDM
