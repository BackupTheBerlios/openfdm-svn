/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

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
#include "RevoluteActuator.h"

namespace OpenFDM {

RevoluteActuator::RevoluteActuator(const std::string& name) :
  Joint(name),
  mMaxVel(1),
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
  if (!outFrame->isParentFrame(inFrame)) {
    inFrame->addChildFrame(mRevoluteActuatorFrame);
  }
}

void
RevoluteActuator::setJointAxis(const Vector3& axis)
{
  real_type nrm = norm(axis);
  if (nrm < Limits<real_type>::min()) {
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
  OpenFDMAssert(getInputPort(0)->isConnected());
  RealPortHandle rh = getInputPort(0)->toRealPortHandle();
  real_type desiredPos = rh.getRealValue();

  real_type posErr = desiredPos - mRevoluteActuatorFrame->getJointPos();

  real_type desiredVel = mVelGain*sign(posErr)*min(fabs(posErr), mMaxVel);

  real_type velErr = desiredVel - mRevoluteActuatorFrame->getJointVel();

  CartesianActuatorFrame<1>::VectorN tau;
  tau(1) = mVelDotGain*velErr;
  mRevoluteActuatorFrame->jointArticulation(artI, artF, outF, outI, tau);
}

void
RevoluteActuator::setState(const Vector& state, unsigned offset)
{
  mRevoluteActuatorFrame->setJointPos(state(offset+1));
  mRevoluteActuatorFrame->setJointVel(state(offset+2));
}

void
RevoluteActuator::getState(Vector& state, unsigned offset) const
{
  state(offset+1) = mRevoluteActuatorFrame->getJointPos();
  state(offset+2) = mRevoluteActuatorFrame->getJointVel();
}

void
RevoluteActuator::getStateDeriv(Vector& state, unsigned offset)
{
  state(offset+1) = mRevoluteActuatorFrame->getJointVel();
  state(offset+2) = mRevoluteActuatorFrame->getJointVelDot()(1);
}

} // namespace OpenFDM
