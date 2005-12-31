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
#include "RevoluteJointFrame.h"
#include "RevoluteJoint.h"

namespace OpenFDM {

RevoluteJoint::RevoluteJoint(const std::string& name) :
  Joint(name)
{
  setNumContinousStates(2);

  mRevoluteJointFrame = new RevoluteJointFrame(name);

  setNumInputPorts(1);
  setInputPortName(0, "jointForce");

  setNumOutputPorts(2);
  setOutputPort(0, "jointPos", this, &RevoluteJoint::getJointPos);
  setOutputPort(1, "jointVel", this, &RevoluteJoint::getJointVel);
}

RevoluteJoint::~RevoluteJoint(void)
{
}

bool
RevoluteJoint::init(void)
{
  /// Check if we have an input port connected to the joint force ...
  if (getInputPort(0)->isConnected())
    mJointForcePort = getInputPort(0)->toRealPortHandle();
  else
    mJointForcePort = 0;

  recheckTopology();
  return Joint::init();
}

void
RevoluteJoint::recheckTopology(void)
{
  if (!getOutboardBody() || !getInboardBody())
    return;
  
  // check for the inboard frame
  Frame* inFrame = getInboardBody()->getFrame();
  if (!inFrame)
    return;
  
  Frame* outFrame = getOutboardBody()->getFrame();
  if (!outFrame) {
    getOutboardBody()->setFrame(mRevoluteJointFrame);
  }
  outFrame = getOutboardBody()->getFrame();
  if (!outFrame->isDirectChildFrameOf(inFrame)) {
    inFrame->addChildFrame(mRevoluteJointFrame);
  }
}

void
RevoluteJoint::setJointAxis(const Vector3& axis)
{
  real_type nrm = norm(axis);
  if (nrm < Limits<real_type>::min()) {
    Log(Initialization, Error) << "JointAxis is zero ..." << endl;
    return;
  }
  mRevoluteJointFrame->setJointAxis((1/nrm)*axis);
}

const real_type&
RevoluteJoint::getJointPos(void) const
{
  return mRevoluteJointFrame->getJointPos();
}

void
RevoluteJoint::setJointPos(real_type pos)
{
  mRevoluteJointFrame->setJointPos(pos);
}

const real_type&
RevoluteJoint::getJointVel(void) const
{
  return mRevoluteJointFrame->getJointVel();
}

void
RevoluteJoint::setJointVel(real_type vel)
{
  mRevoluteJointFrame->setJointVel(vel);
}

void
RevoluteJoint::setPosition(const Vector3& position)
{
  mRevoluteJointFrame->setPosition(position);
}

void
RevoluteJoint::setOrientation(const Quaternion& orientation)
{
  mRevoluteJointFrame->setZeroOrientation(orientation);
}

void
RevoluteJoint::jointArticulation(SpatialInertia& artI, Vector6& artF,
                                 const SpatialInertia& outI,
                                 const Vector6& outF)
{
  CartesianJointFrame<1>::VectorN tau;
  if (mJointForcePort.isConnected()) {
    tau(1) = mJointForcePort.getRealValue();
  } else
    tau.clear();
  /// FIXME the old obsolete joint force computation
  tau(1) += getJointForce();
  mRevoluteJointFrame->jointArticulation(artI, artF, outF, outI, tau);
}

void
RevoluteJoint::setState(const StateStream& state)
{
  CartesianJointFrame<1>::VectorN v;
  state.readSubState(v);
  mRevoluteJointFrame->setJointPos(v(1));
  state.readSubState(v);
  mRevoluteJointFrame->setJointVel(v(1));
}

void
RevoluteJoint::getState(StateStream& state) const
{
  state.writeSubState(mRevoluteJointFrame->getJointPos());
  state.writeSubState(mRevoluteJointFrame->getJointVel());
}

void
RevoluteJoint::getStateDeriv(StateStream& stateDeriv)
{
  stateDeriv.writeSubState(mRevoluteJointFrame->getJointVel());
  stateDeriv.writeSubState(mRevoluteJointFrame->getJointVelDot());
}

} // namespace OpenFDM
