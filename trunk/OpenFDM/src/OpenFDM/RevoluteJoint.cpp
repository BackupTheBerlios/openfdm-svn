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

  setNumOutputPorts(2);
  setOutputPort(0, "jointPos", this, &RevoluteJoint::getJointPos);
  setOutputPort(1, "jointVel", this, &RevoluteJoint::getJointVel);
}

RevoluteJoint::~RevoluteJoint(void)
{
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
  if (!outFrame->isParentFrame(inFrame)) {
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
  tau(1) = getJointForce();
  mRevoluteJointFrame->jointArticulation(artI, artF, outF, outI, tau);
}

void
RevoluteJoint::setState(const Vector& state, unsigned offset)
{
  mRevoluteJointFrame->setJointPos(state(offset+1));
  mRevoluteJointFrame->setJointVel(state(offset+2));
}

void
RevoluteJoint::getState(Vector& state, unsigned offset) const
{
  state(offset+1) = mRevoluteJointFrame->getJointPos();
  state(offset+2) = mRevoluteJointFrame->getJointVel();
}

void
RevoluteJoint::getStateDeriv(Vector& state, unsigned offset)
{
  state(offset+1) = mRevoluteJointFrame->getJointVel();
  state(offset+2) = mRevoluteJointFrame->getJointVelDot()(1);
}

} // namespace OpenFDM
