/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Limits.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "PrismaticJointFrame.h"
#include "PrismaticJoint.h"

namespace OpenFDM {

PrismaticJoint::PrismaticJoint(const std::string& name)
  : Joint(name)
{
  setNumContinousStates(2);

  mPrismaticJointFrame = new PrismaticJointFrame(name);

  setNumOutputPorts(2);
  setOutputPort(0, "jointPos", this, &PrismaticJoint::getJointPos);
  setOutputPort(1, "jointVel", this, &PrismaticJoint::getJointVel);
}

PrismaticJoint::~PrismaticJoint(void)
{
}

void
PrismaticJoint::recheckTopology(void)
{
  if (!getOutboardBody() || !getInboardBody())
    return;
  
  // check for the inboard frame
  Frame* inFrame = getInboardBody()->getFrame();
  if (!inFrame)
    return;
  
  Frame* outFrame = getOutboardBody()->getFrame();
  if (!outFrame) {
    getOutboardBody()->setFrame(mPrismaticJointFrame);
  }
  outFrame = getOutboardBody()->getFrame();
  if (!outFrame->isDirectChildFrameOf(inFrame)) {
    inFrame->addChildFrame(mPrismaticJointFrame);
  }
}

void
PrismaticJoint::setJointAxis(const Vector3& axis)
{
  real_type nrm = norm(axis);
  if (nrm < Limits<real_type>::min()) {
    Log(Initialization, Error) << "JointAxis is zero ..." << endl;
    return;
  }

  mPrismaticJointFrame->setJointAxis((1/nrm)*axis);
}

const real_type&
PrismaticJoint::getJointPos(void) const
{
  return mPrismaticJointFrame->getJointPos();
}

void
PrismaticJoint::setJointPos(real_type pos)
{
  mPrismaticJointFrame->setJointPos(pos);
}

void
PrismaticJoint::setJointVel(real_type vel)
{
  mPrismaticJointFrame->setJointVel(vel);
}

const real_type&
PrismaticJoint::getJointVel(void) const
{
  return mPrismaticJointFrame->getJointVel();
}

void
PrismaticJoint::setOrientation(const Quaternion& orientation)
{
  mPrismaticJointFrame->setOrientation(orientation);
}

void
PrismaticJoint::setPosition(const Vector3& position)
{
  mPrismaticJointFrame->setZeroPosition(position);
}

void
PrismaticJoint::jointArticulation(SpatialInertia& artI, Vector6& artF,
                                 const SpatialInertia& outI,
                                 const Vector6& outF)
{
  // That projects away tha components where the degrees of freedom
  // of the joint are.
  CartesianJointFrame<1>::VectorN tau;
  tau(1) = getJointForce();
  mPrismaticJointFrame->jointArticulation(artI, artF, outF, outI, tau);
}

void
PrismaticJoint::setState(const StateStream& state)
{
  CartesianJointFrame<1>::VectorN v;
  state.readSubState(v);
  mPrismaticJointFrame->setJointPos(v(1));
  state.readSubState(v);
  mPrismaticJointFrame->setJointVel(v(1));
}

void
PrismaticJoint::getState(StateStream& state) const
{
  state.writeSubState(mPrismaticJointFrame->getJointPos());
  state.writeSubState(mPrismaticJointFrame->getJointVel());
}

void
PrismaticJoint::getStateDeriv(StateStream& stateDeriv)
{
  stateDeriv.writeSubState(mPrismaticJointFrame->getJointVel());
  stateDeriv.writeSubState(mPrismaticJointFrame->getJointVelDot());
}

} // namespace OpenFDM
