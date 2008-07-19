/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "PrismaticJoint.h"

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

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(PrismaticJoint, Joint)
  END_OPENFDM_OBJECT_DEF

BEGIN_OPENFDM_OBJECT_DEF(PrismaticJointFrame, Frame)
  END_OPENFDM_OBJECT_DEF

PrismaticJoint::PrismaticJoint(const std::string& name)
  : Joint(name)
{
  setNumContinousStates(2);

  mPrismaticJointFrame = new PrismaticJointFrame(name);

  setNumInputPorts(1);
  setInputPortName(0, "jointForce");

  // Since these output ports are just fed by the current state of the
  // multibody system, we do not have a direct feedthrough model
  setNumOutputPorts(2);
  setOutputPort(0, "jointPos", this, &PrismaticJoint::getJointPos);
  setOutputPort(1, "jointVel", this, &PrismaticJoint::getJointVel);
}

PrismaticJoint::~PrismaticJoint(void)
{
}

bool
PrismaticJoint::init(void)
{
  /// Check if we have an input port connected to the joint force ...
  if (getInputPort(0))
    mJointForcePort = getInputPort(0)->toRealPortHandle();
  else
    mJointForcePort = 0;

  recheckTopology();
  return Joint::init();
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
  if (nrm <= Limits<real_type>::min()) {
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
  if (mJointForcePort.isConnected()) {
    tau(0) = mJointForcePort.getRealValue();
  } else
    tau.clear();
  mPrismaticJointFrame->jointArticulation(artI, artF, outF, outI, tau);
}

void
PrismaticJoint::setState(const StateStream& state)
{
  CartesianJointFrame<1>::VectorN v;
  state.readSubState(v);
  mPrismaticJointFrame->setJointPos(v(0));
  state.readSubState(v);
  mPrismaticJointFrame->setJointVel(v(0));
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
