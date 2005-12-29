/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Gravity.h"
#include "Frame.h"
#include "RigidBody.h"
#include "RootFrame.h"
#include "MobileRootJointFrame.h"
#include "MobileRootJoint.h"

namespace OpenFDM {

MobileRootJoint::MobileRootJoint(const std::string& name)
  : Joint(name),
    mFrame(new MobileRootJointFrame(name))
{
  setNumContinousStates(13);
  addSampleTime(SampleTime::Continous);
}

MobileRootJoint::~MobileRootJoint(void)
{
}

bool
MobileRootJoint::init(void)
{
  Environment* environment = getEnvironment();
  if (!environment) {
    Log(Model,Error) << "Can not get environment pointer! Most propably the"
      " Model is not put together correctly!" << endl;
    return false;
  }
  mGravity = environment->getGravity();
  if (!mGravity) {
    Log(Model,Error) << "Can not get gravity model!" << endl;
    return false;
  }
  Frame* rootFrame = environment->getRootFrame();
  if (!rootFrame) {
    Log(Model,Error) << "Can not get rootFrame model!" << endl;
    return false;
  }
  recheckTopology();

  return Joint::init();
}

void
MobileRootJoint::recheckTopology(void)
{
  // Hmm, works for the first cut, but rethink what happens with strange
  // attach reattach sequences ...
  RigidBody* rigidBody = getOutboardBody();
  if (!rigidBody)
    return;
  // check if already done
  if (mFrame != rigidBody->getFrame())
    rigidBody->setFrame(mFrame);

  // Check if we are attached to some rigid body ...
  rigidBody = getInboardBody();
  if (rigidBody) {
    Frame* frame = rigidBody->getFrame();
    if (frame && !frame->isParentFrame(mFrame))
      frame->addChildFrame(mFrame);
  } else {
    Environment* environment = getEnvironment();
    if (environment) {
      Frame* rootFrame = environment->getRootFrame();
      if (rootFrame)
        rootFrame->addChildFrame(mFrame);
    }
  }
}

void
MobileRootJoint::setRelVel(const Vector6& vel)
{
  mFrame->setRelVel(vel);
}

void
MobileRootJoint::setLinearRelVel(const Vector3& vel)
{
  mFrame->setLinearRelVel(vel);
}

void
MobileRootJoint::setAngularRelVel(const Vector3& vel)
{
  mFrame->setAngularRelVel(vel);
}

void
MobileRootJoint::setRefPosition(const Vector3& p)
{
  mFrame->setRefPosition(p);
}

void
MobileRootJoint::setRefOrientation(const Quaternion& o)
{
  mFrame->setRefOrientation(o);
}

void
MobileRootJoint::jointArticulation(SpatialInertia& artI, Vector6& artF,
                             const SpatialInertia& outI,
                             const Vector6& outF)
{
  artI.clear();
  artF.clear();

  Log(ArtBody, Debug) << "MobileRootJoint::computeRelVelDot():\n" << outI << endl;
  mFrame->jointArticulation(outF, outI, mGravity);
}

void
MobileRootJoint::setState(const StateStream& state)
{
  Quaternion q;
  state.readSubState(q);
  mFrame->setOrientation(q);
  Vector3 p;
  state.readSubState(p);
  mFrame->setPosition(p);
  Vector6 v;
  state.readSubState(v);
  mFrame->setRelVel(v);
}

void
MobileRootJoint::getState(StateStream& state) const
{
  state.writeSubState(mFrame->getOrientation());
  state.writeSubState(mFrame->getPosition());
  state.writeSubState(mFrame->getRelVel());
}

void
MobileRootJoint::getStateDeriv(StateStream& stateDeriv)
{
  Quaternion q = mFrame->getOrientation();
  Vector3 angVel = mFrame->getRelVel().getAngular();
  Vector3 vel = mFrame->rotToParent(mFrame->getRelVel().getLinear());

  // Compute the derivative term originating from the angular velocity.
  // Correction term to keep the quaternion normalized.
  // That is if |q| < 1 add a little radial component outward,
  // if |q| > 1 add a little radial component inward
  Vector4 qderiv = derivative(q, angVel) + 0.1*(normalize(q) - q);
  stateDeriv.writeSubState(qderiv);
  stateDeriv.writeSubState(vel);
  stateDeriv.writeSubState(mFrame->getRelVelDot());
}

} // namespace OpenFDM
