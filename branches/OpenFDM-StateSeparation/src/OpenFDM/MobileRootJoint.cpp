/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "MobileRootJoint.h"

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
#include "ModelVisitor.h"
#include "RootFrame.h"
#include "MobileRootJointFrame.h"

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

void
MobileRootJoint::accept(ModelVisitor& visitor)
{
  visitor.apply(*this);
}

bool
MobileRootJoint::init(void)
{
  mGravity = mEnvironment->getGravity();
  if (!mGravity) {
    Log(Model,Error) << "Can not get gravity model!" << endl;
    return false;
  }
  const Frame* rootFrame = mEnvironment->getRootFrame();
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
    if (frame && !frame->isDirectParentFrameOf(mFrame))
      frame->addChildFrame(mFrame);
  } else {
    if (mEnvironment) {
      Frame* rootFrame = mEnvironment->getRootFrame();
      if (rootFrame && !rootFrame->isDirectParentFrameOf(mFrame))
        rootFrame->addChildFrame(mFrame);
    }
  }
}

const Vector6&
MobileRootJoint::getRelVel(void) const
{
  return mFrame->getRelVel();
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

const Vector6&
MobileRootJoint::getRelVelDot(void) const
{
  return mFrame->getRelVelDot();
}

const Vector3&
MobileRootJoint::getRefPosition(void) const
{
  return mFrame->getRefPosition();
}

void
MobileRootJoint::setRefPosition(const Vector3& p)
{
  mFrame->setRefPosition(p);
}

const Quaternion&
MobileRootJoint::getRefOrientation(void) const
{
  return mFrame->getRefOrientation();
}

void
MobileRootJoint::setRefOrientation(const Quaternion& o)
{
  mFrame->setRefOrientation(o);
}

Geodetic
MobileRootJoint::getGeodPosition(void) const
{
  if (!mEnvironment)
    return Geodetic();
  return mEnvironment->getPlanet()->toGeod(getRefPosition());
}

void
MobileRootJoint::setGeodPosition(const Geodetic& geod)
{
  if (!mEnvironment)
    return;
  setRefPosition(mEnvironment->getPlanet()->toCart(geod));
}

Quaternion
MobileRootJoint::getGeodOrientation(void) const
{
  if (!mEnvironment)
    return Quaternion::unit();
  Quaternion hlOr = mEnvironment->getPlanet()->getGeodHLOrientation(getRefPosition());
  return inverse(hlOr)*getRefOrientation();
}

Vector4
MobileRootJoint::getQDot(void) const
{
  // Compute the derivative term originating from the angular velocity.
  // Correction term to keep the quaternion normalized.
  // That is if |q| < 1 add a little radial component outward,
  // if |q| > 1 add a little radial component inward
  Quaternion q = mFrame->getOrientation();
  Vector3 angVel = mFrame->getRelVel().getAngular();
  Vector4 qderiv = derivative(q, angVel) + 1e1*(normalize(q) - q);
  return qderiv;
}

Vector3 MobileRootJoint::getPosDot(void) const
{
  return mFrame->rotToParent(mFrame->getRelVel().getLinear());
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
  stateDeriv.writeSubState(getQDot());
  stateDeriv.writeSubState(getPosDot());
  stateDeriv.writeSubState(getRelVelDot());
}

void
MobileRootJoint::setEnvironment(Environment* environment)
{
  mEnvironment = environment;
}

} // namespace OpenFDM
