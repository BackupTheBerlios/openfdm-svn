/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "FixedRootJoint.h"

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
#include "FixedRootJointFrame.h"

namespace OpenFDM {

FixedRootJoint::FixedRootJoint(const std::string& name)
  : Joint(name),
    mFrame(new FixedRootJointFrame(name))
{
}

FixedRootJoint::~FixedRootJoint(void)
{
}

void
FixedRootJoint::accept(ModelVisitor& visitor)
{
  visitor.apply(*this);
}

bool
FixedRootJoint::init(void)
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
FixedRootJoint::recheckTopology(void)
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

const Vector3&
FixedRootJoint::getRefPosition(void) const
{
  return mFrame->getRefPosition();
}

void
FixedRootJoint::setRefPosition(const Vector3& p)
{
  mFrame->setRefPosition(p);
}

const Quaternion&
FixedRootJoint::getRefOrientation(void) const
{
  return mFrame->getRefOrientation();
}

void
FixedRootJoint::setRefOrientation(const Quaternion& o)
{
  mFrame->setRefOrientation(o);
}

Geodetic
FixedRootJoint::getGeodPosition(void) const
{
  if (!mEnvironment)
    return Geodetic();
  return mEnvironment->getPlanet()->toGeod(getRefPosition());
}

void
FixedRootJoint::setGeodPosition(const Geodetic& geod)
{
  if (!mEnvironment)
    return;
  setRefPosition(mEnvironment->getPlanet()->toCart(geod));
}

Quaternion
FixedRootJoint::getGeodOrientation(void) const
{
  if (!mEnvironment)
    return Quaternion::unit();
  Quaternion hlOr = mEnvironment->getPlanet()->getGeodHLOrientation(getRefPosition());
  return inverse(hlOr)*getRefOrientation();
}

void
FixedRootJoint::jointArticulation(SpatialInertia& artI, Vector6& artF,
                             const SpatialInertia& outI,
                             const Vector6& outF)
{
  artI.clear();
  artF.clear();

  Log(ArtBody, Debug) << "FixedRootJoint::computeRelVelDot():\n" << outI << endl;
  mFrame->jointArticulation(outF, outI, mGravity);
}

void
FixedRootJoint::setEnvironment(Environment* environment)
{
  mEnvironment = environment;
}

} // namespace OpenFDM
