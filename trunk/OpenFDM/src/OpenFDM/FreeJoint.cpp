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
#include "FreeJoint.h"

namespace OpenFDM {

FreeJoint::FreeJoint(const std::string& name)
  : Joint(name),
    mFrame(new FreeFrame(name))
{
  setNumContinousStates(13);
  addSampleTime(SampleTime::Continous);
}

FreeJoint::~FreeJoint(void)
{
}

bool
FreeJoint::init(void)
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
FreeJoint::recheckTopology(void)
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

bool
FreeJoint::jointArticulation(SpatialInertia& artI, Vector6& artF)
{
  artI = SpatialInertia::zeros();
  artF = Vector6::zeros();
}

Vector6
FreeJoint::computeRelVelDot(const SpatialInertia& artI,
                            const Vector6& artF)
{
  RigidBody* topBody = getParentRigidBody(0);
  OpenFDMAssert(topBody);

  Log(ArtBody, Debug) << "FreeJoint::computeRelVelDot():\n" << artI << endl;

  // Assumption: body is small compared to the distance to the planets
  // center of mass. That means gravity could be considered equal for the whole
  // vehicle.
  // See Featherstone, Orin: Equations and Algorithms
  Vector3 ga = mGravity->gravityAccel(mFrame->getRefPosition());
  Vector6 grav = Vector6(Vector3::zeros(), mFrame->rotFromRef(ga));

  Log(ArtBody, Debug) << "grav = " << trans(grav) << endl
                      << "solve = " << trans(solve(artI, artF)) << endl
                      << "parent spatial accel = "
                      << trans(mFrame->getParentSpAccel()) << endl
                      << "Hdot = " << trans(mFrame->getHdot()) << endl;
  
  Vector6 accel = grav - solve(artI, artF)
    - mFrame->getParentSpAccel() - mFrame->getHdot();
  return accel;
}

void
FreeJoint::setState(const Vector& state, unsigned offset)
{
  mFrame->setOrientation(Vector4(state(offset+1), state(offset+2),
                                 state(offset+3), state(offset+4)));
  mFrame->setPosition(Vector3(state(offset+5), state(offset+6), state(offset+7)));
  mFrame->setRelVel(Vector6(state(offset+8), state(offset+9), state(offset+10),
                            state(offset+11), state(offset+12), state(offset+13)));
}

void
FreeJoint::getState(Vector& state, unsigned offset) const
{
  Quaternion q = mFrame->getOrientation();
  state(offset+1) = q(1);
  state(offset+2) = q(2);
  state(offset+3) = q(3);
  state(offset+4) = q(4);
  
  Vector3 p = mFrame->getPosition();
  state(offset+5) = p(1);
  state(offset+6) = p(2);
  state(offset+7) = p(3);
  
  Vector6 v = mFrame->getRelVel();
  state(offset+8) = v(1);
  state(offset+9) = v(2);
  state(offset+10) = v(3);
  state(offset+11) = v(4);
  state(offset+12) = v(5);
  state(offset+13) = v(6);
}

void
FreeJoint::getStateDeriv(Vector& state, unsigned offset)
{
  Quaternion q = mFrame->getOrientation();
  Vector3 angVel = mFrame->getRelVel().getAngular();
  Vector3 vel = mFrame->rotToParent(mFrame->getRelVel().getLinear());

  // Compute the derivative term originating from the angular velocity.
  // Correction term to keep the quaternion normalized.
  // That is if |q| < 1 add a little radial component outward,
  // if |q| > 1 add a little radial component inward
  Vector4 qderiv = derivative(q, angVel) + 0.1*(normalize(q) - q);
  state(offset+1) = qderiv(1);
  state(offset+2) = qderiv(2);
  state(offset+3) = qderiv(3);
  state(offset+4) = qderiv(4);
  
  state(offset+5) = vel(1);
  state(offset+6) = vel(2);
  state(offset+7) = vel(3);
  
  Vector6 accel = mFrame->getRelVelDot();
  state(offset+8)  = accel(1);
  state(offset+9)  = accel(2);
  state(offset+10) = accel(3);
  state(offset+11) = accel(4);
  state(offset+12) = accel(5);
  state(offset+13) = accel(6);
}

} // namespace OpenFDM
