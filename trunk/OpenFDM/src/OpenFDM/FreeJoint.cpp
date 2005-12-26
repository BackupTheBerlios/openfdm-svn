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
#include "FreeJoint.h"

namespace OpenFDM {

FreeJoint::FreeJoint(Environment* env,const std::string& name)
  : Joint(name)
{
  setNumContinousStates(13);
  mEnvironment = env;
  addSampleTime(SampleTime::Continous);
}

FreeJoint::~FreeJoint(void)
{
}

bool
FreeJoint::jointArticulation(SpatialInertia& artI, Vector6& artF)
{
  artI = SpatialInertia::zeros();
  artF = Vector6::zeros();
}

Vector6
FreeJoint::computeRelAccel(const SpatialInertia& artI,
                           const Vector6& artF)
{
  RigidBody* topBody = getOutboardBody();
  if (!topBody)
    return Vector6::zeros();

  FreeFrame* frame = topBody->getFreeFrame();

  Log(ArtBody, Debug) << "FreeJoint::computeRelAccel():\n" << artI << endl;

  // Assumption: body is small compared to the distance to the planets
  // center of mass. That means gravity could be considered equal for the whole
  // vehicle.
  // See Featherstone, Orin: Equations and Algorithms
  Vector3 ga = mEnvironment->getGravity()->gravityAccel(frame->getRefPosition());
  Vector6 grav = Vector6(Vector3::zeros(), frame->rotFromRef(ga));

  Log(ArtBody, Debug) << "grav = " << trans(grav) << endl;
  Log(ArtBody, Debug) << "solve = " << trans(solve(artI, artF)) << endl;
  Log(ArtBody, Debug) << "parent spatial accel = " << trans(frame->getParentSpAccel()) << endl;
  Log(ArtBody, Debug) << "Hdot = " << trans(getHdot()) << endl;
  

  Vector6 accel = grav - solve(artI, artF)
    - frame->getParentSpAccel() - getHdot();
  return accel;
}

void
FreeJoint::setState(const Vector& state, unsigned offset)
{
  setOutboardOrientation(Vector4(state(offset+1), state(offset+2),
                         state(offset+3), state(offset+4)));
  setOutboardPosition(Vector3(state(offset+5), state(offset+6), state(offset+7)));
  setOutboardRelVel(Vector6(state(offset+8), state(offset+9), state(offset+10),
                            state(offset+11), state(offset+12), state(offset+13)));
}

void
FreeJoint::getState(Vector& state, unsigned offset) const
{
  const RigidBody* topBody = getParentRigidBody(0);
  if (!topBody)
    return;
  const Frame* frame = topBody->getFrame();
  Quaternion q = frame->getOrientation();
  state(offset+1) = q(1);
  state(offset+2) = q(2);
  state(offset+3) = q(3);
  state(offset+4) = q(4);
  
  Vector3 p = frame->getPosition();
  state(offset+5) = p(1);
  state(offset+6) = p(2);
  state(offset+7) = p(3);
  
  Vector6 v = frame->getRelVel();
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
  RigidBody* topBody = getOutboardBody();
  if (!topBody)
    return;
  FreeFrame* frame = topBody->getFreeFrame();
  Quaternion q = frame->getOrientation();
  Vector3 angVel = frame->getRelVel().getAngular();
  Vector3 vel = frame->rotToParent(frame->getRelVel().getLinear());

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
  
  Vector6 accel = frame->getRelAccel();
  state(offset+8)  = accel(1);
  state(offset+9)  = accel(2);
  state(offset+10) = accel(3);
  state(offset+11) = accel(4);
  state(offset+12) = accel(5);
  state(offset+13) = accel(6);
}

} // namespace OpenFDM
