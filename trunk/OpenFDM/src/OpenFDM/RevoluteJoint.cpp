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
#include "RevoluteJoint.h"

namespace OpenFDM {

RevoluteJoint::RevoluteJoint(const std::string& name, bool trackPosition)
  : Joint(name)
{
  setNumContinousStates(trackPosition ? 2 : 1);
  mTrackPosition = trackPosition;
  mJointPosition = 0;
  mJointVelocity = 0;
  mJointAcceleration = 0;
  mJointAxis = Vector3::unit(1);
  mOrientation = Quaternion::unit();

  setNumOutputPorts(2);
  setOutputPort(0, "jointPos", Property(this, &RevoluteJoint::getJointPos));
  setOutputPort(1, "jointVel", Property(this, &RevoluteJoint::getJointVel));
}

RevoluteJoint::~RevoluteJoint(void)
{
}

void
RevoluteJoint::setJointAxis(const Vector3& axis)
{
  real_type nrm = norm(axis);
  if (nrm < Limits<real_type>::min()) {
    Log(Initialization, Error) << "JointAxis is zero ..." << endl;
    return;
  }

  mJointAxis = (1/nrm)*axis;
}

void
RevoluteJoint::setJointPos(real_type pos)
{
  mJointPosition = pos;
  Quaternion q = mOrientation;
  q *= Quaternion::fromAngleAxis(mJointPosition, mJointAxis);
  setOutboardOrientation(q);
}

void
RevoluteJoint::setJointVel(real_type vel)
{
  mJointVelocity = vel;
  setOutboardRelVel(mJointVelocity*getJointAxis());
}

void
RevoluteJoint::setPosition(const Vector3& position)
{
  setOutboardPosition(position);
}

void
RevoluteJoint::setOrientation(const Quaternion& orientation)
{
  mOrientation = orientation;
  Quaternion q = orientation;
  q *= Quaternion::fromAngleAxis(mJointPosition, mJointAxis);
  setOutboardOrientation(q);
}

bool
RevoluteJoint::jointArticulation(SpatialInertia& artI, Vector6& artF)
{
  // That projects away tha components where the degrees of freedom
  // of the joint are.
  RigidBody* out = getOutboardGroup()->toRigidBody();
  real_type tau = getJointForce();
  return JointT<1>::jointArticulation(artI, artF, out->getPAlpha(),
                                      tau*getJointAxis(),
                                      getJointAxis());
}

Vector6
RevoluteJoint::computeRelAccel(const SpatialInertia&,
                               const Vector6&)
{
  RigidBody* out = getOutboardGroup()->toRigidBody();
  Vector6 parentAccel = out->getParentSpAccel();

  SpatialInertia artI = out->getArtInertia();
  Vector6 pAlpha = out->getPAlpha();

  JointT<1>::VectorN acc;
  JointT<1>::computeRelAccel(artI, parentAccel, pAlpha, getJointAxis(), acc);
  mJointAcceleration = acc(1);
  Log(ArtBody, Debug) << "Relative acceleration for Joint \""
                      << getName() << "\" is " << trans(acc) << endl;
  return getJointAxis()*acc;
}

void
RevoluteJoint::setState(const Vector& state, unsigned offset)
{
  if (mTrackPosition) {
    setJointPos(state(offset+1));
    setJointVel(state(offset+2));
  } else {
    setJointVel(state(offset+1));
  }
}

void
RevoluteJoint::getState(Vector& state, unsigned offset) const
{
  if (mTrackPosition) {
    state(offset+1) = mJointPosition;
    state(offset+2) = mJointVelocity;
  } else {
    state(offset+1) = mJointVelocity;
  }
}

void
RevoluteJoint::getStateDeriv(Vector& state, unsigned offset)
{
  if (mTrackPosition) {
    state(offset+1) = mJointVelocity;
    state(offset+2) = mJointAcceleration;
  } else {
    state(offset+1) = mJointAcceleration;
  }
}

} // namespace OpenFDM
