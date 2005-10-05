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
#include "PrismaticJoint.h"

namespace OpenFDM {

PrismaticJoint::PrismaticJoint(const std::string& name)
  : Joint(name)
{
  setNumContinousStates(2);
  mJointPosition = 0;
  mJointVelocity = 0;
  mJointAcceleration = 0;
  mJointAxis = Vector3::unit(1);
  mPosition = Vector3::zeros();
}

PrismaticJoint::~PrismaticJoint(void)
{
}

void
PrismaticJoint::setJointAxis(const Vector3& axis)
{
  real_type nrm = norm(axis);
  if (nrm < Limits<real_type>::min()) {
    Log(Initialization, Error) << "JointAxis is zero ..." << endl;
    return;
  }

  mJointAxis = (1/nrm)*axis;
}

void
PrismaticJoint::setJointPos(real_type pos)
{
  mJointPosition = pos;
  setOutboardPosition(mPosition + mJointPosition*mJointAxis);
}

void
PrismaticJoint::setJointVel(real_type vel)
{
  mJointVelocity = vel;
  setOutboardRelVel(mJointVelocity*getJointAxis());
}

void
PrismaticJoint::setOrientation(const Quaternion& orientation)
{
  setOutboardPosition(mPosition + mJointPosition*mJointAxis);
}

void
PrismaticJoint::setPosition(const Vector3& position)
{
  mPosition = position;
  setOutboardPosition(mPosition + mJointPosition*mJointAxis);
}

bool
PrismaticJoint::jointArticulation(SpatialInertia& artI, Vector6& artF)
{
  // That projects away tha components where the degrees of freedom
  // of the joint are.
  real_type tau = getJointForce();
  return JointT<1>::jointArticulation(artI, artF, tau*getJointAxis(),
                                      getHdot(), getJointAxis());
}


Vector6
PrismaticJoint::computeRelAccel(const SpatialInertia& artI,
                                const Vector6& artF)
{
  JointT<1>::VectorN acc;
  JointT<1>::computeRelAccel(artI, artF, getJointAxis(), acc);
  mJointAcceleration = acc(1);
  return getJointAxis()*acc;
}

void
PrismaticJoint::setState(real_type t, const Vector& state, unsigned offset)
{
  setJointPos(state(offset+1));
  setJointVel(state(offset+2));
}

void
PrismaticJoint::getState(Vector& state, unsigned offset) const
{
  state(offset+1) = mJointPosition;
  state(offset+2) = mJointVelocity;
}

void
PrismaticJoint::getStateDeriv(Vector& state, unsigned offset)
{
  state(offset+1) = mJointVelocity;
  state(offset+2) = mJointAcceleration;
}

} // namespace OpenFDM
