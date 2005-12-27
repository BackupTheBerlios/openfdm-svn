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

  mPrismaticJointFrame = new PrismaticJointFrame(name);

  setNumOutputPorts(2);
  setOutputPort(0, "jointPos", this, &PrismaticJoint::getJointPos);
  setOutputPort(1, "jointVel", this, &PrismaticJoint::getJointVel);
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

  mPrismaticJointFrame->setJointAxis((1/nrm)*axis);
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

bool
PrismaticJoint::jointArticulation(SpatialInertia& artI, Vector6& artF)
{
  // That projects away tha components where the degrees of freedom
  // of the joint are.
  RigidBody* out = getOutboardBody();
  real_type tau = getJointForce();

  return mPrismaticJointFrame->jointArticulation(artI, artF,
                                                 out->getArtForce(),
                                                 out->getArtInertia(),
                                                 tau*getJointAxis(),
                                                 getJointAxis());
}

Vector6
PrismaticJoint::computeRelAccel(const SpatialInertia&,
                                const Vector6&)
{
  CartesianJointFrame<1>::VectorN acc;
  mPrismaticJointFrame->computeRelAccel(getJointAxis(), acc);
  mPrismaticJointFrame->setJointVelDot(acc(1));
  Log(ArtBody, Debug) << "Relative acceleration for Joint \""
                      << getName() << "\" is " << trans(acc) << endl;
  return getJointAxis()*acc;
}

void
PrismaticJoint::setState(const Vector& state, unsigned offset)
{
  mPrismaticJointFrame->setJointPos(state(offset+1));
  mPrismaticJointFrame->setJointVel(state(offset+2));
}

void
PrismaticJoint::getState(Vector& state, unsigned offset) const
{
  state(offset+1) = mPrismaticJointFrame->getJointPos();
  state(offset+2) = mPrismaticJointFrame->getJointVel();
}

void
PrismaticJoint::getStateDeriv(Vector& state, unsigned offset)
{
  state(offset+1) = mPrismaticJointFrame->getJointVel();
  state(offset+2) = mPrismaticJointFrame->getJointVelDot();
}

} // namespace OpenFDM
