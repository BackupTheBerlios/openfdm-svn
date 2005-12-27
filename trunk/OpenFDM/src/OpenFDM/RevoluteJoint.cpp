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

RevoluteJoint::RevoluteJoint(const std::string& name) :
  Joint(name)
{
  setNumContinousStates(2);

  mRevoluteJointFrame = new RevoluteJointFrame(name);

  setNumOutputPorts(2);
  setOutputPort(0, "jointPos", this, &RevoluteJoint::getJointPos);
  setOutputPort(1, "jointVel", this, &RevoluteJoint::getJointVel);
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
  mRevoluteJointFrame->setJointAxis((1/nrm)*axis);
}

void
RevoluteJoint::setJointPos(real_type pos)
{
  mRevoluteJointFrame->setJointPos(pos);
}

void
RevoluteJoint::setJointVel(real_type vel)
{
  mRevoluteJointFrame->setJointVel(vel);
}

void
RevoluteJoint::setPosition(const Vector3& position)
{
  mRevoluteJointFrame->setPosition(position);
}

void
RevoluteJoint::setOrientation(const Quaternion& orientation)
{
  mRevoluteJointFrame->setZeroOrientation(orientation);
}

bool
RevoluteJoint::jointArticulation(SpatialInertia& artI, Vector6& artF)
{
  // That projects away tha components where the degrees of freedom
  // of the joint are.
  RigidBody* out = getOutboardBody();
  LinAlg::Vector<real_type,1> tau;
  tau(1) = getJointForce();
  return mRevoluteJointFrame->jointArticulation(artI, artF,
                                                out->getArtForce(),
                                                out->getArtInertia(),
                                                tau,
                                                getJointAxis());
}

Vector6
RevoluteJoint::computeRelVelDot(const SpatialInertia&,
                                const Vector6&)
{
  CartesianJointFrame<1>::VectorN acc;
  mRevoluteJointFrame->computeRelVelDot(getJointAxis(), acc);
  mRevoluteJointFrame->setJointVelDot(acc(1));
  
  Log(ArtBody, Debug) << "Relative acceleration for Joint \""
                      << getName() << "\" is " << trans(acc) << endl;
  return getJointAxis()*acc;
}

void
RevoluteJoint::setState(const Vector& state, unsigned offset)
{
  mRevoluteJointFrame->setJointPos(state(offset+1));
  mRevoluteJointFrame->setJointVel(state(offset+2));
}

void
RevoluteJoint::getState(Vector& state, unsigned offset) const
{
  state(offset+1) = mRevoluteJointFrame->getJointPos();
  state(offset+2) = mRevoluteJointFrame->getJointVel();
}

void
RevoluteJoint::getStateDeriv(Vector& state, unsigned offset)
{
  state(offset+1) = mRevoluteJointFrame->getJointVel();
  state(offset+2) = mRevoluteJointFrame->getJointVelDot();
}

} // namespace OpenFDM
