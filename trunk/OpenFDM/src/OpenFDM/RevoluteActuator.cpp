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
#include "RevoluteActuator.h"

namespace OpenFDM {

RevoluteActuator::RevoluteActuator(const std::string& name)
  : Joint(name)
{
}

RevoluteActuator::~RevoluteActuator(void)
{
}

void
RevoluteActuator::setJointAxis(const Vector3& axis)
{
  real_type nrm = norm(axis);
  if (nrm < Limits<real_type>::min()) {
    Log(Initialization, Error) << "JointAxis is zero ..." << endl;
    return;
  }

  mJointAxis = (1/nrm)*axis;
}

void
RevoluteActuator::setJointPos(real_type pos)
{
  mJointPos = pos;
  Quaternion q = mOrientation;
  q *= Quaternion::fromAngleAxis(mJointPos, mJointAxis);
  setOutboardOrientation(q);
}

void
RevoluteActuator::setJointVel(real_type vel)
{
  mJointVel = vel;
  setOutboardRelVel(mJointVel*getJointAxis());
}

void
RevoluteActuator::setPosition(const Vector3& position)
{
  setOutboardPosition(position);
}

void
RevoluteActuator::setOrientation(const Quaternion& orientation)
{
  mOrientation = orientation;
  Quaternion q = orientation;
  q *= Quaternion::fromAngleAxis(mJointPos, mJointAxis);
  setOutboardOrientation(q);
}

// bool
// RevoluteJoint::jointArticulation(SpatialInertia& artI, Vector6& artF)
// {
// }

Vector6
RevoluteActuator::computeRelAccel(void)
{
  return mJointAccel*getJointAxis();
}

void
RevoluteActuator::output(void)
{
  /*FIXME*/
}

void
RevoluteActuator::update(real_type h)
{
  Log(ArtBody, Debug) << __PRETTY_FUNCTION__ << " " << getName() << endl;

  if (mLocked) {
    mJointVel = 0;
    mJointAccel = 0;
    setJointPos(mJointPos);
    setJointVel(mJointVel);
    return;
  }

  real_type Pp = 1;
  real_type Pv = 1;

  if (mCtrlMode == PositionControl) {
    real_type posErr = mJointPos - mDesiredPos;
    mDesiredVel = Pp*posErr;
    mDesiredVel = max(mDesiredVel, mMinVel);
    mDesiredVel = min(mDesiredVel, mMaxVel);

    Log(ArtBody, Debug) << " posErr = " << posErr << endl;
  }
  real_type velErr = mJointVel - mDesiredVel;
  mJointAccel = Pv*velErr;
  mJointAccel = max(mJointAccel, mMinAccel);
  mJointAccel = min(mJointAccel, mMaxAccel);

  Log(ArtBody, Debug) << " velErr = " << velErr << endl;

  // Integrate
  mJointPos += h*mJointVel;
  mJointVel += h*mJointAccel;

  setJointPos(mJointPos);
  setJointVel(mJointVel);

  Log(ArtBody, Debug) << " vel = " << mJointVel << " accel = " << mJointAccel
                      << endl;
}

} // namespace OpenFDM
