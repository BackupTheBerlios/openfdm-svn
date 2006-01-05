/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RevoluteJointFrame_H
#define OpenFDM_RevoluteJointFrame_H

#include "CartesianJointFrame.h"

namespace OpenFDM {

class RevoluteJointFrame :
    public CartesianJointFrame<1> {
public:
  RevoluteJointFrame(const std::string& name) :
    CartesianJointFrame<1>(name),
    mZeroOrient(Quaternion::unit()),
    mJointAxis(Vector3::unit(1)),
    mJointPos(0),
    mJointVel(0)
  { }
  virtual ~RevoluteJointFrame(void) {}

  /// Gets the joint axis where this joint is allowed to rotate around.
  const Vector3& getJointAxis(void) const
  { return mJointAxis; }

  /// Sets the joint axis where this joint is allowed to rotate around.
  void setJointAxis(const Vector3& axis)
  {
    mJointAxis = axis;
    setOrientation(mZeroOrient*Quaternion::fromAngleAxis(mJointPos, mJointAxis));
    setAngularRelVel(mJointVel*mJointAxis);

    setJointMatrix(Vector6(axis, Vector3::zeros()));
  }

  /// Returns the joint position.
  const real_type& getJointPos(void) const
  { return mJointPos; }

  /// Sets the joint position.
  void setJointPos(real_type pos)
  {
    mJointPos = pos;
    setOrientation(mZeroOrient*Quaternion::fromAngleAxis(mJointPos, mJointAxis));
  }

  /// Returns the joint velocity.
  const real_type& getJointVel(void) const
  { return mJointVel; }

  /// Sets the joint velocity.
  void setJointVel(real_type vel)
  { setAngularRelVel(mJointVel*mJointAxis); mJointVel = vel; }

  /// Sets the zero orientation of the joint.
  void setZeroOrientation(const Quaternion& zeroOrient)
  {
    setOrientation(zeroOrient*Quaternion::fromAngleAxis(mJointPos, mJointAxis));
    mZeroOrient = zeroOrient;
  }
  const Quaternion& getZeroOrientation(void) const
  { return mZeroOrient; }

  using CartesianJointFrame<1>::setPosition;

private:
  /// The zero orientation with respect to the parent frame.
  Quaternion mZeroOrient;

  /// The joint rotation axis.
  Vector3 mJointAxis;

  /// The relative joint rotation with respect to the zero orientation.
  real_type mJointPos;

  /// The rotational velocity with respect to the rotation axis.
  real_type mJointVel;
};

} // namespace OpenFDM

#endif
