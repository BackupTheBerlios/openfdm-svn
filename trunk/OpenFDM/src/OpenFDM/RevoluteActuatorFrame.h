/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RevoluteActuatorFrame_H
#define OpenFDM_RevoluteActuatorFrame_H

#include "CartesianActuatorFrame.h"

namespace OpenFDM {

class RevoluteActuatorFrame : public CartesianActuatorFrame<1> {
  OPENFDM_OBJECT(RevoluteActuatorFrame, CartesianActuatorFrame<1>);
public:
  RevoluteActuatorFrame(const std::string& name) :
    CartesianActuatorFrame<1>(name),
    mZeroOrient(Quaternion::unit()),
    mJointAxis(Vector3::unit(0)),
    mJointPos(0),
    mJointVel(0)
  { }
  virtual ~RevoluteActuatorFrame(void) {}

  /// Gets the joint axis where this joint is allowed to rotate around.
  const Vector3& getJointAxis(void) const
  { return mJointAxis; }

  /// Sets the joint axis where this joint is allowed to rotate around.
  void setJointAxis(const Vector3& axis)
  {
    mJointAxis = axis;
    setOrientation(mZeroOrient*Quaternion::fromAngleAxis(mJointPos, mJointAxis));
    setAngularRelVel(mJointVel*mJointAxis);
  }

  /// Returns the joint position.
  const real_type& getJointPos(void) const
  { return mJointPos; }

  /// Sets the joint position.
  void setJointPos(const real_type& pos)
  {
    mJointPos = pos;
    setOrientation(mZeroOrient*Quaternion::fromAngleAxis(mJointPos, mJointAxis));
  }

  /// Returns the joint velocity.
  const real_type& getJointVel(void) const
  { return mJointVel; }

  /// Sets the joint velocity.
  void setJointVel(const real_type& vel)
  { mJointVel = vel; setAngularRelVel(mJointVel*mJointAxis); }

  /// Returns the joint acceleration.
  const real_type& getJointVelDot(void) const
  { return mJointVelDot; }

  /// Sets the joint velocity.
  void setJointVelDot(const real_type& velDot)
  { mJointVelDot = velDot; setAngularRelVelDot(mJointVelDot*mJointAxis); }

  /// Sets the zero orientation of the joint.
  void setZeroOrientation(const Quaternion& zeroOrient)
  {
    mZeroOrient = zeroOrient;
    setOrientation(zeroOrient*Quaternion::fromAngleAxis(mJointPos, mJointAxis));
  }
  const Quaternion& getZeroOrientation(void) const
  { return mZeroOrient; }

  using CartesianActuatorFrame<1>::setPosition;

private:
  /// The zero orientation with respect to the parent frame.
  Quaternion mZeroOrient;

  /// The joint rotation axis.
  Vector3 mJointAxis;

  /// The relative joint rotation with respect to the zero orientation.
  real_type mJointPos;

  /// The rotational velocity with respect to the rotation axis.
  real_type mJointVel;

  /// The rotational acceleration with respect to the rotation axis.
  real_type mJointVelDot;
};

} // namespace OpenFDM

#endif
