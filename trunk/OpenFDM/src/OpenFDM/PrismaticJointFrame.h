/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PrismaticJointFrame_H
#define OpenFDM_PrismaticJointFrame_H

#include "CartesianJointFrame.h"

namespace OpenFDM {

class PrismaticJointFrame : public CartesianJointFrame<1> {
  OPENFDM_OBJECT(PrismaticJointFrame, CartesianJointFrame<1>);
public:
  PrismaticJointFrame(const std::string& name) :
    CartesianJointFrame<1>(name),
    mZeroPos(Vector3::zeros()),
    mJointAxis(Vector3::unit(0)),
    mJointPos(0),
    mJointVel(0)
  {
    setJointMatrix(Vector6(Vector3::zeros(), mJointAxis));
  }
  virtual ~PrismaticJointFrame(void) {}

  /// Gets the joint axis where this joint is allowed to rotate around.
  const Vector3& getJointAxis(void) const
  { return mJointAxis; }

  /// Sets the joint axis where this joint is allowed to rotate around.
  void setJointAxis(const Vector3& axis)
  {
    mJointAxis = axis;
    setJointMatrix(Vector6(Vector3::zeros(), axis));
    setPosition(mZeroPos + mJointPos*mJointAxis);
    setLinearRelVel(mJointVel*mJointAxis);
  }

  /// Returns the joint position.
  const real_type& getJointPos(void) const
  { return mJointPos; }

  /// Sets the joint position.
  void setJointPos(const real_type& pos)
  { mJointPos = pos; setPosition(mZeroPos + mJointPos*mJointAxis); }

  /// Returns the joint velocity.
  const real_type& getJointVel(void) const
  { return mJointVel; }

  /// Sets the joint velocity.
  void setJointVel(const real_type& vel)
  { mJointVel = vel; setLinearRelVel(mJointVel*mJointAxis); }

  /// Sets the zero position of the joint.
  void setZeroPosition(const Vector3& zeroPos)
  { mZeroPos = zeroPos; setPosition(mZeroPos + mJointPos*mJointAxis); }
  const Vector3& getZeroPosition(void) const
  { return mZeroPos; }

  using CartesianJointFrame<1>::setOrientation;
  
private:
  /// The zero position with respect to the parent frame.
  Vector3 mZeroPos;

  /// The joint rotation axis.
  Vector3 mJointAxis;

  /// The relative joint translation along the joint axis
  real_type mJointPos;

  /// The realtive linear velocity along the joint axis
  real_type mJointVel;
};

} // namespace OpenFDM

#endif
