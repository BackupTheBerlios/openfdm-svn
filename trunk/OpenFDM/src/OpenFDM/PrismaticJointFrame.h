/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PrismaticJointFrame_H
#define OpenFDM_PrismaticJointFrame_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Joint.h"
#include "LineForce.h"
#include "CartesianJointFrame.h"

namespace OpenFDM {

class PrismaticJointFrame :
    public CartesianJointFrame<1> {
public:
  PrismaticJointFrame(const std::string& name) :
    CartesianJointFrame<1>(name),
    mZeroPos(Vector3::zeros()),
    mJointAxis(Vector3::unit(1)),
    mJointPos(0),
    mJointVel(0),
    mJointVelDot(0)
  { }
  virtual ~PrismaticJointFrame(void) {}

  /// Gets the joint axis where this joint is allowed to rotate around.
  const Vector3& getJointAxis(void) const
  { return mJointAxis; }

  /// Sets the joint axis where this joint is allowed to rotate around.
  void setJointAxis(const Vector3& axis)
  {
    mJointAxis = axis;
    setPosition(mZeroPos + mJointPos*mJointAxis);
    setLinearRelVel(mJointVel*mJointAxis);
    setLinearRelAccel(mJointVelDot*mJointAxis);
  }

  /// Returns the joint position.
  const real_type& getJointPos(void) const
  { return mJointPos; }

  /// Sets the joint position.
  void setJointPos(real_type pos)
  { mJointPos = pos; setPosition(mZeroPos + mJointPos*mJointAxis); }

  /// Returns the joint velocity.
  const real_type& getJointVel(void) const
  { return mJointVel; }

  /// Sets the joint velocity.
  void setJointVel(real_type vel)
  { mJointVel = vel; setLinearRelVel(mJointVel*mJointAxis); }

  /// Returns the derivative of the relative velocity
  const real_type& getJointVelDot(void) const
  { return mJointVelDot; }

  /// Returns the derivative of the relative velocity
  void setJointVelDot(real_type velDot)
  { mJointVelDot = velDot; setLinearRelAccel(mJointVelDot*mJointAxis); }

  /// Sets the zero position of the joint.
  void setZeroPosition(const Vector3& zeroPos)
  { mZeroPos = zeroPos; setPosition(mZeroPos + mJointPos*mJointAxis); }
  const Vector3& getZeroPosition(void) const
  { return mZeroPos; }

  using Frame::setOrientation;
  /// FIXME Hdot
  
private:
  /// The zero position with respect to the parent frame.
  Vector3 mZeroPos;
  /// The joint rotation axis.
  Vector3 mJointAxis;

  /// The relative joint translation along the joint axis
  real_type mJointPos;

  /// The realtive linear velocity along the joint axis
  real_type mJointVel;

  /// The realtive linear velocity derivative along the joint axis
  real_type mJointVelDot;
};

} // namespace OpenFDM

#endif
