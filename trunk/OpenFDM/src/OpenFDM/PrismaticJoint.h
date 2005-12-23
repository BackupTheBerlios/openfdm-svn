/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PrismaticJoint_H
#define OpenFDM_PrismaticJoint_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "JointT.h"
#include "Joint.h"
#include "LineForce.h"

namespace OpenFDM {

class PrismaticJoint
  : public Joint, public JointT<1> {
public:
  PrismaticJoint(const std::string& name = std::string());
  virtual ~PrismaticJoint(void);

  /** Gets the joint axis where this joint is allowed to rotate around.
   */
  Vector6 getJointAxis(void) const
  { return Vector6(Vector3::zeros(), mJointAxis); }

  /** Sets the joint axis where this joint is allowed to rotate around.
   */
  void setJointAxis(const Vector3& axis);

  /** Returns the joint position.
   */
  const real_type& getJointPos(void) const
  { return mJointPosition; }

  /** Sets the joint position.
   */
  void setJointPos(real_type pos);

  /** Returns the joint velocity.
   */
  const real_type& getJointVel(void) const
  { return mJointVelocity; }

  /** Sets the joint velocity.
   */
  void setJointVel(real_type vel);

  /** Set the orientation of the joint.
   */
  void setOrientation(const Quaternion& orientation);

  /** Sets the zero position of the joint.
   */
  void setPosition(const Vector3& position);

  const LineForce* getLineForce(void) const
  { return mLineForce; }
  LineForce* getLineForce(void)
  { return mLineForce; }
  void setLineForce(LineForce* lineForce)
  { mLineForce = lineForce; }

  real_type getJointForce(void)
  {
    if (!mLineForce)
      return 0;
    
    mLineForce->computeForce(mJointPosition, mJointVelocity);
    return mLineForce->getForce();
  }

private:
  /** Computes the inboard articulated inertia and force for
      this articulated body. It is part of the articulated body algorithm.
   */
  virtual bool jointArticulation(SpatialInertia& artI, Vector6& artF);

  /** Computes the relative acceleration of this body with respect to its
      parent. It is part of the articulated body algorithm.
   */
  virtual Vector6 computeRelAccel(const SpatialInertia& artI,
                                  const Vector6& artF);

  /** Methods for the OpenFDM::Part.
   */
  virtual void setState(const Vector& state, unsigned offset);
  virtual void getState(Vector& state, unsigned offset) const;
  virtual void getStateDeriv(Vector& state, unsigned offset);

  /** The joint rotation axis.
   */
  Vector3 mJointAxis;

  /** The relative joint rotation with respect to the zero orientation.
   */
  real_type mJointPosition;

  /** The rotational velocity with respect to the rotation axis.
   */
  real_type mJointVelocity;

  /** The rotational velocity with respect to the rotation axis.
   */
  real_type mJointAcceleration;

  /** The zero orientation with respect to the parent frame.
   */
  Vector3 mPosition;

  /** The direct joint interaction force
   */
  SharedPtr<LineForce> mLineForce;
};

} // namespace OpenFDM

#endif
