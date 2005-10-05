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
  real_type getJointPos(void) const
  { return mJointPosition; }

  /** Sets the joint position.
   */
  void setJointPos(real_type pos);

  /** Returns the joint velocity.
   */
  real_type getJointVel(void) const
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

  virtual real_type getJointForce(void) const
//   { return 0; }
  { return -1e1*mJointPosition - 1e2*mJointVelocity; }

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
  virtual void setState(real_type t, const Vector& state, unsigned offset);
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
};

} // namespace OpenFDM

#endif
