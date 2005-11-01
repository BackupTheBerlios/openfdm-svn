/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RevoluteJoint_H
#define OpenFDM_RevoluteJoint_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Joint.h"
#include "JointT.h"

namespace OpenFDM {

class RevoluteJoint
  : public Joint, public JointT<1> {
public:
  RevoluteJoint(const std::string& name = std::string(), bool trackPosition = true);
  virtual ~RevoluteJoint(void);

  /** Gets the joint axis where this joint is allowed to rotate around.
   */
  Vector6 getJointAxis(void) const
  { return Vector6(mJointAxis, Vector3::zeros()); }

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

  /** Set the position of the joint.
   */
  void setPosition(const Vector3& position);

  /** Sets the zero orientation of the joint.
   */
  void setOrientation(const Quaternion& orientation);

  void setSpringConstant(real_type springConstant)
  { mSpringCoef = springConstant; }
  real_type getSpringConstant(void) const
  { return mSpringCoef; }
  void setDampConstant(real_type dampConstant)
  { mDampCoef = dampConstant; }
  real_type getDampConstant(void) const
  { return mDampCoef; }

  virtual real_type getJointForce(void) const
  {
    Log(ArtBody, Debug) << "RevoluteJoint " << getName()
                        << " pos " << convertTo(uDegree, mJointPosition)
                        << " vel " << mJointVelocity
                        << " torque " << mSpringCoef*mJointPosition + mDampCoef*mJointVelocity
                        << endl;
    return mSpringCoef*mJointPosition + mDampCoef*mJointVelocity;
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
  Quaternion mOrientation;

  /** Signals if we should track the joint position in the ordinary
      differential equation part of the multibody system.
      This is useful for wheel models where the position of the wheel is
      only used for animations and thus could be postprocessed.
   */
  bool mTrackPosition;

  /** Well, for now, just to test. Include these here
   */
  real_type mSpringCoef;
  real_type mDampCoef;
};

} // namespace OpenFDM

#endif
