/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RevoluteActuator_H
#define OpenFDM_RevoluteActuator_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Joint.h"

namespace OpenFDM {

class RevoluteActuator
  : public Joint {
public:
  enum ControlMode {
    PositionControl,
    VelocityControl
  };

  RevoluteActuator(const std::string& name = std::string());
  virtual ~RevoluteActuator(void);

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
  { return mJointPos; }

  /** Sets the joint position.
   */
  void setJointPos(real_type pos);

  /** Returns the joint velocity.
   */
  real_type getJointVel(void) const
  { return mJointVel; }

  /** Sets the joint velocity.
   */
  void setJointVel(real_type vel);

  void setMaxVel(real_type vel)
  { mMaxVel = vel; }
  real_type getMinVel(void) const
  { return mMinVel; }
  void setMinVel(real_type vel)
  { mMinVel = vel; }
  real_type getMaxVel(void) const
  { return mMaxVel; }

  void setDesiredVel(real_type vel)
  { mDesiredVel = vel; }
  real_type getDesiredVel(void) const
  { return mDesiredVel; }

  void setDesiredPos(real_type pos)
  { mDesiredPos = pos; }
  real_type getDesiredPos(void) const
  { return mDesiredPos; }

  void setControlMode(ControlMode mode)
  { mCtrlMode = mode; }
  ControlMode getControlMode(void) const
  { return mCtrlMode; }

  void unlock(void)
  { mLocked = false; }
  void lock(void)
  { mLocked = true; }
  void setLocked(bool locked)
  { mLocked = locked; }
  bool getLocked(void) const
  { return mLocked; }

  /** Set the position of the joint.
   */
  void setPosition(const Vector3& position);

  /** Sets the zero orientation of the joint.
   */
  void setOrientation(const Quaternion& orientation);

private:
  /** Computes the inboard articulated inertia and force for
      this articulated body. It is part of the articulated body algorithm.
   */
//   virtual bool jointArticulation(SpatialInertia& artI, Vector6& artF);

  /** Computes the relative acceleration of this body with respect to its
      parent. It is part of the articulated body algorithm.
   */
  virtual Vector6 computeRelAccel(void);

  virtual void output(const TaskInfo&);
  virtual void update(const TaskInfo& taskInfo);

  /** The joint rotation axis.
   */
  Vector3 mJointAxis;

  /** The relative joint rotation with respect to the zero orientation.
   */
  real_type mJointPos;

  /** The rotational velocity with respect to the rotation axis.
   */
  real_type mJointVel;

  real_type mMaxVel;
  real_type mMinVel;
  real_type mMaxAccel;
  real_type mMinAccel;

  real_type mDesiredVel;
  real_type mDesiredPos;

  /** The rotational velocity with respect to the rotation axis.
   */
  real_type mJointAccel;

  /** The zero orientation with respect to the parent frame.
   */
  Quaternion mOrientation;

  ControlMode mCtrlMode;

  bool mLocked;
};

} // namespace OpenFDM

#endif
