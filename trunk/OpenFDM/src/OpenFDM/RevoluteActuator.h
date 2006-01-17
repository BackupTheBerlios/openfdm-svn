/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
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

class RevoluteActuatorFrame;

class RevoluteActuator : public Joint {
  OPENFDM_OBJECT(RevoluteActuator, Joint);
public:
  RevoluteActuator(const std::string& name);
  virtual ~RevoluteActuator(void);

  virtual bool init(void)
  { recheckTopology(); return Joint::init(); }

  virtual void recheckTopology(void);

  /** Sets the joint axis where this joint is allowed to rotate around.
   */
  void setJointAxis(const Vector3& axis);

  /** Returns the joint position.
   */
  const real_type& getJointPos(void) const;

  /** Sets the joint position.
   */
  void setJointPos(real_type pos);

  /** Returns the joint velocity.
   */
  const real_type& getJointVel(void) const;

  /** Sets the joint velocity.
   */
  void setJointVel(real_type vel);

  /** Set the position of the joint.
   */
  void setPosition(const Vector3& position);

  /** Sets the zero orientation of the joint.
   */
  void setOrientation(const Quaternion& orientation);

  const real_type& getMaxVel(void) const
  { return mMaxVel; }
  void setMaxVel(real_type maxVel)
  { mMaxVel = maxVel; }

  const real_type& getVelGain(void) const
  { return mVelGain; }
  void setVelGain(real_type velGain)
  { mVelGain = velGain; }

  const real_type& getVelDotGain(void) const
  { return mVelDotGain; }
  void setVelDotGain(real_type velDotGain)
  { mVelDotGain = velDotGain; }

private:
  /** Computes the inboard articulated inertia and force for
      this articulated body. It is part of the articulated body algorithm.
   */
  virtual void jointArticulation(SpatialInertia& artI, Vector6& artF,
                                 const SpatialInertia& outI,
                                 const Vector6& outF);

  /** Methods for the OpenFDM::Part.
   */
  virtual void setState(const StateStream& state);
  virtual void getState(StateStream& state) const;
  virtual void getStateDeriv(StateStream& state);

  /// The maximum movement of the controler
  real_type mMaxVel;
  /// The velocity p gain
  real_type mVelGain;
  /// The velocity derivative p gain
  real_type mVelDotGain;

  /// The frame of the mobile root
  SharedPtr<RevoluteActuatorFrame> mRevoluteActuatorFrame;
};

} // namespace OpenFDM

#endif
