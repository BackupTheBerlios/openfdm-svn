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
#include "Joint.h"
#include "LineForce.h"

namespace OpenFDM {

class PrismaticJointFrame;

class PrismaticJoint
  : public Joint {
public:
  PrismaticJoint(const std::string& name);
  virtual ~PrismaticJoint(void);

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
    
    mLineForce->computeForce(getJointPos(), getJointVel());
    return mLineForce->getForce();
  }

private:
  /** Computes the inboard articulated inertia and force for
      this articulated body. It is part of the articulated body algorithm.
   */
  virtual void jointArticulation(SpatialInertia& artI, Vector6& artF,
                                 const SpatialInertia& outI,
                                 const Vector6& outF);

  /** Methods for the OpenFDM::Part.
   */
  virtual void setState(const Vector& state, unsigned offset);
  virtual void getState(Vector& state, unsigned offset) const;
  virtual void getStateDeriv(Vector& state, unsigned offset);

  /** The direct joint interaction force
   */
  SharedPtr<LineForce> mLineForce;

  /// The frame of the mobile root
  SharedPtr<PrismaticJointFrame> mPrismaticJointFrame;
};

} // namespace OpenFDM

#endif
