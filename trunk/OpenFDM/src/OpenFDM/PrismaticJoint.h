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
#include "PrismaticJointFrame.h"

namespace OpenFDM {

class PrismaticJoint
  : public Joint {
public:
  PrismaticJoint(const std::string& name);
  virtual ~PrismaticJoint(void);

  virtual bool init(void)
  { recheckTopology(); return Joint::init(); }

  virtual void recheckTopology(void)
  {
    if (!getOutboardBody() || !getInboardBody())
      return;

    // check for the inboard frame
    Frame* inFrame = getInboardBody()->getFrame();
    if (!inFrame)
      return;

    Frame* outFrame = getOutboardBody()->getFrame();
    if (!outFrame) {
      getOutboardBody()->setFrame(mPrismaticJointFrame);
    }
    outFrame = getOutboardBody()->getFrame();
    if (!outFrame->isParentFrame(inFrame)) {
      inFrame->addChildFrame(mPrismaticJointFrame);
    }
  }

  /** Gets the joint axis where this joint is allowed to rotate around.
   */
  Vector6 getJointAxis(void) const
  { return Vector6(Vector3::zeros(), mPrismaticJointFrame->getJointAxis()); }

  /** Sets the joint axis where this joint is allowed to rotate around.
   */
  void setJointAxis(const Vector3& axis);

  /** Returns the joint position.
   */
  const real_type& getJointPos(void) const
  { return mPrismaticJointFrame->getJointPos(); }

  /** Sets the joint position.
   */
  void setJointPos(real_type pos);

  /** Returns the joint velocity.
   */
  const real_type& getJointVel(void) const
  { return mPrismaticJointFrame->getJointVel(); }

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

  /** The direct joint interaction force
   */
  SharedPtr<LineForce> mLineForce;

  /// The frame of the mobile root
  SharedPtr<PrismaticJointFrame> mPrismaticJointFrame;
};

} // namespace OpenFDM

#endif
