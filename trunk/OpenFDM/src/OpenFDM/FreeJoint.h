/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_FreeJoint_H
#define OpenFDM_FreeJoint_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Joint.h"
#include "Environment.h"

namespace OpenFDM {

class FreeJoint
  : public Joint {
public:
  FreeJoint(Environment* env, const std::string& name = std::string());
  virtual ~FreeJoint(void);

  /// HACK
  virtual bool isArticulatedJoint(void) const
  { return true; }
  virtual Frame* getInboardGroup(void)
  { return 0; }
  virtual const Frame* getInboardGroup(void) const
  { return 0; }
  virtual Frame* getOutboardGroup(void)
  { return getParentFrame(0); }
  virtual const Frame* getOutboardGroup(void) const
  { return getParentFrame(0); }
  virtual RigidBody* getOutboardBody(void)
  { return getParentRigidBody(0); }

  /** Set the relative velocity.
   */
  void setRelVel(const Vector6& vel)
  { Joint::setOutboardRelVel(vel); }
  /** Set the relative velocity.
   */
  void setLinearRelVel(const Vector3& vel)
  {
    FreeFrame* topBody = getOutboardBody()->getFreeFrame();
    if (!topBody)
      return;
    topBody->setLinearRelVel(vel);
  }
  /** Set the relative velocity.
   */
  void setAngularRelVel(const Vector3& vel)
  {
    FreeFrame* topBody = getOutboardBody()->getFreeFrame();
    if (!topBody)
      return;
    topBody->setAngularRelVel(vel);
  }

  /** Set the reference position.
   */
  void setRefPosition(const Vector3& p)
  {
    FreeFrame* topBody = getOutboardBody()->getFreeFrame();
    if (!topBody)
      return;
    topBody->setRefPosition(p);
  }
  /** Set the reference orientation.
   */
  void setRefOrientation(const Quaternion& o)
  {
    FreeFrame* topBody = getOutboardBody()->getFreeFrame();
    if (!topBody)
      return;
    topBody->setRefOrientation(o);
  }

  /** Sets the state of this multibody system from the state vector state
      and returns the time derivative in deriv.
   */
  void computeStateDeriv(real_type t, const Vector& state, Vector& deriv);

private:
  /** Plugin function for the articulated body algorithm.
   */
  virtual bool jointArticulation(SpatialInertia& artI, Vector6& artF);
  /** Plugin function for the articulated body algorithm.
   */
  virtual Vector6 computeRelAccel(const SpatialInertia& artI,
                                  const Vector6& artF);


  /** Plugin function for the state propagation.
   */
  virtual void setState(const Vector& state, unsigned offset);
  /** Plugin function for the state propagation.
   */
  virtual void getState(Vector& state, unsigned offset) const;
  /** Plugin function for the state propagation.
   */
  virtual void getStateDeriv(Vector& state, unsigned offset);

  /** Reference to the vehicles environment.
   */
  SharedPtr<Environment> mEnvironment;
};

} // namespace OpenFDM

#endif
