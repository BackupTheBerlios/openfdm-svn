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
  FreeJoint(const std::string& name);
  virtual ~FreeJoint(void);

  virtual bool init(void);

  virtual void recheckTopology(void);

  /// Set the relative velocity.
  void setRelVel(const Vector6& vel)
  { mFrame->setRelVel(vel); }
  /// Set the relative velocity.
  void setLinearRelVel(const Vector3& vel)
  { mFrame->setLinearRelVel(vel); }
  /// Set the relative velocity.
  void setAngularRelVel(const Vector3& vel)
  { mFrame->setAngularRelVel(vel); }

  /// Set the reference position.
  void setRefPosition(const Vector3& p)
  { mFrame->setRefPosition(p); }
  /// Set the reference orientation.
  void setRefOrientation(const Quaternion& o)
  { mFrame->setRefOrientation(o); }

private:
  /** Plugin function for the articulated body algorithm.
   */
  virtual bool jointArticulation(SpatialInertia& artI, Vector6& artF,
                                 const SpatialInertia& outI,
                                 const Vector6& outF);
  /** Plugin function for the articulated body algorithm.
   */
  virtual void computeRelVelDot();


  /** Plugin function for the state propagation.
   */
  virtual void setState(const Vector& state, unsigned offset);
  /** Plugin function for the state propagation.
   */
  virtual void getState(Vector& state, unsigned offset) const;
  /** Plugin function for the state propagation.
   */
  virtual void getStateDeriv(Vector& state, unsigned offset);

  /// The commonly used gravity model from the environment class
  /// It is initialized at the init() call
  SharedPtr<const Gravity> mGravity;

  /// The frame of the mobile root
  SharedPtr<FreeFrame> mFrame;
};

} // namespace OpenFDM

#endif
