/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MobileRootJoint_H
#define OpenFDM_MobileRootJoint_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Planet.h"
#include "Joint.h"
#include "Environment.h"

namespace OpenFDM {

class ModelVisitor;
class MobileRootJointFrame;

class MobileRootJoint
  : public Joint {
public:
  MobileRootJoint(const std::string& name);
  virtual ~MobileRootJoint(void);

  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the multibody system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

  virtual bool init(void);
  virtual void output(const TaskInfo& taskInfo);

  virtual void recheckTopology(void);

  /// Gets the relative velocity.
  const Vector6& getRelVel(void) const;
  /// Set the relative velocity.
  void setRelVel(const Vector6& vel);
  /// Set the relative velocity.
  void setLinearRelVel(const Vector3& vel);
  /// Set the relative velocity.
  void setAngularRelVel(const Vector3& vel);

  /// Gets the relative velocity derivative.
  const Vector6& getRelVelDot(void) const;

  /// Get the reference position.
  const Vector3& getRefPosition(void) const;
  /// Set the reference position.
  void setRefPosition(const Vector3& p);

  /// Get the reference orientation.
  const Quaternion& getRefOrientation(void) const;
  /// Set the reference orientation.
  void setRefOrientation(const Quaternion& o);

  /// Get the geodetic position.
  Geodetic getGeodPosition(void) const;
  /// Set the geodetic position.
  void setGeodPosition(const Geodetic& geod);
  /// Get orientation wrt the geodetic hl frame.
  Quaternion getGeodOrientation(void) const;

  Vector4 getQDot(void) const;
  Vector3 getPosDot(void) const;

  /** Plugin function for the articulated body algorithm.
   */
  virtual void jointArticulation(SpatialInertia& artI, Vector6& artF,
                                 const SpatialInertia& outI,
                                 const Vector6& outF);


  /** Plugin function for the state propagation.
   */
  virtual void setState(const StateStream& state);
  /** Plugin function for the state propagation.
   */
  virtual void getState(StateStream& state) const;
  /** Plugin function for the state propagation.
   */
  virtual void getStateDeriv(StateStream& state);

private:
  /// The commonly used gravity model from the environment class
  /// It is initialized at the init() call
  SharedPtr<const Gravity> mGravity;

  /// The frame of the mobile root
  SharedPtr<MobileRootJointFrame> mFrame;
};

} // namespace OpenFDM

#endif
