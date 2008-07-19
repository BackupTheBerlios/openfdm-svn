/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_FixedRootJoint_H
#define OpenFDM_FixedRootJoint_H

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
class FixedRootJointFrame;

class FixedRootJoint
  : public Joint {
public:
  FixedRootJoint(const std::string& name);
  virtual ~FixedRootJoint(void);

  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the multibody system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

  virtual bool init(void);

  virtual void recheckTopology(void);

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

  /** Plugin function for the articulated body algorithm.
   */
  virtual void jointArticulation(SpatialInertia& artI, Vector6& artF,
                                 const SpatialInertia& outI,
                                 const Vector6& outF);


protected:
  virtual void setEnvironment(Environment* environment);

private:
  /// The commonly used gravity model from the environment class
  /// It is initialized at the init() call
  SharedPtr<const Gravity> mGravity;

  /// The frame of the mobile root
  SharedPtr<FixedRootJointFrame> mFrame;

  /// The environment pointer
  SharedPtr<Environment> mEnvironment;
};

} // namespace OpenFDM

#endif
