/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Joint_H
#define OpenFDM_Joint_H

#include "Assert.h"
#include "Object.h"
#include "Frame.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Interact.h"
#include "Frame.h"
#include "LogStream.h"

namespace OpenFDM {

/// FIXME: joint's should be lockable, which means trylock == true and
/// velocity small enough - keep position ...

class ModelVisitor;

class Joint
  : public Interact {
public:
  Joint(const std::string& name);
  virtual ~Joint(void);

  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the multibody system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

  virtual const Joint* toJoint(void) const;
  virtual Joint* toJoint(void);

  virtual void output(const TaskInfo& taskInfo);

  RigidBody* getOutboardBody(void)
  { return getParentRigidBody(0); }
  RigidBody* getInboardBody(void)
  { return getParentRigidBody(1); }

  virtual void interactWith(RigidBody* rigidBody);

  // Joint slot ...
  virtual void jointArticulation(SpatialInertia& artI, Vector6& artF,
                                 const SpatialInertia& outI,
                                 const Vector6& outF) = 0;

};

} // namespace OpenFDM

#endif
