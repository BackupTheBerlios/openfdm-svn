/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MultiBodySystem_H
#define OpenFDM_MultiBodySystem_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Joint.h"
#include "FreeJoint.h"
#include "RootFrame.h"
#include "ModelGroup.h"

namespace OpenFDM {

class MultiBodySystem :
    public ModelGroup {
public:
  MultiBodySystem(RootFrame* rootFrame);
  virtual ~MultiBodySystem(void);

  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the multibody system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

//   virtual bool init(void);
  virtual void output(const TaskInfo& taskInfo);

  /// Add a RigidBody to that MultiBodySystem
  void addRigidBody(RigidBody* rigidBody);
  void removeRigidBody(RigidBody* rigidBody);

  /// Add an Interact to that MultiBodySystem
  void addInteract(Interact* interact);
  void removeInteract(Interact* interact);
private:
  /// At the moment each MultiBodySystem has its own root frame,
  /// In the future just store the root joint and reference a common root frame
  /// move that to environment
  SharedPtr<RootFrame> mRootFrame;

  /// That is the root for now ...
  SharedPtr<FreeJoint> mFreeJoint;

  /// A list of RigidBody objects in this MultiBodySystem
  typedef std::vector<SharedPtr<RigidBody> > RigidBodyList;
  RigidBodyList mRigidBodies;
};

} // namespace OpenFDM

#endif
