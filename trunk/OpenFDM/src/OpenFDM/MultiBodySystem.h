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
#include "RootFrame.h"

namespace OpenFDM {

class MultiBodySystem :
    public Model {
public:
  MultiBodySystem(RootFrame* rootFrame);
  virtual ~MultiBodySystem(void);

  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the multibody system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

  void traverse(ModelVisitor& visitor)
  {
    InteractList::iterator it = mInteracts.begin();
    while (it != mInteracts.end()) {
      (*it)->accept(visitor);
      ++it;
    }
  }

  /** Sets the state of this multibody system from the state vector state.
   */
  void setEvalState(const Vector& state);
  /** Sets the state of this multibody system from the state vector state
      and returns the time derivative in deriv.
   */
  void computeStateDeriv(real_type t, const Vector& state, Vector& deriv);
  
  virtual void setState(const Vector& state, unsigned offset);
  virtual void getState(Vector& state, unsigned offset) const;
  virtual void getStateDeriv(Vector& stateDeriv, unsigned offset);

  virtual bool init(void);
  virtual void output(const TaskInfo& taskInfo);
  virtual void update(const TaskInfo& taskInfo);

  /// Add a RigidBody to that MultiBodySystem
  void addRigidBody(RigidBody* rigidBody);
  void removeRigidBody(RigidBody* rigidBody);

  /// Add an Interact to that MultiBodySystem
  void addInteract(Interact* interact);
  void removeInteract(Interact* interact);
private:
  /// At the moment each MultiBodySystem has its own root frame,
  /// In the future just store the root joint and reference a common root frame
  SharedPtr<RootFrame> mRootFrame;

  /// A list of RigidBody objects in this MultiBodySystem
  typedef std::vector<SharedPtr<RigidBody> > RigidBodyList;
  RigidBodyList mRigidBodies;

  /// A list of Interact objects in this MultiBodySystem
  typedef std::vector<SharedPtr<Interact> > InteractList;
  InteractList mInteracts;
};

} // namespace OpenFDM

#endif
