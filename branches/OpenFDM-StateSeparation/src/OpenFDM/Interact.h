/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Interact_H
#define OpenFDM_Interact_H

#include <iosfwd>
#include <list>
#include <string>

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Model.h"

namespace OpenFDM {

class Interact : public Model {
  OPENFDM_OBJECT(Interact, Model);
public:
  Interact(const std::string& name, unsigned numParents);
  virtual ~Interact(void);

  virtual const Interact* toInteract(void) const;
  virtual Interact* toInteract(void);

  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the multibody system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

  /// This is the primary gate function which handles interaction
  virtual void interactWith(RigidBody* rigidBody) = 0;

  virtual bool dependsDirectOn(Model* model);

  SharedPtr<const RigidBody> getParentRigidBody(unsigned id = 0) const
  {
    OpenFDMAssert(id < mParents.size());
    return mParents[id].lock();
  }
  SharedPtr<RigidBody> getParentRigidBody(unsigned id = 0)
  {
    OpenFDMAssert(id < mParents.size());
    return mParents[id].lock();
  }
  bool isChildOf(const RigidBody* const rigidBody) const;

private:
  bool attachTo(RigidBody* rigidBody, bool upstream);
  bool detachFrom(RigidBody* rigidBody);

  /// Called whenever the local topology chages, use to manage frames with this
  /// interact
  virtual void recheckTopology(void) {}

  friend class RigidBody;

  void swapParents(void)
  {
    OpenFDMAssert(2 == mParents.size());
    mParents[0].swap(mParents[1]);
  }

private:
  typedef std::vector<WeakPtr<RigidBody> > ParentList;
  ParentList mParents;
};

} // namespace OpenFDM

#endif
