/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
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

class Interact :
    public Model {
public:
  Interact(const std::string& name, unsigned numParents);
  virtual ~Interact(void);

  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the multibody system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

  /// This is the primary gate function which handles interaction
  virtual void interactWith(RigidBody* rigidBody) = 0;



private:
  bool attachTo(RigidBody* rigidBody, bool upstream);
  bool detachFrom(RigidBody* rigidBody);

  /// Called whenever the local topology chages, use to manage frames with this
  /// interact
  virtual void recheckTopology(void) {}

  friend class RigidBody;

public:

  const RigidBody* getParentRigidBody(unsigned id = 0) const
  {
    OpenFDMAssert(id < mParents.size());
    return mParents[id];
  }
  RigidBody* getParentRigidBody(unsigned id = 0)
  {
    OpenFDMAssert(id < mParents.size());
    return mParents[id];
  }
  void swapParents(void)
  {
    OpenFDMAssert(2 == mParents.size());
    RigidBody* rigidBody = mParents[0];
    mParents[0] = mParents[1];
    mParents[1] = rigidBody;
  }


  /// FIXME: hmm
  virtual bool updateAccels(RigidBody*) { return true; }

  /// FIXME remove
  const Frame* getParentFrame(unsigned id = 0) const
  {
    OpenFDMAssert(id < mParents.size() && mParents[id]);
    return mParents[id]->getFrame();
  }
  /// FIXME remove
  Frame* getParentFrame(unsigned id = 0)
  {
    OpenFDMAssert(id < mParents.size() && mParents[id]);
    return mParents[id]->getFrame();
  }

private:
  typedef std::vector<WeakPtr<RigidBody> > ParentList;
  ParentList mParents;
};

} // namespace OpenFDM

#endif
