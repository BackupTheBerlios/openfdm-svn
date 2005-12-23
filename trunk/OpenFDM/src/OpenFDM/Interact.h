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

namespace OpenFDM {

class Interact :
    public Model {
public:
  Interact(const std::string& name, unsigned numParents);
  virtual ~Interact(void);

  /// Double dispatch helper for the multibody system visitor
  virtual void accept(Visitor& visitor);
  /// Double dispatch helper for the multibody system visitor
  virtual void traverse(Visitor& visitor);
  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ConstVisitor& visitor) const;
  /// Double dispatch helper for the multibody system visitor
  virtual void traverse(ConstVisitor& visitor) const;


  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the multibody system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;



  bool attachTo(RigidBody* rigidBody);
  bool detachFrom(RigidBody* rigidBody);

  virtual void interactWith(RigidBody* rigidBody) = 0;

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
