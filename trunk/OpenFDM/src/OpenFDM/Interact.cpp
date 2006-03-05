/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <iosfwd>
#include <list>
#include <string>

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "RigidBody.h"
#include "ModelVisitor.h"
#include "Interact.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Interact, Model)
  END_OPENFDM_OBJECT_DEF

Interact::Interact(const std::string& name, unsigned numParents) :
  Model(name),
  mParents(numParents)
{
}

Interact::~Interact(void)
{
}

const Interact*
Interact::toInteract(void) const
{
  return this;
}

Interact*
Interact::toInteract(void)
{
  return this;
}

void
Interact::accept(ModelVisitor& visitor)
{
  visitor.apply(*this);
}

bool
Interact::dependsDirectOn(Model* model)
{
  // HACK HACK HACK FIXME
  // non joint interacts need to have their inputs, but the outputs are
  // already present when the state is set ...
  // We need to split out sensors which could be scheduled earlier
  setDirectFeedThrough(true);
  if (Model::dependsDirectOn(model)) {
    setDirectFeedThrough(false);
    return true;
  }
  setDirectFeedThrough(false);
  
  return false;
}

bool
Interact::isChildOf(const RigidBody* const rigidBody) const
{
  if (!rigidBody)
    return false;
  ParentList::const_iterator it = mParents.begin();
  while (it != mParents.end()) {
    if ((*it) == rigidBody)
      return true;
    ++it;
  }
  return false;
}

bool
Interact::attachTo(RigidBody* rigidBody, bool upstream)
{
  if (!rigidBody) {
    Log(MultiBody,Error) << "Got 0 RigidBody pointer to attach to." << endl;
    return false;
  }
  ParentList::iterator it;
  for (it = mParents.begin(); it != mParents.end(); ++it) {
    if ((*it) == 0) {
      (*it) = rigidBody;
      if (upstream && it != mParents.begin())
        swapParents();
      recheckTopology();
      return true;
    }
  }
  
  Log(MultiBody,Error) << "Cannot attach Interact \"" << getName()
                       << "\" to RigidBody \"" << rigidBody->getName()
                       << "\": Already attached to 2 Rigid bodies."
                       << endl;
  return false;
}

bool
Interact::detachFrom(RigidBody* rigidBody)
{
  if (!rigidBody) {
    Log(MultiBody,Error) << "Got 0 RigidBody pointer to attach to." << endl;
    return false;
  }
  ParentList::iterator it;
  for (it = mParents.begin(); it != mParents.end(); ++it) {
    if ((*it) == rigidBody) {
      (*it) = 0;
      recheckTopology();
      return true;
    }
  }
  
  Log(MultiBody,Error) << "Cannot detatach Interact \"" << getName()
                       << "\" from RigidBody \"" << rigidBody->getName()
                       << "\": Interact is not attached to that RigidBody."
                       << endl;
  return false;
}

} // namespace OpenFDM
