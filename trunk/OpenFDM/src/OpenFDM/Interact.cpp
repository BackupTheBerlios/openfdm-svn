/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
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
#include "Frame.h"
#include "RigidBody.h"
#include "ModelVisitor.h"
#include "Visitor.h"
#include "ConstVisitor.h"
#include "Interact.h"

namespace OpenFDM {

Interact::Interact(const std::string& name, unsigned numParents) :
  Model(name),
  mParents(numParents)
{
}

Interact::~Interact(void)
{
}

void
Interact::accept(Visitor& visitor)
{
  visitor.apply(*this);
}

void
Interact::traverse(Visitor& visitor)
{
}

void
Interact::accept(ConstVisitor& visitor) const
{
  visitor.apply(*this);
}

void
Interact::traverse(ConstVisitor& visitor) const
{
}

void
Interact::accept(ModelVisitor& visitor)
{
  visitor.apply(*this);
}

bool
Interact::attachTo(RigidBody* rigidBody)
{
  if (!rigidBody) {
    Log(MultiBody,Error) << "Got 0 RigidBody pointer to attach to." << endl;
    return false;
  }
  ParentList::iterator it;
  for (it = mParents.begin(); it != mParents.end(); ++it) {
    if (!(*it)) {
      (*it) = rigidBody;
      (*it)->addInteract(this);
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
      (*it)->removeInteract(this);
      (*it) = 0;
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
