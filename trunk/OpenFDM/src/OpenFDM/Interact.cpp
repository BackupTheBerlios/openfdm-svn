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
#include "RigidBody.h"
#include "ModelVisitor.h"
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
    if ((*it) == 0) {
      (*it) = rigidBody;
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
