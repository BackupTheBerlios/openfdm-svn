/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "GroupInput.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(GroupInput, Model)
  END_OPENFDM_OBJECT_DEF

GroupInput::GroupInput(const std::string& name) :
  Model(name)
{
  setNumOutputPorts(1);
  getOutputPort(0)->setName("output");
}

GroupInput::~GroupInput()
{
}

unsigned
GroupInput::addParent(Group* group)
{
  unsigned parentIndex = Model::addParent(group);
  if (parentIndex == ~0u)
    return parentIndex;
    
  mPortProxy = new NumericPortProxy(group, getOutputPort(0));
  mPortProxy->setName(getName());
  group->addInputPort(mPortProxy);

  return parentIndex;
}

void
GroupInput::removeParent(unsigned idx)
{
  SharedPtr<Group> oldParent = getParent(idx).lock();
  if (oldParent) {
    // detach from a ModelGroup
    oldParent->removeInputPort(mPortProxy);
    mPortProxy = 0;
  }
  
  Model::removeParent(idx);
}

} // namespace OpenFDM
