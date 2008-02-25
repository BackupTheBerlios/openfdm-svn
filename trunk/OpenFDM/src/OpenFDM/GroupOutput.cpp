/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "GroupOutput.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(GroupOutput, Model)
  END_OPENFDM_OBJECT_DEF

GroupOutput::GroupOutput(const std::string& name) :
  Model(name)
{
  mPortProxy = new NumericPortProxy(this, 0);
  mPortProxy->setName("input");
  addInputPort(mPortProxy);
}

GroupOutput::~GroupOutput()
{
}

unsigned
GroupOutput::addParent(ModelGroup* group)
{
  unsigned parentIndex = Model::addParent(group);
  if (parentIndex == ~0u)
    return parentIndex;

  // attach to a ModelGroup
  mPortProxy->setPortProvider(new NumericPortProvider(group));
  mPortProxy->getPortProvider()->setName(getName());
  group->addOutputPort(mPortProxy->getPortProvider());

  return parentIndex;
}

void
GroupOutput::removeParent(unsigned idx)
{
  SharedPtr<ModelGroup> oldParent = getParent(idx).lock();
  if (oldParent) {
    // detach from a ModelGroup
    oldParent->removeOutputPort(mPortProxy->getPortProvider());
  }

  Model::removeParent(idx);
}

} // namespace OpenFDM
