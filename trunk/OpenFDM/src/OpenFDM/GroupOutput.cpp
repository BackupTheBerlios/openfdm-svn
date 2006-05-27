/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
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

void
GroupOutput::setParent(ModelGroup* modelGroup)
{
  if (modelGroup) {
    // attach to a ModelGroup
    mPortProxy->setPortProvider(new NumericPortProvider(modelGroup));
    mPortProxy->getPortProvider()->setName(getName());
    modelGroup->addOutputPort(mPortProxy->getPortProvider());
    
  } else {
    ModelGroup* oldParent = getParent();
    if (oldParent) {
      // detach from a ModelGroup
      oldParent->removeOutputPort(mPortProxy->getPortProvider());
    }
  }
  Model::setParent(modelGroup);
}

} // namespace OpenFDM
