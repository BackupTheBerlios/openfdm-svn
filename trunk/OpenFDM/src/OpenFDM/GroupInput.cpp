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

void
GroupInput::setParent(ModelGroup* modelGroup)
{
  if (modelGroup) {
    // attach to a ModelGroup
    mPortProxy = new NumericPortProxy(modelGroup, getOutputPort(0));
    mPortProxy->setName(getName());
    modelGroup->addInputPort(mPortProxy);
    
  } else {
    ModelGroup* oldParent = getParent();
    if (oldParent) {
      // detach from a ModelGroup
      oldParent->removeInputPort(mPortProxy);
      mPortProxy = 0;
    }
  }
  
  Model::setParent(modelGroup);
}

} // namespace OpenFDM
