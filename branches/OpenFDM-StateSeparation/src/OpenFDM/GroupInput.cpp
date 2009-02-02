/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "GroupInput.h"

#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(GroupInput, GroupInterfaceNode)
  END_OPENFDM_OBJECT_DEF

GroupInput::GroupInput(const std::string& name) :
  GroupInterfaceNode(name),
  mGroupInternalPort(new OutputPort(this, "output", Size(0, 0), false))
{
}

GroupInput::~GroupInput()
{
}

bool
GroupInput::addParent(Node* parent)
{
  if (!GroupInterfaceNode::addParent(parent))
    return false;
  setExternalPort(new InputPort(parent, "input", Size(0, 0), false));
  return true;
}

} // namespace OpenFDM
