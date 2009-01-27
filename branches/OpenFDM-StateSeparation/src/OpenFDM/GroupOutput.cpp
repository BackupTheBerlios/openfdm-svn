/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "GroupOutput.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(GroupOutput, GroupInterfaceNode)
  END_OPENFDM_OBJECT_DEF

GroupOutput::GroupOutput(const std::string& name) :
  GroupInterfaceNode(name),
  mGroupInternalPort(new InputPort(this, "input", Size(0, 0), false))
{
}

GroupOutput::~GroupOutput()
{
}

bool
GroupOutput::addParent(Node* parent)
{
  if (!GroupInterfaceNode::addParent(parent))
    return false;
  setExternalPort(new OutputPort(parent, "output", Size(0, 0), false));
  return true;
}

} // namespace OpenFDM
