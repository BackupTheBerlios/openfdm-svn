/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "GroupMechanicLink.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(GroupMechanicLink, GroupInterfaceNode)
  END_OPENFDM_OBJECT_DEF

GroupMechanicLink::GroupMechanicLink(const std::string& name) :
  GroupInterfaceNode(name),
  mGroupInternalPort(new MechanicLinkInfo(this, "link"))
{
}

GroupMechanicLink::~GroupMechanicLink()
{
}

bool
GroupMechanicLink::addParent(Node* parent)
{
  if (!GroupInterfaceNode::addParent(parent))
    return false;
  setExternalPortInfo(new MechanicLinkInfo(parent, "link"));
  return true;
}

} // namespace OpenFDM
