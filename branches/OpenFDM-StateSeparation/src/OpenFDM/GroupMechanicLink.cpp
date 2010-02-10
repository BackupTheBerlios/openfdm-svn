/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "GroupMechanicLink.h"

#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(GroupMechanicLink, GroupInterfaceNode)
  END_OPENFDM_OBJECT_DEF

GroupMechanicLink::GroupMechanicLink(const std::string& name) :
  GroupInterfaceNode(name),
  mGroupInternalPort(new MechanicLink(this, "link"))
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
  setExternalPort(new MechanicLink(parent, "link"));
  return true;
}

} // namespace OpenFDM
