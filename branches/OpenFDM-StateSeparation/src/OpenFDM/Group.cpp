/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Group.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Group, Node)
  END_OPENFDM_OBJECT_DEF

GroupAcceptorNode::GroupAcceptorNode() :
  _groupInternalPort(new ProxyProviderPortInfo(this, "output"))
{
}

GroupProviderNode::GroupProviderNode() :
  _groupInternalPort(new ProxyAcceptorPortInfo(this, "input"))
{
}

} // namespace OpenFDM
