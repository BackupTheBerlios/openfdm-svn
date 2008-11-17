/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "GroupInterfaceNode.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(GroupInterfaceNode, Node)
  END_OPENFDM_OBJECT_DEF

GroupInterfaceNode::GroupInterfaceNode(const std::string& name) :
  Node(name)
{
}

GroupInterfaceNode::~GroupInterfaceNode()
{
}

void
GroupInterfaceNode::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
GroupInterfaceNode::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

unsigned
GroupInterfaceNode::getExternalPortIndex() const
{
  return mExternalPortInfo->getIndex();
}

bool
GroupInterfaceNode::addParent(Node* parent)
{
  if (getNumParents()) {
    Log(Model,Warning) << "Group Interface Nodes cannot have more than "
      "one parent!" << std::endl;
    return false;
  }
  
  return Node::addParent(parent);
}

void
GroupInterfaceNode::removeParent(Node* parent)
{
  Node::removeParent(parent);
}

void
GroupInterfaceNode::setExternalPortInfo(PortInfo* portInfo)
{
  mExternalPortInfo = portInfo;
}

} // namespace OpenFDM
