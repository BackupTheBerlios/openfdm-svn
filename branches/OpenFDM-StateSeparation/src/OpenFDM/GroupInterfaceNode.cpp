/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "GroupInterfaceNode.h"

#include "ConstNodeVisitor.h"
#include "LogStream.h"
#include "Model.h"
#include "NodeVisitor.h"
#include "TypeInfo.h"
#include "Variant.h"

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
  return mExternalPort->getIndex();
}

void
GroupInterfaceNode::setExternalPortName(const std::string& portName)
{
  mExternalPort->setName(portName);
}

const std::string&
GroupInterfaceNode::getExternalPortName() const
{
  return mExternalPort->getName();
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
GroupInterfaceNode::setExternalPort(Port* portInfo)
{
  mExternalPort = portInfo;
}

} // namespace OpenFDM
